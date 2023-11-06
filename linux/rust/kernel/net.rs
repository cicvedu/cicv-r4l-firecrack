// SPDX-License-Identifier: GPL-2.0

//! Networking core.
//!
//! C headers: [`include/net/net_namespace.h`](../../../../include/linux/net/net_namespace.h),
//! [`include/linux/netdevice.h`](../../../../include/linux/netdevice.h),
//! [`include/linux/skbuff.h`](../../../../include/linux/skbuff.h).

use crate::{
    bindings, device,
    error::{code::ENOMEM, from_kernel_result},
    str::CStr,
    sync::UniqueArc,
    to_result,
    types::PointerWrapper,
    ARef, AlwaysRefCounted, Error, Result,
};
use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    pin::Pin,
    ptr::{addr_of, addr_of_mut, NonNull},
};
use macros::vtable;

#[cfg(CONFIG_NETFILTER)]
pub mod filter;

/// Wraps the kernel's `struct net_device`.
#[repr(transparent)]
pub struct Device(UnsafeCell<bindings::net_device>);

// SAFETY: Instances of `Device` are created on the C side. They are always refcounted.
unsafe impl AlwaysRefCounted for Device {
    fn inc_ref(&self) {
        // SAFETY: The existence of a shared reference means that the refcount is nonzero.
        unsafe { bindings::dev_hold(self.0.get()) };
    }

    unsafe fn dec_ref(obj: core::ptr::NonNull<Self>) {
        // SAFETY: The safety requirements guarantee that the refcount is nonzero.
        unsafe { bindings::dev_put(obj.cast().as_ptr()) };
    }
}

impl Device {
    /// # Safety
    ///
    /// The caller must ensure that `ptr` is valid and remains valid for the lifetime of the
    /// returned [`Device`] instance.
    pub(crate) unsafe fn from_ptr<'a>(ptr: *const bindings::net_device) -> &'a Device {
        // SAFETY: The safety requirements guarantee the validity of the dereference, while the
        // `Device` type being transparent makes the cast ok.
        unsafe { &*ptr.cast() }
    }

    /// Sets carrier.
    pub fn netif_carrier_on(&self) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::netif_carrier_on(self.0.get()) }
    }

    /// Clears carrier.
    pub fn netif_carrier_off(&self) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::netif_carrier_off(self.0.get()) }
    }

    /// Assigns Ethernet address to a net_device.
    pub fn eth_hw_addr_set(&self, addr: &[u8; 6]) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::eth_hw_addr_set(self.0.get(), addr as _) }
    }

    /// Returns the mtu of the device.
    pub fn mtu_get(&self) -> u32 {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of!((*self.0.get()).mtu).read() }
    }

    /// Sets the max mtu of the device.
    pub fn max_mtu_set(&self, max_mtu: u32) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of_mut!((*self.0.get()).max_mtu).write(max_mtu) };
    }

    /// Sets the minimum mtu of the device.
    pub fn min_mtu_set(&self, min_mtu: u32) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of_mut!((*self.0.get()).min_mtu).write(min_mtu) };
    }

    ///　Returns the flags of the device.
    pub fn flags_get(&self) -> u32 {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of!((*self.0.get()).flags).read() }
    }

    ///　Sets the priv_flags of the device.
    pub fn priv_flags_get(&self) -> u64 {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of!((*self.0.get()).priv_flags).read() }
    }

    ///　Returns the priv_flags of the device.
    pub fn priv_flags_set(&self, flags: u64) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { addr_of_mut!((*self.0.get()).priv_flags).write(flags) }
    }

    /// Reports the number of bytes queued to hardware.
    pub fn sent_queue(&self, bytes: u32) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::netdev_sent_queue(self.0.get(), bytes) }
    }

    /// Allows the upper layers to transmit.
    pub fn netif_start_queue(&self) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::netif_start_queue(self.0.get()) }
    }

    /// Stops the upper layers to transmit.
    pub fn netif_stop_queue(&self) {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        unsafe { bindings::netif_stop_queue(self.0.get()) }
    }

    /// Reports bytes and packets completed by device.
    pub fn completed_queue(&self, pkts: u32, bytes: u32) {
        unsafe { bindings::netdev_completed_queue(self.0.get(), pkts, bytes) }
    }

    /// Allocate an skbuff for rx on the device.
    /// with IP header placed at an aligned offset.
    pub fn alloc_skb_ip_align(&self, length: u32) -> Result<ARef<SkBuff>> {
        // SAFETY: The netdev is valid because the shared reference guarantees a nonzero refcount.
        let skb = unsafe { bindings::netdev_alloc_skb_ip_align(self.0.get(), length) };
        if skb.is_null() {
            Err(ENOMEM)
        } else {
            Ok(unsafe { &*(skb as *const SkBuff) }.into())
        }
    }
}

/// Registration structure for a network device.
pub struct Registration<T: DeviceOperations> {
    dev: *mut bindings::net_device,
    registered: bool,
    _p: PhantomData<T>,
}

impl<T: DeviceOperations> Registration<T> {
    /// Creates new instance of registration.
    pub fn try_new(parent: &dyn device::RawDevice) -> Result<Self> {
        // SAFETY: FFI call.
        let dev = unsafe { bindings::alloc_etherdev_mqs(0, 1, 1) };
        if dev.is_null() {
            Err(ENOMEM)
        } else {
            // SAFETY: `dev` was allocated during initialization and is guaranteed to be valid.
            unsafe { (*dev).dev.parent = parent.raw_device() }
            Ok(Registration {
                dev,
                registered: false,
                _p: PhantomData,
            })
        }
    }

    /// Returns a network device.
    /// A driver might configure the device before registration.
    pub fn dev_get(&self) -> ARef<Device> {
        unsafe { &*(self.dev as *const Device) }.into()
    }

    /// Register a network device.
    pub fn register(&mut self, data: T::Data) -> Result {
        // SAFETY: `dev` was allocated during initialization and is guaranteed to be valid.
        let ret = unsafe {
            (*self.dev).netdev_ops = Self::build_device_ops();

            // SAFETY: The C contract guarantees that `data` is available
            // for implementers of the net_device operations (no other C code accesses
            // it), so we know that there are no concurrent threads/CPUs accessing
            // it (it's not visible to any other Rust code).
            bindings::dev_set_drvdata(&mut (*self.dev).dev, data.into_pointer() as _);
            bindings::register_netdev(self.dev)
        };
        if ret != 0 {
            // SAFETY: `dev` was allocated during initialization and is guaranteed to be valid.
            unsafe { bindings::dev_set_drvdata(&mut (*self.dev).dev, core::ptr::null_mut()) }
            Err(Error::from_kernel_errno(ret))
        } else {
            self.registered = true;
            Ok(())
        }
    }
}

impl<T: DeviceOperations> Drop for Registration<T> {
    fn drop(&mut self) {
        // SAFETY: `dev` was allocated during initialization and guaranteed to be valid.
        unsafe {
            if self.registered {
                bindings::unregister_netdev(self.dev);
            }
            bindings::free_netdev(self.dev);
        }
    }
}

impl<T: DeviceOperations> Registration<T> {
    const DEVICE_OPS: bindings::net_device_ops = bindings::net_device_ops {
        ndo_init: None,
        ndo_uninit: None,
        ndo_open: if <T>::HAS_OPEN {
            Some(Self::open_callback)
        } else {
            None
        },
        ndo_stop: if <T>::HAS_STOP {
            Some(Self::stop_callback)
        } else {
            None
        },
        ndo_start_xmit: if <T>::HAS_START_XMIT {
            Some(Self::start_xmit_callback)
        } else {
            None
        },
        ndo_features_check: None,
        ndo_select_queue: None,
        ndo_change_rx_flags: None,
        ndo_set_rx_mode: None,
        ndo_set_mac_address: None,
        ndo_validate_addr: None,
        ndo_do_ioctl: None,
        ndo_eth_ioctl: None,
        ndo_siocbond: None,
        ndo_siocwandev: None,
        ndo_siocdevprivate: None,
        ndo_set_config: None,
        ndo_change_mtu: None,
        ndo_neigh_setup: None,
        ndo_tx_timeout: None,
        ndo_get_stats64: if <T>::HAS_GET_STATS64 {
            Some(Self::get_stats64_callback)
        } else {
            None
        },
        ndo_has_offload_stats: None,
        ndo_get_offload_stats: None,
        ndo_get_stats: None,
        ndo_vlan_rx_add_vid: None,
        ndo_vlan_rx_kill_vid: None,
        #[cfg(CONFIG_NET_POLL_CONTROLLER)]
        ndo_poll_controller: None,
        #[cfg(CONFIG_NET_POLL_CONTROLLER)]
        ndo_netpoll_setup: None,
        #[cfg(CONFIG_NET_POLL_CONTROLLER)]
        ndo_netpoll_cleanup: None,
        ndo_set_vf_mac: None,
        ndo_set_vf_vlan: None,
        ndo_set_vf_rate: None,
        ndo_set_vf_spoofchk: None,
        ndo_set_vf_trust: None,
        ndo_get_vf_config: None,
        ndo_set_vf_link_state: None,
        ndo_get_vf_stats: None,
        ndo_set_vf_port: None,
        ndo_get_vf_port: None,
        ndo_get_vf_guid: None,
        ndo_set_vf_guid: None,
        ndo_set_vf_rss_query_en: None,
        ndo_setup_tc: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_enable: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_disable: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_ddp_setup: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_ddp_done: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_ddp_target: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_get_hbainfo: None,
        #[cfg(CONFIG_FCOE)]
        ndo_fcoe_get_wwn: None,
        #[cfg(CONFIG_RFS_ACCEL)]
        ndo_rx_flow_steer: None,
        ndo_add_slave: None,
        ndo_del_slave: None,
        ndo_get_xmit_slave: None,
        ndo_sk_get_lower_dev: None,
        ndo_fix_features: None,
        ndo_set_features: None,
        ndo_neigh_construct: None,
        ndo_neigh_destroy: None,
        ndo_fdb_add: None,
        ndo_fdb_del: None,
        ndo_fdb_del_bulk: None,
        ndo_fdb_dump: None,
        ndo_fdb_get: None,
        ndo_bridge_setlink: None,
        ndo_bridge_getlink: None,
        ndo_bridge_dellink: None,
        ndo_change_carrier: None,
        ndo_get_phys_port_id: None,
        ndo_get_port_parent_id: None,
        ndo_get_phys_port_name: None,
        ndo_dfwd_add_station: None,
        ndo_dfwd_del_station: None,
        ndo_set_tx_maxrate: None,
        ndo_get_iflink: None,
        ndo_fill_metadata_dst: None,
        ndo_set_rx_headroom: None,
        ndo_bpf: None,
        ndo_xdp_xmit: None,
        ndo_xdp_get_xmit_slave: None,
        ndo_xsk_wakeup: None,
        ndo_get_devlink_port: None,
        ndo_tunnel_ctl: None,
        ndo_get_peer_dev: None,
        ndo_fill_forward_path: None,
        ndo_get_tstamp: None,
    };

    const fn build_device_ops() -> &'static bindings::net_device_ops {
        &Self::DEVICE_OPS
    }

    unsafe extern "C" fn open_callback(netdev: *mut bindings::net_device) -> core::ffi::c_int {
        from_kernel_result! {
            // SAFETY: The C API guarantees that `net_device` isn't released while this function is running.
            let dev = unsafe { Device::from_ptr(netdev) };
            // SAFETY: The value stored as driver data was returned by `into_pointer` during registration.
            let data = unsafe { T::Data::borrow(bindings::dev_get_drvdata(&mut (*netdev).dev)) };
            T::open(dev, data)?;
            Ok(0)
        }
    }

    unsafe extern "C" fn stop_callback(netdev: *mut bindings::net_device) -> core::ffi::c_int {
        from_kernel_result! {
            // SAFETY: The C API guarantees that `net_device` isn't released while this function is running.
            let dev = unsafe { Device::from_ptr(netdev) };
            // SAFETY: The value stored as driver data was returned by `into_pointer` during registration.
            let data = unsafe { T::Data::borrow(bindings::dev_get_drvdata(&mut (*netdev).dev)) };
            T::stop(dev, data)?;
            Ok(0)
        }
    }

    unsafe extern "C" fn start_xmit_callback(
        skb: *mut bindings::sk_buff,
        netdev: *mut bindings::net_device,
    ) -> bindings::netdev_tx_t {
        // SAFETY: The C API guarantees that `net_device` isn't released while this function is running.
        let dev = unsafe { Device::from_ptr(netdev) };
        // SAFETY: The C API guarantees that `sk_buff` isn't released while this function is running.
        let skb = unsafe { SkBuff::from_ptr(skb) };
        // SAFETY: The value stored as driver data was returned by `into_pointer` during registration.
        let data = unsafe { T::Data::borrow(bindings::dev_get_drvdata(&mut (*netdev).dev)) };
        T::start_xmit(skb, dev, data) as bindings::netdev_tx_t
    }

    unsafe extern "C" fn get_stats64_callback(
        netdev: *mut bindings::net_device,
        storage: *mut bindings::rtnl_link_stats64,
    ) {
        // SAFETY: The C API guarantees that `net_device` isn't released while this function is running.
        let dev = unsafe { Device::from_ptr(netdev) };
        // SAFETY: The value stored as driver data was returned by `into_pointer` during registration.
        let data = unsafe { T::Data::borrow(bindings::dev_get_drvdata(&mut (*netdev).dev)) };

        T::get_stats64(dev, data, &mut RtnlLinkStats64 { ptr: storage });
    }
}

/// Corresponds to the kernel's `struct rtnl_link_stats64`.
pub struct RtnlLinkStats64 {
    ptr: *mut bindings::rtnl_link_stats64,
}

impl RtnlLinkStats64 {
    /// Set rx_bytes.
    pub fn set_rx_bytes(&mut self, value: u64) {
        // SAFETY: By the type invariants, `self.ptr` is valid.
        unsafe { (*self.ptr).rx_bytes = value }
    }

    /// Set rx_packets.
    pub fn set_rx_packets(&mut self, value: u64) {
        // SAFETY: By the type invariants, `self.ptr` is valid.
        unsafe { (*self.ptr).rx_packets = value }
    }

    /// Set tx_bytes.
    pub fn set_tx_bytes(&mut self, value: u64) {
        // SAFETY: By the type invariants, `self.ptr` is valid.
        unsafe { (*self.ptr).tx_bytes = value }
    }

    /// Set tx_packets.
    pub fn set_tx_packets(&mut self, value: u64) {
        // SAFETY: By the type invariants, `self.ptr` is valid.
        unsafe { (*self.ptr).tx_packets = value }
    }
}

/// Driver transmit return codes.
#[repr(i32)]
pub enum NetdevTx {
    /// Driver took care of packet.
    Ok = bindings::netdev_tx_NETDEV_TX_OK,
    /// Driver tx path was busy.
    Busy = bindings::netdev_tx_NETDEV_TX_BUSY,
}

// SAFETY: `Registration` does not expose any of its state across threads.
unsafe impl<T: DeviceOperations> Sync for Registration<T> {}

// SAFETY: `Registration` is not restricted to a single thread,
// its `T::Data` is also `Send` so it may be moved to different threads.
#[allow(clippy::non_send_fields_in_send_ty)]
unsafe impl<T: DeviceOperations> Send for Registration<T> {}

/// Corresponds to the kernel's `struct net_device_ops`.
#[vtable]
pub trait DeviceOperations {
    /// The pointer type that will be used to hold driver-defined data type.
    type Data: PointerWrapper + Send + Sync = ();

    /// Corresponds to `ndo_open` in `struct net_device_ops`.
    fn open(dev: &Device, data: <Self::Data as PointerWrapper>::Borrowed<'_>) -> Result;

    /// Corresponds to `ndo_stop` in `struct net_device_ops`.
    fn stop(dev: &Device, data: <Self::Data as PointerWrapper>::Borrowed<'_>) -> Result;

    /// Corresponds to `ndo_start_xmit` in `struct net_device_ops`.
    fn start_xmit(
        skb: &SkBuff,
        dev: &Device,
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
    ) -> NetdevTx;

    /// Corresponds to `ndo_get_stats64` in `struct net_device_ops`.
    fn get_stats64(
        _dev: &Device,
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _storage: &mut RtnlLinkStats64,
    ) {
    }
}

/// Wraps the kernel's `struct napi_struct`.
#[repr(transparent)]
pub struct Napi(UnsafeCell<bindings::napi_struct>);

impl Napi {
    unsafe fn from_ptr<'a>(ptr: *const bindings::napi_struct) -> &'a Napi {
        // SAFETY: The safety requirements guarantee the validity of the dereference, while the
        // `Napi` type being transparent makes the cast ok.
        unsafe { &*ptr.cast() }
    }

    fn new() -> Self {
        Napi(UnsafeCell::new(bindings::napi_struct::default()))
    }

    /// Enable NAPI scheduling.
    pub fn enable(&self) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            bindings::napi_enable(self.0.get());
        }
    }

    /// Schedule NAPI poll routine to be called if it is not already running.
    pub fn schedule(&self) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            if bindings::napi_schedule_prep(self.0.get()) {
                bindings::__napi_schedule(self.0.get())
            }
        }
    }

    /// Marks NAPI processing as complete.
    pub fn complete_done(&self, work_done: i32) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            bindings::napi_complete_done(self.0.get(), work_done);
        }
    }

    /// Sends the skb up the stack.
    pub fn gro_receive(&self, skb: &SkBuff) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            bindings::napi_gro_receive(self.0.get(), skb.0.get());
        }
    }

    /// Returns a network device.
    pub fn dev_get(&self) -> ARef<Device> {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { &*(addr_of!((*self.0.get()).dev).read() as *const Device) }.into()
    }
}

/// Adapter for initializing `Napi` object.
pub struct NapiAdapter<T: NapiPoller>(PhantomData<T>);

impl<T: NapiPoller> NapiAdapter<T> {
    /// Creates a new Napi object.
    pub fn add_weight(dev: &Device, weight: i32) -> Result<Pin<UniqueArc<Napi>>> {
        let mut napi = Pin::from(UniqueArc::try_new(Napi::new())?);

        unsafe {
            bindings::netif_napi_add_weight(
                &*dev as *const Device as *mut bindings::net_device,
                napi.as_mut().0.get(),
                Some(Self::poll_callback),
                weight,
            )
        }
        Ok(napi)
    }

    unsafe extern "C" fn poll_callback(
        ptr: *mut bindings::napi_struct,
        budget: core::ffi::c_int,
    ) -> core::ffi::c_int {
        let netdev = unsafe { (*ptr).dev };
        let p = unsafe { bindings::dev_get_drvdata(&mut (*netdev).dev) };
        let data = unsafe { T::Data::borrow(p) };
        let napi = unsafe { Napi::from_ptr(ptr) };
        T::poll(napi, budget, unsafe { Device::from_ptr(netdev) }, data)
    }
}

/// Trait for NAPI poll method.
pub trait NapiPoller {
    /// The pointer type that will be used to hold driver-defined data type.
    /// This must be same as DeviceOperations::Data.
    type Data: PointerWrapper + Send + Sync = ();

    /// Corresponds to NAPI poll method.
    fn poll(
        napi: &Napi,
        budget: i32,
        dev: &Device,
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
    ) -> i32;
}

/// Wraps the kernel's `struct net`.
#[repr(transparent)]
pub struct Namespace(UnsafeCell<bindings::net>);

impl Namespace {
    /// Finds a network device with the given name in the namespace.
    pub fn dev_get_by_name(&self, name: &CStr) -> Option<ARef<Device>> {
        // SAFETY: The existence of a shared reference guarantees the refcount is nonzero.
        let ptr =
            NonNull::new(unsafe { bindings::dev_get_by_name(self.0.get(), name.as_char_ptr()) })?;
        Some(unsafe { ARef::from_raw(ptr.cast()) })
    }
}

// SAFETY: Instances of `Namespace` are created on the C side. They are always refcounted.
unsafe impl AlwaysRefCounted for Namespace {
    fn inc_ref(&self) {
        // SAFETY: The existence of a shared reference means that the refcount is nonzero.
        unsafe { bindings::get_net(self.0.get()) };
    }

    unsafe fn dec_ref(obj: core::ptr::NonNull<Self>) {
        // SAFETY: The safety requirements guarantee that the refcount is nonzero.
        unsafe { bindings::put_net(obj.cast().as_ptr()) };
    }
}

/// Returns the network namespace for the `init` process.
pub fn init_ns() -> &'static Namespace {
    unsafe { &*core::ptr::addr_of!(bindings::init_net).cast() }
}

/// Wraps the kernel's `struct sk_buff`.
#[repr(transparent)]
pub struct SkBuff(UnsafeCell<bindings::sk_buff>);

impl SkBuff {
    /// Creates a reference to an [`SkBuff`] from a valid pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `ptr` is valid and remains valid for the lifetime of the
    /// returned [`SkBuff`] instance.
    pub unsafe fn from_ptr<'a>(ptr: *const bindings::sk_buff) -> &'a SkBuff {
        // SAFETY: The safety requirements guarantee the validity of the dereference, while the
        // `SkBuff` type being transparent makes the cast ok.
        unsafe { &*ptr.cast() }
    }

    /// Returns the remaining data in the buffer's first segment.
    pub fn head_data(&self) -> &[u8] {
        // SAFETY: The existence of a shared reference means that the refcount is nonzero.
        let headlen = unsafe { bindings::skb_headlen(self.0.get()) };
        let len = headlen.try_into().unwrap_or(usize::MAX);
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        let data = unsafe { core::ptr::addr_of!((*self.0.get()).data).read() };
        // SAFETY: The `struct sk_buff` conventions guarantee that at least `skb_headlen(skb)` bytes
        // are valid from `skb->data`.
        unsafe { core::slice::from_raw_parts(data, len) }
    }

    /// Returns the total length of the data (in all segments) in the skb.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> u32 {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { core::ptr::addr_of!((*self.0.get()).len).read() }
    }

    /// Returns the current the data length (`struct sk_buff::data_len`).
    pub fn data_len(&self) -> u32 {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { core::ptr::addr_of!((*self.0.get()).data_len).read() }
    }

    /// Returns the packet's protocol ID.
    pub fn eth_type_trans(&self, dev: &Device) -> u16 {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { bindings::eth_type_trans(self.0.get(), dev.0.get()) }
    }

    /// Increase size and pad an skbuff up to the length
    pub fn put_padto(&self, len: u32) -> i32 {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { bindings::skb_put_padto(self.0.get(), len) }
    }

    /// Consuming the skb in NAPI context
    pub fn napi_consume(&self, budget: i32) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe { bindings::napi_consume_skb(self.0.get(), budget) }
    }

    /// Extends the used data area of the buffer.
    pub fn put(&self, len: u32) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            bindings::skb_put(self.0.get(), len);
        }
    }

    /// Set the protocol ID in the skb.
    pub fn protocol_set(&self, protocol: u16) {
        // SAFETY: The existence of a shared reference means `self.0` is valid.
        unsafe {
            addr_of_mut!((*self.0.get()).__bindgen_anon_5.headers.as_mut().protocol).write(protocol)
        }
    }
}

// SAFETY: Instances of `SkBuff` are created on the C side. They are always refcounted.
unsafe impl AlwaysRefCounted for SkBuff {
    fn inc_ref(&self) {
        // SAFETY: The existence of a shared reference means that the refcount is nonzero.
        unsafe { bindings::skb_get(self.0.get()) };
    }

    unsafe fn dec_ref(obj: core::ptr::NonNull<Self>) {
        // SAFETY: The safety requirements guarantee that the refcount is nonzero.
        unsafe {
            bindings::kfree_skb_reason(
                obj.cast().as_ptr(),
                bindings::skb_drop_reason_SKB_DROP_REASON_NOT_SPECIFIED,
            )
        };
    }
}

/// An IPv4 address.
///
/// This is equivalent to C's `in_addr`.
#[repr(transparent)]
pub struct Ipv4Addr(bindings::in_addr);

impl Ipv4Addr {
    /// A wildcard IPv4 address.
    ///
    /// Binding to this address means binding to all IPv4 addresses.
    pub const ANY: Self = Self::new(0, 0, 0, 0);

    /// The IPv4 loopback address.
    pub const LOOPBACK: Self = Self::new(127, 0, 0, 1);

    /// The IPv4 broadcast address.
    pub const BROADCAST: Self = Self::new(255, 255, 255, 255);

    /// Creates a new IPv4 address with the given components.
    pub const fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self(bindings::in_addr {
            s_addr: u32::from_be_bytes([a, b, c, d]).to_be(),
        })
    }
}

/// An IPv6 address.
///
/// This is equivalent to C's `in6_addr`.
#[repr(transparent)]
pub struct Ipv6Addr(bindings::in6_addr);

impl Ipv6Addr {
    /// A wildcard IPv6 address.
    ///
    /// Binding to this address means binding to all IPv6 addresses.
    pub const ANY: Self = Self::new(0, 0, 0, 0, 0, 0, 0, 0);

    /// The IPv6 loopback address.
    pub const LOOPBACK: Self = Self::new(0, 0, 0, 0, 0, 0, 0, 1);

    /// Creates a new IPv6 address with the given components.
    #[allow(clippy::too_many_arguments)]
    pub const fn new(a: u16, b: u16, c: u16, d: u16, e: u16, f: u16, g: u16, h: u16) -> Self {
        Self(bindings::in6_addr {
            in6_u: bindings::in6_addr__bindgen_ty_1 {
                u6_addr16: [
                    a.to_be(),
                    b.to_be(),
                    c.to_be(),
                    d.to_be(),
                    e.to_be(),
                    f.to_be(),
                    g.to_be(),
                    h.to_be(),
                ],
            },
        })
    }
}

/// A socket address.
///
/// It's an enum with either an IPv4 or IPv6 socket address.
pub enum SocketAddr {
    /// An IPv4 socket address.
    V4(SocketAddrV4),

    /// An IPv6 socket address.
    V6(SocketAddrV6),
}

/// An IPv4 socket address.
///
/// This is equivalent to C's `sockaddr_in`.
#[repr(transparent)]
pub struct SocketAddrV4(bindings::sockaddr_in);

impl SocketAddrV4 {
    /// Creates a new IPv4 socket address.
    pub const fn new(addr: Ipv4Addr, port: u16) -> Self {
        Self(bindings::sockaddr_in {
            sin_family: bindings::AF_INET as _,
            sin_port: port.to_be(),
            sin_addr: addr.0,
            __pad: [0; 8],
        })
    }
}

/// An IPv6 socket address.
///
/// This is equivalent to C's `sockaddr_in6`.
#[repr(transparent)]
pub struct SocketAddrV6(bindings::sockaddr_in6);

impl SocketAddrV6 {
    /// Creates a new IPv6 socket address.
    pub const fn new(addr: Ipv6Addr, port: u16, flowinfo: u32, scopeid: u32) -> Self {
        Self(bindings::sockaddr_in6 {
            sin6_family: bindings::AF_INET6 as _,
            sin6_port: port.to_be(),
            sin6_addr: addr.0,
            sin6_flowinfo: flowinfo,
            sin6_scope_id: scopeid,
        })
    }
}

/// A socket listening on a TCP port.
///
/// # Invariants
///
/// The socket pointer is always non-null and valid.
pub struct TcpListener {
    pub(crate) sock: *mut bindings::socket,
}

// SAFETY: `TcpListener` is just a wrapper for a kernel socket, which can be used from any thread.
unsafe impl Send for TcpListener {}

// SAFETY: `TcpListener` is just a wrapper for a kernel socket, which can be used from any thread.
unsafe impl Sync for TcpListener {}

impl TcpListener {
    /// Creates a new TCP listener.
    ///
    /// It is configured to listen on the given socket address for the given namespace.
    pub fn try_new(ns: &Namespace, addr: &SocketAddr) -> Result<Self> {
        let mut socket = core::ptr::null_mut();
        let (pf, addr, addrlen) = match addr {
            SocketAddr::V4(addr) => (
                bindings::PF_INET,
                addr as *const _ as _,
                core::mem::size_of::<bindings::sockaddr_in>(),
            ),
            SocketAddr::V6(addr) => (
                bindings::PF_INET6,
                addr as *const _ as _,
                core::mem::size_of::<bindings::sockaddr_in6>(),
            ),
        };

        // SAFETY: The namespace is valid and the output socket pointer is valid for write.
        to_result(unsafe {
            bindings::sock_create_kern(
                ns.0.get(),
                pf as _,
                bindings::sock_type_SOCK_STREAM as _,
                bindings::IPPROTO_TCP as _,
                &mut socket,
            )
        })?;

        // INVARIANT: The socket was just created, so it is valid.
        let listener = Self { sock: socket };

        // SAFETY: The type invariant guarantees that the socket is valid, and `addr` and `addrlen`
        // were initialised based on valid values provided in the address enum.
        to_result(unsafe { bindings::kernel_bind(socket, addr, addrlen as _) })?;

        // SAFETY: The socket is valid per the type invariant.
        to_result(unsafe { bindings::kernel_listen(socket, bindings::SOMAXCONN as _) })?;

        Ok(listener)
    }

    /// Accepts a new connection.
    ///
    /// On success, returns the newly-accepted socket stream.
    ///
    /// If no connection is available to be accepted, one of two behaviours will occur:
    /// - If `block` is `false`, returns [`crate::error::code::EAGAIN`];
    /// - If `block` is `true`, blocks until an error occurs or some connection can be accepted.
    pub fn accept(&self, block: bool) -> Result<TcpStream> {
        let mut new = core::ptr::null_mut();
        let flags = if block { 0 } else { bindings::O_NONBLOCK };
        // SAFETY: The type invariant guarantees that the socket is valid, and the output argument
        // is also valid for write.
        to_result(unsafe { bindings::kernel_accept(self.sock, &mut new, flags as _) })?;
        Ok(TcpStream { sock: new })
    }
}

impl Drop for TcpListener {
    fn drop(&mut self) {
        // SAFETY: The type invariant guarantees that the socket is valid.
        unsafe { bindings::sock_release(self.sock) };
    }
}

/// A connected TCP socket.
///
/// # Invariants
///
/// The socket pointer is always non-null and valid.
pub struct TcpStream {
    pub(crate) sock: *mut bindings::socket,
}

// SAFETY: `TcpStream` is just a wrapper for a kernel socket, which can be used from any thread.
unsafe impl Send for TcpStream {}

// SAFETY: `TcpStream` is just a wrapper for a kernel socket, which can be used from any thread.
unsafe impl Sync for TcpStream {}

impl TcpStream {
    /// Reads data from a connected socket.
    ///
    /// On success, returns the number of bytes read, which will be zero if the connection is
    /// closed.
    ///
    /// If no data is immediately available for reading, one of two behaviours will occur:
    /// - If `block` is `false`, returns [`crate::error::code::EAGAIN`];
    /// - If `block` is `true`, blocks until an error occurs, the connection is closed, or some
    ///   becomes readable.
    pub fn read(&self, buf: &mut [u8], block: bool) -> Result<usize> {
        let mut msg = bindings::msghdr::default();
        let mut vec = bindings::kvec {
            iov_base: buf.as_mut_ptr().cast(),
            iov_len: buf.len(),
        };
        // SAFETY: The type invariant guarantees that the socket is valid, and `vec` was
        // initialised with the output buffer.
        let r = unsafe {
            bindings::kernel_recvmsg(
                self.sock,
                &mut msg,
                &mut vec,
                1,
                vec.iov_len,
                if block { 0 } else { bindings::MSG_DONTWAIT } as _,
            )
        };
        if r < 0 {
            Err(Error::from_kernel_errno(r))
        } else {
            Ok(r as _)
        }
    }

    /// Writes data to the connected socket.
    ///
    /// On success, returns the number of bytes written.
    ///
    /// If the send buffer of the socket is full, one of two behaviours will occur:
    /// - If `block` is `false`, returns [`crate::error::code::EAGAIN`];
    /// - If `block` is `true`, blocks until an error occurs or some data is written.
    pub fn write(&self, buf: &[u8], block: bool) -> Result<usize> {
        let mut msg = bindings::msghdr {
            msg_flags: if block { 0 } else { bindings::MSG_DONTWAIT },
            ..bindings::msghdr::default()
        };
        let mut vec = bindings::kvec {
            iov_base: buf.as_ptr() as *mut u8 as _,
            iov_len: buf.len(),
        };
        // SAFETY: The type invariant guarantees that the socket is valid, and `vec` was
        // initialised with the input  buffer.
        let r = unsafe { bindings::kernel_sendmsg(self.sock, &mut msg, &mut vec, 1, vec.iov_len) };
        if r < 0 {
            Err(Error::from_kernel_errno(r))
        } else {
            Ok(r as _)
        }
    }
}

impl Drop for TcpStream {
    fn drop(&mut self) {
        // SAFETY: The type invariant guarantees that the socket is valid.
        unsafe { bindings::sock_release(self.sock) };
    }
}
