// SPDX-License-Identifier: GPL-2.0

//! PCI devices and drivers.
//!
//! C header: [`include/linux/pci.h`](../../../../include/linux/pci.h)

use crate::{
    bindings, device, driver,
    error::{
        code::{EINVAL, ENOMEM},
        from_kernel_result, Error, Result,
    },
    str::CStr,
    to_result,
    types::PointerWrapper,
    ThisModule,
};

/// An adapter for the registration of PCI drivers.
pub struct Adapter<T: Driver>(T);

impl<T: Driver> driver::DriverOps for Adapter<T> {
    type RegType = bindings::pci_driver;

    unsafe fn register(
        reg: *mut bindings::pci_driver,
        name: &'static CStr,
        module: &'static ThisModule,
    ) -> Result {
        // SAFETY: By the safety requirements of this function (defined in the trait definition),
        // `reg` is non-null and valid.
        let pdrv: &mut bindings::pci_driver = unsafe { &mut *reg };

        pdrv.name = name.as_char_ptr();
        pdrv.probe = Some(Self::probe_callback);
        pdrv.remove = Some(Self::remove_callback);
        pdrv.id_table = T::ID_TABLE.as_ref();
        // SAFETY:
        //   - `pdrv` lives at least until the call to `pci_unregister_driver()` returns.
        //   - `name` pointer has static lifetime.
        //   - `probe()` and `remove()` are static functions.
        //   - `of_match_table` is a raw pointer with static lifetime,
        //      as guaranteed by the [`driver::IdTable`] type.
        to_result(unsafe { bindings::__pci_register_driver(reg, module.0, name.as_char_ptr()) })
    }

    unsafe fn unregister(reg: *mut bindings::pci_driver) {
        // SAFETY: By the safety requirements of this function (defined in the trait definition),
        // `reg` was passed (and updated) by a previous successful call to
        // `__pci_register_driver`.
        unsafe { bindings::pci_unregister_driver(reg) }
    }
}

impl<T: Driver> Adapter<T> {
    extern "C" fn probe_callback(
        pdev: *mut bindings::pci_dev,
        id: *const bindings::pci_device_id,
    ) -> core::ffi::c_int {
        from_kernel_result! {
            // SAFETY: `pdev` is valid by the contract with the C code. `dev` is alive only for the
            // duration of this call, so it is guaranteed to remain alive for the lifetime of
            // `pdev`.
            let mut dev = unsafe { Device::from_ptr(pdev) };

            // SAFETY: `id` is a pointer within the static table, so it's always valid.
            let offset = unsafe {(*id).driver_data};
            // SAFETY: The offset comes from a previous call to `offset_from` in `IdArray::new`, which
            // guarantees that the resulting pointer is within the table.
            let info = {
                let ptr = unsafe {id.cast::<u8>().offset(offset as _).cast::<Option<T::IdInfo>>()};
                unsafe {(&*ptr).as_ref()}
            };
            let data = T::probe(&mut dev, info)?;
             // SAFETY: `pdev` is guaranteed to be a valid, non-null pointer.
            unsafe { bindings::pci_set_drvdata(pdev, data.into_pointer() as _) };
            Ok(0)
        }
    }

    extern "C" fn remove_callback(pdev: *mut bindings::pci_dev) {
        // SAFETY: `pdev` is guaranteed to be a valid, non-null pointer.
        let ptr = unsafe { bindings::pci_get_drvdata(pdev) };
        // SAFETY:
        //   - we allocated this pointer using `T::Data::into_pointer`,
        //     so it is safe to turn back into a `T::Data`.
        //   - the allocation happened in `probe`, no-one freed the memory,
        //     `remove` is the canonical kernel location to free driver data. so OK
        //     to convert the pointer back to a Rust structure here.
        let data = unsafe { T::Data::from_pointer(ptr) };
        T::remove(&data);
        <T::Data as driver::DeviceRemoval>::device_remove(&data);
    }
}

/// Abstraction for bindings::pci_device_id.
#[derive(Clone, Copy)]
pub struct DeviceId {
    /// Vendor ID
    pub vendor: u32,
    /// Device ID
    pub device: u32,
    /// Subsystem vendor ID
    pub subvendor: u32,
    /// Subsystem device ID
    pub subdevice: u32,
    /// Device class and subclass
    pub class: u32,
    /// Limit which sub-fields of the class
    pub class_mask: u32,
}

impl DeviceId {
    const PCI_ANY_ID: u32 = !0;

    /// PCI_DEVICE macro.
    pub const fn new(vendor: u32, device: u32) -> Self {
        Self {
            vendor,
            device,
            subvendor: DeviceId::PCI_ANY_ID,
            subdevice: DeviceId::PCI_ANY_ID,
            class: 0,
            class_mask: 0,
        }
    }

    /// PCI_DEVICE_CLASS macro.
    pub const fn with_class(class: u32, class_mask: u32) -> Self {
        Self {
            vendor: DeviceId::PCI_ANY_ID,
            device: DeviceId::PCI_ANY_ID,
            subvendor: DeviceId::PCI_ANY_ID,
            subdevice: DeviceId::PCI_ANY_ID,
            class,
            class_mask,
        }
    }
}

// SAFETY: `ZERO` is all zeroed-out and `to_rawid` stores `offset` in `pci_device_id::driver_data`.
unsafe impl const driver::RawDeviceId for DeviceId {
    type RawType = bindings::pci_device_id;

    const ZERO: Self::RawType = bindings::pci_device_id {
        vendor: 0,
        device: 0,
        subvendor: 0,
        subdevice: 0,
        class: 0,
        class_mask: 0,
        driver_data: 0,
        override_only: 0,
    };

    fn to_rawid(&self, offset: isize) -> Self::RawType {
        bindings::pci_device_id {
            vendor: self.vendor,
            device: self.device,
            subvendor: self.subvendor,
            subdevice: self.subdevice,
            class: self.class,
            class_mask: self.class_mask,
            driver_data: offset as _,
            override_only: 0,
        }
    }
}

/// Define a const pci device id table
///
/// # Examples
///
/// ```ignore
/// # use kernel::{pci, define_pci_id_table};
/// #
/// struct MyDriver;
/// impl pci::Driver for MyDriver {
///     // [...]
/// #   fn probe(_dev: &mut pci::Device, _id_info: Option<&Self::IdInfo>) -> Result {
/// #       Ok(())
/// #   }
/// #   define_pci_id_table! {u32, [
/// #       (pci::DeviceId::new(0x010800, 0xffffff), None),
/// #       (pci::DeviceId::with_class(0x010802, 0xfffff), Some(0x10)),
/// #   ]}
/// }
/// ```
#[macro_export]
macro_rules! define_pci_id_table {
    ($data_type:ty, $($t:tt)*) => {
        type IdInfo = $data_type;
        const ID_TABLE: $crate::driver::IdTable<'static, $crate::pci::DeviceId, $data_type> = {
            $crate::define_id_array!(ARRAY, $crate::pci::DeviceId, $data_type, $($t)* );
            ARRAY.as_table()
        };
    };
}

/// A PCI driver
pub trait Driver {
    /// Data stored on device by driver.
    ///
    /// Corresponds to the data set or retrieved via the kernel's
    /// `pci_{set,get}_drvdata()` functions.
    ///
    /// Require that `Data` implements `PointerWrapper`. We guarantee to
    /// never move the underlying wrapped data structure.
    type Data: PointerWrapper + Send + Sync + driver::DeviceRemoval = ();

    /// The type holding information about each device id supported by the driver.
    type IdInfo: 'static = ();

    /// The table of device ids supported by the driver.
    const ID_TABLE: driver::IdTable<'static, DeviceId, Self::IdInfo>;

    /// PCI driver probe.
    ///
    /// Called when a new platform device is added or discovered.
    /// Implementers should attempt to initialize the device here.
    fn probe(dev: &mut Device, id: Option<&Self::IdInfo>) -> Result<Self::Data>;

    /// PCI driver remove.
    ///
    /// Called when a platform device is removed.
    /// Implementers should prepare the device for complete removal here.
    fn remove(_data: &Self::Data);
}

/// PCI resource
pub struct Resource {
    start: bindings::resource_size_t,
    end: bindings::resource_size_t,
    flags: u64,
}

impl Resource {
    /// resource length
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        if self.end == 0 {
            0
        } else {
            (self.end - self.start + 1) as usize
        }
    }

    /// check resource flags
    pub fn check_flags(&self, bits: u32) -> bool {
        self.flags & (bits as u64) > 0
    }
}

/// A PCI device.
///
/// # Invariants
///
/// The field `ptr` is non-null and valid for the lifetime of the object.
pub struct Device {
    ptr: *mut bindings::pci_dev,
}

impl Device {
    /// Creates a new PCI device from the given pointer.
    ///
    /// # Safety
    ///
    /// `ptr` must be non-null and valid. It must remain valid for the lifetime of the returned
    /// instance.
    unsafe fn from_ptr(ptr: *mut bindings::pci_dev) -> Self {
        Self { ptr }
    }

    /// enables bus-mastering for device
    pub fn set_master(&self) {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { bindings::pci_set_master(self.ptr) };
    }

    /// get legacy irq number
    pub fn irq(&self) -> u32 {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { (*self.ptr).irq }
    }

    /// Initialize device
    pub fn enable_device(&mut self) -> Result {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        let ret = unsafe { bindings::pci_enable_device(self.ptr) };
        if ret != 0 {
            Err(Error::from_kernel_errno(ret))
        } else {
            Ok(())
        }
    }

    /// iter PCI Resouces
    pub fn iter_resource(&self) -> impl Iterator<Item = Resource> + '_ {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        let pdev = unsafe { &*self.ptr };
        pdev.resource.iter().map(|x| Resource {
            start: x.start,
            end: x.end,
            flags: x.flags,
        })
    }

    /// Return BAR mask from the type of resource
    pub fn select_bars(&self, flags: u64) -> i32 {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { bindings::pci_select_bars(self.ptr, flags) }
    }

    /// Reserve selected PCI I/O and memory resources
    pub fn request_selected_regions(&mut self, bars: i32, name: &'static CStr) -> Result {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        let ret =
            unsafe { bindings::pci_request_selected_regions(self.ptr, bars, name.as_char_ptr()) };
        if ret != 0 {
            Err(Error::from_kernel_errno(ret))
        } else {
            Ok(())
        }
    }

    /// Get address for accessing the device
    pub fn map_resource(&self, resource: &Resource, len: usize) -> Result<MappedResource> {
        MappedResource::try_new(resource.start, len)
    }
}

unsafe impl device::RawDevice for Device {
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { &mut (*self.ptr).dev }
    }
}

/// Address for accessing the device
/// io_mem.rs requires const size but some drivers have to handle
/// non const size with ioremap().
pub struct MappedResource {
    /// address
    pub ptr: usize,
    len: usize,
}

macro_rules! define_read {
    ($(#[$attr:meta])* $name:ident, $type_name:ty) => {
        /// Reads IO data from the given offset
        $(#[$attr])*
        #[inline]
        pub fn $name(&self, offset: usize) -> Result<$type_name> {
            if offset + core::mem::size_of::<$type_name>() > self.len {
                return Err(EINVAL);
            }
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // guarantees that the code won't link if `offset` makes the write go out of bounds
            // (including the type size).
            let ptr = self.ptr.wrapping_add(offset);
            Ok(unsafe { bindings::$name(ptr as _) })
        }
    };
}

macro_rules! define_write {
    ($(#[$attr:meta])* $name:ident, $type_name:ty) => {
        /// Writes IO data to the given offset
        $(#[$attr])*
        #[inline]
        pub fn $name(&self, value: $type_name, offset: usize) -> Result {
            if offset + core::mem::size_of::<$type_name>() > self.len {
                return Err(EINVAL);
            }
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // guarantees that the code won't link if `offset` makes the write go out of bounds
            // (including the type size).
            let ptr = self.ptr.wrapping_add(offset);
            unsafe { bindings::$name(value, ptr as _) };
            Ok(())
        }
   };
}

impl MappedResource {
    fn try_new(offset: bindings::resource_size_t, len: usize) -> Result<Self> {
        let addr = unsafe { bindings::ioremap(offset, len as _) };
        if addr.is_null() {
            Err(ENOMEM)
        } else {
            Ok(Self {
                ptr: addr as usize,
                len,
            })
        }
    }

    define_read!(readb, u8);
    define_read!(readb_relaxed, u8);
    define_read!(readw, u16);
    define_read!(readw_relaxed, u16);
    define_read!(readl, u32);
    define_read!(readl_relaxed, u32);
    define_read!(
        #[cfg(CONFIG_64BIT)]
        readq,
        u64
    );
    define_read!(
        #[cfg(CONFIG_64BIT)]
        readq_relaxed,
        u64
    );

    define_write!(writeb, u8);
    define_write!(writeb_relaxed, u8);
    define_write!(writew, u16);
    define_write!(writew_relaxed, u16);
    define_write!(writel, u32);
    define_write!(writel_relaxed, u32);
    define_write!(
        #[cfg(CONFIG_64BIT)]
        writeq,
        u64
    );
    define_write!(
        #[cfg(CONFIG_64BIT)]
        writeq_relaxed,
        u64
    );
}

impl Drop for MappedResource {
    fn drop(&mut self) {
        unsafe {
            // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
            bindings::iounmap(self.ptr as _);
        }
    }
}

///
pub struct IoPort {
    ptr: usize,
    len: usize,
}

impl IoPort {
    ///
    pub fn try_new(re: &Resource) -> Result<Self> {
        Ok(Self {
            ptr: re.start as usize,
            len: re.len(),
        })
    }

    define_read!(inb, u8);
    define_read!(inw, u16);
    define_read!(inl, u32);

    define_write!(outb, u8);
    define_write!(outw, u16);
    define_write!(outl, u32);
}
