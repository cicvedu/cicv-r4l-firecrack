#!/bin/sh
busybox_folder="../busybox-1.36.1"
kernel_image="../linux/arch/x86/boot/bzImage"
work_dir=$PWD
rootfs="rootfs"
rootfs_img=$PWD"/rootfs_img"

make LLVM=1
echo $base_path
if [ ! -d $rootfs ]; then
    mkdir $rootfs
fi
cp $busybox_folder/_install/*  $rootfs/ -rf
cp $work_dir/r4l_e1000_demo.ko $work_dir/$rootfs/
cd $rootfs
if [ ! -d proc ] && [ ! -d sys ] && [ ! -d dev ] && [ ! -d etc/init.d ]; then
    mkdir proc sys dev etc etc/init.d
fi
 
if [ -f etc/init.d/rcS ]; then
    rm etc/init.d/rcS
fi
echo "#!/bin/sh" > etc/init.d/rcS
echo "mount -t proc none /proc" >> etc/init.d/rcS
echo "mount -t sysfs none /sys" >> etc/init.d/rcS
echo "/sbin/mdev -s" >> etc/init.d/rcS
echo "mknod /dev/cicv c 248 0" >> etc/init.d/rcS
chmod +x etc/init.d/rcS
if [ -f $rootfs_img ]; then
    rm $rootfs_img
fi

cd $work_dir

cd $rootfs 
find . | cpio -o --format=newc > $rootfs_img

cd $work_dir

qemu-system-x86_64 \
-netdev "user,id=eth0" \
-device "e1000,netdev=eth0" \
-object "filter-dump,id=eth0,netdev=eth0,file=dump.dat" \
-kernel $kernel_image \
-append "root=/dev/ram rdinit=sbin/init ip=10.0.2.15::10.0.2.1:255.255.255.0 console=ttyS0 no_timer_check" \
-nographic \
-initrd $rootfs_img

