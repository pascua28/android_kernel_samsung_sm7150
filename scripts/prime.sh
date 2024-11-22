#! /vendor/bin/sh

# Run under a new tmpfs to avoid /dev selabel
mkdir /dev/ep
mount -t tmpfs nodev /dev/ep

cat /system/build.prop > /dev/ep/build.prop
echo "
persist.sys.fuse.passthrough.enable=true
" >> /dev/ep/build.prop
chmod 600 /dev/ep/build.prop
mount --bind /dev/ep/build.prop /system/build.prop
chcon "u:object_r:system_file:s0" /system/build.prop

# Commands after this line will be executed on post-fs-data
while [ "$(getprop vold.post_fs_data_done)" != 1 ]; do sleep 0.25s; done

# Re-enable SELinux
echo "97" > /sys/fs/selinux/enforce
