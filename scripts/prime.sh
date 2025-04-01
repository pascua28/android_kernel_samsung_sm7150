#! /vendor/bin/sh

# Run under a new tmpfs to avoid /dev selabel
mkdir /dev/ep
mount -t tmpfs nodev /dev/ep

/dev/resetprop persist.sys.fuse.passthrough.enable true

# Commands after this line will be executed on post-fs-data
while [ "$(getprop vold.post_fs_data_done)" != 1 ]; do sleep 0.25s; done

# Re-enable SELinux
echo "97" > /sys/fs/selinux/enforce
