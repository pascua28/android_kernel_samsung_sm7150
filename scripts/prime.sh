#! /vendor/bin/sh

# Run under a new tmpfs to avoid /dev selabel
mkdir /dev/ep
mount -t tmpfs nodev /dev/ep

cat /vendor/bin/init.qcom.post_boot.sh > /dev/ep/init.qcom.post_boot.sh

echo '
/dev/resetprop persist.sys.fuse.passthrough.enable true

# Re-enable SELinux
echo "97" > /sys/fs/selinux/enforce
' >> /dev/ep/init.qcom.post_boot.sh

mount --bind /dev/ep/init.qcom.post_boot.sh /vendor/bin/init.qcom.post_boot.sh
chmod 755 /vendor/bin/init.qcom.post_boot.sh
chcon "u:object_r:vendor_qti_init_shell_exec:s0" /vendor/bin/init.qcom.post_boot.sh

# Add UClamp-aware task-profiles
if [ ! -e "/vendor/etc/task_profiles.json" ]; then
    mkdir /dev/ep/etc

    cp /dev/task_profiles.json /dev/ep/task_profiles.json

    mount --bind /dev/ep/task_profiles.json /system/etc/task_profiles.json
    chcon "u:object_r:task_profiles_file:s0" /system/etc/task_profiles.json

    cp -a --preserve=all /vendor/etc/* /dev/ep/etc/
    cp /dev/task_profiles.json /dev/ep/etc/
    chcon "u:object_r:vendor_configs_file:s0" /dev/ep/etc/task_profiles.json
    mount --bind /dev/ep/etc /vendor/etc
fi
