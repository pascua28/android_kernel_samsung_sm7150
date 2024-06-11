#! /vendor/bin/sh

# Re-enable SELinux
echo "97" > /sys/fs/selinux/enforce
