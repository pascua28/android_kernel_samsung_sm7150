#!/bin/sh

build_kernel() {
    echo "-----------------------------------------------"
    echo "Beginning kernel compilation..."
    echo "-----------------------------------------------"

    export ARCH=arm64
    mkdir out

    export PATH=$(pwd)/llvm-22/bin:$PATH

    BUILD_VAR="-j$(nproc) -C $(pwd) O=$(pwd)/out ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LLVM=1 LLVM_IAS=1"

    cat arch/arm64/configs/sdmmagpie_defconfig arch/arm64/configs/a71.config > arch/arm64/configs/temp_defconfig

    echo "
    CONFIG_THINLTO=y
    # CONFIG_LTO_NONE is not set
    CONFIG_LTO_CLANG=y
    " >> arch/arm64/configs/temp_defconfig

    make $BUILD_VAR temp_defconfig
    rm arch/arm64/configs/temp_defconfig
}

build_dtb() {
    echo "-----------------------------------------------"
    echo "Building dtb..."
    echo "-----------------------------------------------"
    make $BUILD_VAR
    make $BUILD_VAR dtbs
}

build_dtbo() {
    echo "-----------------------------------------------"
    echo "Building dtbo.img..."
    echo "-----------------------------------------------"
    DTBO_FILES=$(find $(pwd)/out/arch/arm64/boot/dts/samsung/ -name sm*150-sec-a71-eur-overlay-*.dtbo)
    $(pwd)/tools/mkdtimg create $(pwd)/out/dtbo.img --page_size=4096 ${DTBO_FILES}

    mv $(pwd)/out/dtbo.img dtbo.img
}

build_boot() {
    echo "-----------------------------------------------"
    echo "Building boot.img..."
    echo "-----------------------------------------------"
    MKBOOTIMG="$(pwd)/mkbootimg/mkbootimg.py"
    OUT_KERNEL="$(pwd)/out/arch/arm64/boot/Image"
    DTB_OUT="$(pwd)/out/arch/arm64/boot/dts/qcom/sdmmagpie.dtb"
    CMDLINE="console=null androidboot.hardware=qcom androidboot.memcg=1 lpm_levels.sleep_disabled=1 video=vfb:640x400,bpp=32,memsize=3072000 msm_rtb.filter=0x237 service_locator.enable=1 swiotlb=1 androidboot.usbcontroller=a600000.dwc3 firmware_class.path=/vendor/firmware_mnt/image nokaslr printk.devkmsg=on loop.max_part=7"
    BASE="0x00000000"
    KOFFSET="0x00008000"
    ROFFSET="0x02000000"
    SECOFFSET="0x00000000"
    DTBOFFSET="0x01f00000"
    TAGSOFFSET="0x01e00000"
    BOARD="SRPSF18B011"
    PAGESZ="4096"
    RAMDISK="$(pwd)/boot/ramdisk"
    MONTH="$(date +%Y-%m)"

    $MKBOOTIMG \
        --header_version 2 \
        --kernel "$OUT_KERNEL" \
        --ramdisk "$RAMDISK" \
        --dtb "$DTB_OUT" \
        --cmdline "$CMDLINE" \
        --base "$BASE" \
        --kernel_offset "$KOFFSET" \
        --ramdisk_offset "$ROFFSET" \
        --second_offset "$SECOFFSET" \
        --dtb_offset "$DTBOFFSET" \
        --tags_offset "$TAGSOFFSET" \
        --board "$BOARD" \
        --pagesize "$PAGESZ" \
        --os_version 16.0.0 \
        --os_patch_level "$MONTH" \
        --output boot.img
}

build_kernel
build_dtb
build_dtbo
build_boot
