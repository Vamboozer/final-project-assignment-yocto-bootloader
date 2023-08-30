SUMMARY = "RFuC Bootloader Service"
DESCRIPTION = "Service to run the RFuC Bootloader at boot"

# See https://git.yoctoproject.org/poky/tree/meta/files/common-licenses
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "python3 python3-spidev"
RDEPENDS_${PN} = "python3-spidev"

SRC_URI = "file://RFuCBootloader.py \
           file://rfucboot.init"

S = "${WORKDIR}"
MY_LAYER_DIR = "${@os.path.dirname(d.getVar('FILE_DIRNAME', True))}"

do_fetch_append() {
    import shutil
    import glob
    import os

    layer_dir = os.path.dirname(d.getVar('FILE_DIRNAME', True))
    work_dir = d.getVar('WORKDIR')

    bb.debug(1, f"MY_LAYER_DIR is {layer_dir}")
    bb.debug(1, f"WORKDIR is {work_dir}")

    work_dir_image_archive = os.path.join(work_dir, 'ImageArchive')
    os.makedirs(work_dir_image_archive, exist_ok=True)

    # Additional debugging information
    bb.debug(1, f"Checking files in {layer_dir}/rfucboot/files/ImageArchive")
    debug_files_image_archive = glob.glob(f"{layer_dir}/rfucboot/files/ImageArchive/RFuC_*.bin")
    bb.debug(1, f"Files found: {debug_files_image_archive}")

    source_files = glob.glob(f"{layer_dir}/rfucboot/files/RFuC_*.bin")
    if len(source_files) != 1:
        bb.fatal(f"Exactly one RFuC_*.bin file should be present in the files directory, found {len(source_files)}.")
    
    source_files_image_archive = glob.glob(f"{layer_dir}/rfucboot/files/ImageArchive/RFuC_*.bin")
    if len(source_files_image_archive) != 1:
        bb.fatal(f"Exactly one RFuC_*.bin file should be present in the files/ImageArchive directory, found {len(source_files_image_archive)}.")

    for source_file in source_files:
        shutil.copy(source_file, work_dir)

    for source_file in source_files_image_archive:
        shutil.copy(source_file, work_dir_image_archive)
}

do_unpack_append() {
    import glob
    import os

    work_dir = d.getVar('WORKDIR')
    work_dir_image_archive = os.path.join(work_dir, 'ImageArchive')

    bb.debug(1, f"Checking files in {work_dir} and {work_dir_image_archive}")

    # Check for files in the 'files' subdirectory within WORKDIR
    source_files = glob.glob(f"{work_dir}/RFuC_*.bin")
    if len(source_files) != 1:
        bb.fatal(f"Exactly one RFuC_*.bin file should be present in the {work_dir} directory, found {len(source_files)}.")

    # Check for files in the 'ImageArchive' subdirectory within WORKDIR
    source_files_image_archive = glob.glob(f"{work_dir_image_archive}/RFuC_*.bin")
    if len(source_files_image_archive) != 1:
        bb.fatal(f"Exactly one RFuC_*.bin file should be present in the {work_dir_image_archive} directory, found {len(source_files_image_archive)}.")
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${S}/RFuCBootloader.py ${D}${bindir}/RFuCBootloader.py
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${S}/rfucboot.init ${D}${sysconfdir}/init.d/rfucboot

    install -d ${D}${bindir}/ImageArchive
    #install -m 0644 ${S}/RFuC_1ef76eed08e80cfe985bd5788fc699b4b2b5a4da_Dirty_Aug-24-2023-09-57-04.bin ${D}${bindir}/
    #install -m 0644 ${S}/ImageArchive/RFuC_1ef76eed08e80cfe985bd5788fc699b4b2b5a4da_Clean_Aug-24-2023-09-53-09.bin ${D}${bindir}/ImageArchive/

    # Install the single RFuC_*.bin file from the main files directory
    main_img=$(find ${S} -maxdepth 1 -name 'RFuC_*.bin')
    if [ -n "$main_img" ]; then
        install -m 0644 $main_img ${D}${bindir}/
    else
        bbfatal "No RFuC_*.bin file found in the files directory, which should not happen."
    fi

    # Install the single RFuC_*.bin file from the ImageArchive directory
    archive_img=$(find ${S}/ImageArchive -name 'RFuC_*.bin')
    if [ -n "$archive_img" ]; then
        install -m 0644 $archive_img ${D}${bindir}/ImageArchive/
    else
        bbfatal "No RFuC_*.bin file found in the ImageArchive directory, which should not happen."
    fi
}

INITSCRIPT_NAME = "rfucboot"
INITSCRIPT_PARAMS = "defaults 99"

inherit update-rc.d
