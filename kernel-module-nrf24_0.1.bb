SUMMARY = "Nordic nRF24L01+ module for kernels >= 3.10"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=4641e94ec96f98fabc56ff9cc48be14b"

inherit module

PR = "r0"
PV = "${SRCPV}"

SRC_URI = "git://danielhilst@bitbucket.org/danielhilst/nrf24.git;protocol=https"
S = "${WORKDIR}/git"
SRCREV = "${AUTOREV}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
