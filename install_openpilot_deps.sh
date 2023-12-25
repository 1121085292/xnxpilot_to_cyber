#!/usr/bin/env bash

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# GLES3
if ldconfig -p | grep -q libgles; then
    info "gles was already installed"
    exit 0
else
    apt_get_update_and_install libgles2-mesa-dev
fi

# OpenCL
if ldconfig -p | grep -q libOpenCL; then
    info "OpenCL was already installed"
    exit 0
else
    apt_get_update_and_install \
    clinfo \
    opencl-headers \
    opencl-dev \
    libpocl2
fi

# zmq
if ldconfig -p | grep -q libzmq; then
    info "zmq was already installed"
    exit 0
else
    apt_get_update_and_install libzmq3-dev
    # VERSION="4.3.5"
    # PKG_OCV="zeromq-${VERSION}.tar.gz"
    # DOWNLOAD_LINK="https://github.com/zeromq/libzmq/releases/download/v${VERSION}/zeromq-${VERSION}.tar.gz"
    # tar xzf ${PKG_OCV}
    # pushd "zeromq-${VERSION}"
    #         ./autogen.sh
    #         ./configure && make check
    #         make install
    #         ldconfig
    # popd

    # rm -rf zeromq*
fi

# libyuv
if ldconfig -p | grep -q libyuv; then
    info "yuv was already installed"
    exit 0
else
    git clone https://chromium.googlesource.com/libyuv/libyuv
    pushd "libyuv"
        mkdir build && cd build
        cmake ..
        make
        make install
    popd

    ldconfig
    # clean up
    rm -rf "libyuv"
fi

# jpeglib
if ldconfig -p | grep -q libjpeg; then
    info "libjpeg was already installed"
    exit 0
else
    apt_get_update_and_install libjpeg-dev
fi
