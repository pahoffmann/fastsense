all: check_files check_env compile_host link_host compile_kernel link_kernel compile_kernel_emu link_kernel_emu

CPP_STD := c++14
SYSROOT := ${CURDIR}/FastSense_platform/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux
CONNECTIVITY := ${CURDIR}/connectivity.cfg
BUILD_DIR := ${CURDIR}/build
SHELL := /bin/bash

clean: 
	@rm -rf _x _vimage *.log build/* 

check_files:
	@echo "-- files --"
	@[ -f "${CONNECTIVITY}" ] && echo "Found connectivity file: connectivity.cfg" || (echo "connectivity.cfg file missing"; exit 1)
	@[ -d "${BUILD_DIR}" ] && echo "Found build directory ${BUILD_DIR}" ||  (mkdir ${BUILD_DIR} && echo "Created build directory")
	@echo "-----------"

check_env:
	@echo "--  env  --"
	@[ "${XILINX_VITIS}" ] && echo "XILINX_VITIS: ${XILINX_VITIS}" || ( echo "XILINX_VITIS is not set"; exit 1 )
	@[ "${XILINX_VIVADO}" ] && echo "XILINX_VIVADO: ${XILINX_VIVADO}" || ( echo "XILINX_VIVADO is not set"; exit 1 )
	@[ "${SYSROOT}" ] && echo "SYSROOT: ${SYSROOT}" || ( echo "SYSROOT is not set"; exit 1 )
	@[ "${PLATFORM_REPO_PATHS}" ] && echo "PLATFORM_REPO_PATHS: ${PLATFORM_REPO_PATHS}" || ( echo "PLATFORM_REPO_PATHS is not set"; exit 1 )
	@echo "-----------"

compile_host:
	@echo "Compile host"	
	@${XILINX_VITIS}/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++ -D__USE_XOPEN2K8 -I${SYSROOT}/usr/include/xrt -I${XILINX_VIVADO}/include -I${SYSROOT}/usr/include -c -fmessage-length=0 -std=${CPP_STD} --sysroot=${SYSROOT} -o build/vadd.o src/vadd.cpp

link_host: build/vadd.o
	@echo "Link host"
	@${XILINX_VITIS}/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++ -o build/vadd.exe build/vadd.o -lxilinxopencl -lpthread -lrt -lstdc++ -lgmp -lxrt_core -L${SYSROOT}/usr/lib/ --sysroot=${SYSROOT}

compile_kernel: build/vadd.exe
	@echo "Compile kernel"
	@v++ -t sw_emu --platform FastSense_platform -c -k krnl_vadd -I src -o build/krnl_vadd.xo src/krnl_vadd.cpp --config ./build.cfg

link_kernel: build/krnl_vadd.xo
	@echo "link kernel"
	@v++ -t sw_emu --platform FastSense_platform --link build/krnl_vadd.xo -o build/krnl_vadd.xclbin --config ./connectivity.cfg

compile_kernel_emu: build/vadd.exe
	@echo "Compile kernel simulation"
	@v++ -t hw --platform FastSense_platform -c -k krnl_vadd -I src -o build/krnl_vadd.xo src/krnl_vadd.cpp

link_kernel_emu: build/krnl_vadd.xo
	@echo "link kernel simulation"
	@v++ -t hw --platform FastSense_platform --link build/krnl_vadd.xo -o build/krnl_vadd.xclbin --config ./connectivity.cfg




