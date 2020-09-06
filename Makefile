#
# Variables
#

# Tools
CXX = $(XILINX_VITIS)/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++
VXX = $(XILINX_VITIS)/bin/v++
HOST_CXX = g++
MKDIR_P = mkdir -p
SHELL = /bin/bash
VIVADO_HLS = vivado_hls

# Global
BUILD_DIR = ${CURDIR}/build
APP_NAME ?= FastSense.exe
PLATFORM_DIR ?= ${CURDIR}/base_design/platform/FastSense_platform/export/FastSense_platform

# Software
SYSROOT = ${PLATFORM_DIR}/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux

# Main entry point
ENTRY_POINT ?= src/vadd.cpp

SRCS = ${ENTRY_POINT} \
	src/driver/lidar/velodyne.cpp \
	src/data/sensor_sync.cpp \
	$(wildcard src/util/*.cpp) \
	$(wildcard src/msg/*.cpp) \
	$(wildcard src/driver/imu/api/*.cpp) \
	src/driver/imu/imu.cpp

OBJS = $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)

INC_DIRS = src

INC_FLAGS = $(addprefix -I,$(INC_DIRS))
CXX_STD = c++17
CXXFLAGS = $(INC_FLAGS) -MMD -MP -D__USE_XOPEN2K8 -I${SYSROOT}/usr/include/xrt -I${XILINX_VIVADO}/include -I${SYSROOT}/usr/include -c -fmessage-length=0 -std=${CXX_STD} --sysroot=${SYSROOT}

LDFLAGS = -lxilinxopencl -lphidget21 -lzmq -lhdf5 -lpthread -lrt -lstdc++ -lgmp -lxrt_core -L${SYSROOT}/usr/lib/ --sysroot=${SYSROOT}

# Hardware
LINK_CFG = ${CURDIR}/link.cfg
BUILD_CFG = ${CURDIR}/build.cfg

HW_TARGET ?= hw
HW_PLATFORM = ${PLATFORM_DIR}/FastSense_platform.xpfm

HW_SRCS = src/krnl_vadd.cpp
HW_OBJS = $(HW_SRCS:%.cpp=$(BUILD_DIR)/%.xo)
HW_DEPS = $(HW_OBJS:.xo=.d)

VXXFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) -c $(INC_FLAGS) --config $(BUILD_CFG)
VXXLDFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(LINK_CFG) --link

HW_DEPS_FLAGS = $(INC_FLAGS) -isystem ${XILINX_VIVADO}/include -MM -MP

#
# Rules
#

.PHONY: all software hardware clean hls_% test

all: software hardware 

clean: 
	@rm -rf _x _vimage *.log build/*

software: $(BUILD_DIR)/$(APP_NAME)

hardware: $(BUILD_DIR)/$(APP_NAME).xclbin

# Link software
$(BUILD_DIR)/$(APP_NAME): $(OBJS)
	@echo "Link: $(APP_NAME)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# Compile software
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compile: $<"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $< -o $@

# Link hardware
$(BUILD_DIR)/$(APP_NAME).xclbin: $(HW_OBJS) $(LINK_CFG)
	@echo "Link hardware: $(APP_NAME).xclbin"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(HW_OBJS) -o $@ $(VXXLDFLAGS)

# Compile kernels
$(BUILD_DIR)/%.xo: %.cpp $(BUILD_CFG)
	@echo "Compile kernel: $<"
	@$(MKDIR_P) $(dir $@)
	@$(HOST_CXX) $< $(HW_DEPS_FLAGS) -MF $(@:.xo=.d) -MT $@
	@$(VXX) $(VXXFLAGS) $< -o $@ -k $(notdir $*)

# Open HLS GUI for kernel
hls_%: $(filter %$*.xo,$(HW_OBJS))
	@echo "Opening HLS for kernel $* ($<) "
	@$(VIVADO_HLS) -p _x/$*/$*/$*/

test_sensor_sync:
	make ENTRY_POINT=test/sensor_sync.cpp APP_NAME=FastSense_test_sensor_sync.exe software hardware

test_zmq_client: 
	make ENTRY_POINT=test/zmq_client.cpp APP_NAME=FastSense_test_zmq_client.exe software hardware

test_hdf5: 
	make ENTRY_POINT=test/hdf5.cpp APP_NAME=FastSense_test_hdf5.exe software hardware

format:
	@echo "Formatting"
	@astyle -q -n --project=.astylerc --recursive "src/*.c??" "src/*.h"

-include $(DEPS)
-include $(HW_DEPS)
