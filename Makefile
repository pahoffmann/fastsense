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
BUILD_DIR = $(CURDIR)/build
APP_NAME ?= FastSense
PLATFORM_DIR ?= $(CURDIR)/base_design/platform/FastSense_platform/export/FastSense_platform

APP_EXE = $(BUILD_DIR)/$(APP_NAME).exe
APP_TEST_EXE = $(BUILD_DIR)/$(APP_NAME)_test.exe
APP_XCLBIN = $(BUILD_DIR)/$(APP_NAME).xclbin

# Software
SYSROOT = $(PLATFORM_DIR)/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux

# Main entry point sources
ENTRY_POINT_SRCS = src/example/example.cpp
ENTRY_POINT_OBJS = $(ENTRY_POINT_SRCS:%.cpp=$(BUILD_DIR)/%.o)
ENTRY_POINT_DEPS = $(ENTRY_POINT_OBJS:.o=.d)

# Software sources
SW_SRCS = src/driver/lidar/velodyne.cpp \
	src/data/sensor_sync.cpp \
	$(wildcard src/map/*.cpp) \
	$(wildcard src/util/*.cpp) \
	$(wildcard src/msg/*.cpp) \
	$(wildcard src/util/config/*.cpp) \
	$(wildcard src/hw/*.cpp) \
	$(wildcard src/util/logging/*.cpp) \
	$(wildcard src/driver/imu/api/*.cpp) \
	src/driver/imu/imu.cpp
SW_OBJS = $(SW_SRCS:%.cpp=$(BUILD_DIR)/%.o)
SW_DEPS = $(SW_OBJS:.o=.d)

# Test sources
TEST_SRCS = $(wildcard test/*.cpp)
TEST_OBJS = $(TEST_SRCS:%.cpp=$(BUILD_DIR)/%.o)
TEST_DEPS = $(TEST_OBJS:.o=.d)

LIBS = \
	-lxilinxopencl \
	-lphidget21 \
	-lzmq \
	-lhdf5 \
	-lpthread \
	-lrt \
	-lstdc++ \
	-lgmp \
	-lxrt_core \
	-L$(SYSROOT)/usr/lib/

INC_DIRS = \
	src \
	ext/Catch2/single_include \
	$(SYSROOT)/usr/include \
	$(SYSROOT)/usr/include/xrt \
	$(XILINX_VIVADO)/include

INC_FLAGS = $(addprefix -I,$(INC_DIRS))
CXX_STD = c++17
GCCFLAGS = -Wall -Wextra -Wnon-virtual-dtor -ansi -pedantic -Weffc++ -Wfatal-errors -O2 -ftree-loop-vectorize -fexceptions
CXXFLAGS = $(INC_FLAGS) $(GCCFLAGS) -MMD -MP -D__USE_XOPEN2K8 -c -fmessage-length=0 -std=$(CXX_STD) --sysroot=$(SYSROOT)

LDFLAGS = $(LIBS) --sysroot=$(SYSROOT)

# Hardware
LINK_CFG = $(CURDIR)/link.cfg
BUILD_CFG = $(CURDIR)/build.cfg

HW_TARGET ?= hw
HW_PLATFORM = $(PLATFORM_DIR)/FastSense_platform.xpfm

HW_SRCS = src/example/krnl_vadd.cpp
HW_OBJS = $(HW_SRCS:%.cpp=$(BUILD_DIR)/%.xo)
HW_DEPS = $(HW_OBJS:.xo=.d)

VXXFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) -c $(INC_FLAGS) --config $(BUILD_CFG)
VXXLDFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(LINK_CFG) --link

HW_DEPS_FLAGS = $(INC_FLAGS) -isystem $(XILINX_VIVADO)/include -MM -MP

#
# Rules
#

.PHONY: all software hardware clean hls_% test clean_software clean_ros_nodes

all: software hardware

clean:
	@rm -rf _x .Xil _vimage *.log pl_script.sh start_simulation.sh
	@rm -rf build/*

clean_software:
	@rm -rf $(SW_OBJS) $(ENTRY_POINT_OBJS) $(SW_DEPS) $(ENTRY_POINT_DEPS) $(APP_EXE)

clean_test:
	@rm -rf $(TEST_OBJS) $(SW_OBJS) $(TEST_DEPS) $(SW_DEPS) $(APP_TEST_EXE)

clean_hardware:
	@rm -rf $(HW_OBJS) $(HW_DEPS) $(APP_XCLBIN) _vimage

clean_ros_nodes:
	@rm -rf test/build/* test/devel/* test/*.log

software: $(APP_EXE)

test: $(APP_TEST_EXE)

hardware: $(APP_XCLBIN)

# Link software
$(APP_EXE): $(ENTRY_POINT_OBJS) $(SW_OBJS)
	@echo "Link: $(APP_EXE)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(ENTRY_POINT_OBJS) $(SW_OBJS) -o $@ $(LDFLAGS)

# Link test
$(APP_TEST_EXE): $(TEST_OBJS) $(SW_OBJS)
	@echo "Link: $(APP_TEST_EXE)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(TEST_OBJS) $(SW_OBJS) -o $@ $(LDFLAGS)

# Compile software
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compile: $<"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $< -o $@

# Link hardware
$(APP_XCLBIN): $(HW_OBJS) $(LINK_CFG)
	@echo "Link hardware: $(APP_XCLBIN)"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(HW_OBJS) -o $@ $(VXXLDFLAGS) > $<.out || cat $<.out

# Compile kernels
$(BUILD_DIR)/%.xo: %.cpp $(BUILD_CFG)
	@echo "Compile kernel: $<"
	@$(MKDIR_P) $(dir $@)
	@$(HOST_CXX) $< $(HW_DEPS_FLAGS) -MF $(@:.xo=.d) -MT $@
	@$(VXX) $(VXXFLAGS) $< -o $@ -k $(notdir $*) > $<.out || cat $<.out

# Open HLS GUI for kernel
hls_%: $(filter %$*.xo,$(HW_OBJS))
	@echo "Opening HLS for kernel $* ($<) "
	@$(VIVADO_HLS) -p _x/$*/$*/$*/

copy_binaries_to_board:
	@rsync --ignore-missing-args -r $(APP_EXE) $(APP_XCLBIN) student@$(BOARD_ADDRESS):

copy_binaries_to_qemu:
	xsct -eval "set filelist {"build/FastSense.exe" "/mnt/FastSense.exe" "build/FastSense.xclbin" "/mnt/FastSense.xclbin"}; source copy_to_qemu.tcl"

copy_test_to_qemu:
	xsct -eval 'set filelist {"build/FastSense_test.exe" "/mnt/FastSense_test.exe" "build/FastSense.xclbin" "/mnt/FastSense.xclbin" "test/config.json" "/mnt/config.json"}; source copy_to_qemu.tcl'

# Add for each port to forward "-redir tcp:localport::vmport" as --qemu-args
start_emulator:
	launch_emulator -no-reboot -runtime ocl -t sw_emu -forward-port 1440 1534 -qemu-args "-redir udp:2368::2368"

rsync:
	@echo 'syning fastsense: to "$(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense"'
	@rsync -azP ./ $(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense

format:
	@echo "Formatting"
	@astyle -q -n --project=.astylerc --recursive "src/*.c??" "src/*.h" "test/*.c??" "test/*.h"

-include $(ENTRY_POINT_DEPS)
-include $(SW_DEPS)
-include $(TEST_DEPS)
-include $(HW_DEPS)
