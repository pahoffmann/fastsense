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

# SSH
BOARD_ADDRESS ?= 192.168.1.214
USER ?= gjulian
FPGA_SERVER ?= fgpa1
FGPA_SERVER_HOME ?= /home/student/g

# Software
SYSROOT = ${PLATFORM_DIR}/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux

# Main entry point
ENTRY_POINT ?= src/example/example.cpp

SRCS = ${ENTRY_POINT} \
	src/driver/lidar/velodyne.cpp \
	src/data/sensor_sync.cpp \
	$(wildcard src/map/*.cpp) \
	$(wildcard src/util/*.cpp) \
	$(wildcard src/msg/*.cpp) \
	$(wildcard src/hw/*.cpp) \
	$(wildcard src/util/logging/*.cpp) \
	$(wildcard src/driver/imu/api/*.cpp) \
	src/driver/imu/imu.cpp

LIBS =  -lxilinxopencl \
		-lphidget21 \
		-lzmq \
		-lhdf5 \
		-lpthread \
		-lrt \
		-lstdc++ \
		-lgmp \
		-lxrt_core \
		-L${SYSROOT}/usr/lib/
	
OBJS = $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)

INC_DIRS = src ext/Catch2/single_include ${SYSROOT}/usr/include/xrt ${XILINX_VIVADO}/include ${SYSROOT}/usr/include

INC_FLAGS = $(addprefix -I,$(INC_DIRS))
CXX_STD = c++17
GCCFLAGS = -Wall -Wextra -Wnon-virtual-dtor -ansi -pedantic -Weffc++ -Wfatal-errors -O2 -ftree-loop-vectorize -fexceptions
CXXFLAGS = $(INC_FLAGS) $(GCCFLAGS) -MMD -MP -D__USE_XOPEN2K8 -c -fmessage-length=0 -std=${CXX_STD} --sysroot=${SYSROOT}

LDFLAGS = $(LIBS) --sysroot=${SYSROOT}

# Hardware
LINK_CFG = ${CURDIR}/link.cfg
BUILD_CFG = ${CURDIR}/build.cfg

HW_TARGET ?= hw
HW_PLATFORM = ${PLATFORM_DIR}/FastSense_platform.xpfm

HW_SRCS = src/example/krnl_vadd.cpp
HW_OBJS = $(HW_SRCS:%.cpp=$(BUILD_DIR)/%.xo)
HW_DEPS = $(HW_OBJS:.xo=.d)

VXXFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) -c $(INC_FLAGS) --config $(BUILD_CFG)
VXXLDFLAGS = -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(LINK_CFG) --link

HW_DEPS_FLAGS = $(INC_FLAGS) -isystem ${XILINX_VIVADO}/include -MM -MP

#
# Rules
#

.PHONY: all software hardware clean hls_% test clean_software clean_ros_nodes

all: software hardware  

test: test_sensor_sync test_zmq test_global_map

clean: 
	@rm -rf _x _vimage *.log build/* test/*.log

# TODO why does build/src/*/*.{o,d} not work
clean_software:
	@rm -rf build/src/driver/* build/src/util/* build/src/hw/* build/src/example/* build/src/map/* build/src/hw/* test/*.log

clean_ros_nodes:
	@rm -rf test/build/* test/devel/* test/*.log

clean_tests:
	@rm -rf test/build-local/*

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
	make ENTRY_POINT=test/sensor_sync.cpp APP_NAME=FastSense_test_sensor_sync.exe software

test_zmq: test_zmq_client test_zmq_server test_sender test_receiver

test_zmq_client: 
	make ENTRY_POINT=test/zmq_client.cpp APP_NAME=FastSense_test_zmq_client.exe software

test_zmq_server: 
	make ENTRY_POINT=test/zmq_server.cpp APP_NAME=FastSense_test_zmq_server.exe software

test_sender: 
	make ENTRY_POINT=test/sender.cpp APP_NAME=FastSense_test_sender.exe software

test_receiver: 
	make ENTRY_POINT=test/receiver.cpp APP_NAME=FastSense_test_receiver.exe software

ros_test_nodes:
	@cd test && . /opt/ros/melodic/setup.bash && catkin_make -j4

test_global_map:
	make ENTRY_POINT=test/global_map.cpp APP_NAME=FastSense_test_global_map.exe software

run_global_map_test: test_global_map
	@cd build/ && ./FastSense_test_global_map.exe

copy_binaries_to_board:
	@scp build/*.exe* student@$(BOARD_ADDRESS):/mnt

rsync:
	@echo 'syning fastsense: to "$(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense"'
	@rsync -azP ./ $(USER)@$(FPGA_SERVER).informatik.uos.de:$(FGPA_SERVER_HOME)/$(USER)/fastsense

format:
	@echo "Formatting"
	@astyle -q -n --project=.astylerc --recursive "src/*.c??" "src/*.h"

-include $(DEPS)
-include $(HW_DEPS)
