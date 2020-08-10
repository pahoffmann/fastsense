#
# Variables
#

# Tools
CXX := aarch64-linux-gnu-g++
VXX := v++
MKDIR_P := mkdir -p
SHELL := /bin/bash
VIVADO_HLS := vivado_hls

# Global
BUILD_DIR := ${CURDIR}/build
APP_NAME := FastSense
PLATFORM_DIR ?= ${CURDIR}/base_design/platform/FastSense_platform/export/FastSense_platform

# Software
SYSROOT := ${PLATFORM_DIR}/sw/FastSense_platform/linux_domain/sysroot/aarch64-xilinx-linux/

SRCS := src/vadd.cpp
OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := src
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
CXX_STD := c++14
CXXFLAGS := $(INC_FLAGS) -MMD -MP -D__USE_XOPEN2K8 -I${SYSROOT}/usr/include/xrt -I${XILINX_VIVADO}/include -I${SYSROOT}/usr/include -c -fmessage-length=0 -std=${CXX_STD} --sysroot=${SYSROOT}

LDFLAGS := -lxilinxopencl -lpthread -lrt -lstdc++ -lgmp -lxrt_core -L${SYSROOT}/usr/lib/ --sysroot=${SYSROOT}

# Hardware
LINK_CFG := ${CURDIR}/link.cfg
BUILD_CFG := ${CURDIR}/build.cfg

HW_TARGET ?= hw
HW_PLATFORM := ${PLATFORM_DIR}/FastSense_platform.xpfm

HW_SRCS := src/krnl_vadd.cpp
HW_OBJS := $(HW_SRCS:%.cpp=$(BUILD_DIR)/%.xo)

VXXFLAGS := -t $(HW_TARGET) -f $(HW_PLATFORM) -c $(INC_FLAGS) --config $(BUILD_CFG)
VXXLDFLAGS := -t $(HW_TARGET) -f $(HW_PLATFORM) --config $(LINK_CFG) --link

#
# Rules
#

all: $(BUILD_DIR)/$(APP_NAME) $(BUILD_DIR)/$(APP_NAME).xclbin

clean: 
	@rm -rf _x _vimage *.log build/*

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
	@echo "Link hardawre: $(APP_NAME).xclbin"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(HW_OBJS) -o $@ $(VXXLDFLAGS)

# Compile kernels
$(BUILD_DIR)/%.xo: %.cpp $(BUILD_CFG)
	@echo "Compile kernel: $<"
	@$(MKDIR_P) $(dir $@)
	@$(VXX) $(VXXFLAGS) $< -o $@ -k $(notdir $*)

# Open HLS GUI for kernel
hls_%: $(filter %$*.xo,$(HW_OBJS))
	@echo "Opening HLS for kernel $* ($<) "
	@$(VIVADO_HLS) -p _x/$*/$*/$*/

.PHONY: all clean hls_%
