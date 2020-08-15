#
# Variables
#

# Tools
CXX := g++
MKDIR_P := mkdir -p
SHELL := /bin/bash
VIVADO_HLS := vivado_hls

# Global
BUILD_DIR := ./build
APP_NAME := imu
SRCS := $(wildcard api/*.cpp) $(wildcard ./*.cpp)
OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := src
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
CXX_STD := c++14
CXXFLAGS := $(INC_FLAGS) -c -std=${CXX_STD}

LDFLAGS := -lpthread -lphidget21

#
# Rules
#

.PHONY: all software clean

all: software

clean:
	@rm -rf _x _vimage *.log build/*

software: $(BUILD_DIR)/$(APP_NAME)

rebuild: clean software

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
