#
# Variables
#

# Tools
CXX := g++
MKDIR_P := mkdir -p
SHELL := /bin/bash

# Global
BUILD_DIR := ./build
APP_NAME := imu_test
SRCS := $(wildcard ../../src/driver/imu/api/*.cpp) $(wildcard ../../src/driver/imu/msg/*.cpp) $(wildcard ../../src/driver/imu/*.cpp) test.cpp
OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
REMOVE = ../../src/driver/imu/
OBJS_CORRECTED = $(subst $(REMOVE),, $(OBJS))
DEPS := $(OBJS_CORRECTED:.o=.d)

INC_DIRS := ../../src 
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
$(BUILD_DIR)/$(APP_NAME): $(OBJS_CORRECTED)
	@echo "Link: $(APP_NAME)"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(OBJS_CORRECTED) -o $@ $(LDFLAGS)

# Compile software
$(BUILD_DIR)/%.o: ${REMOVE}%.cpp
	@echo "Compile: $<"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test.o: test.cpp
	@echo "Compile: $<"
	@$(MKDIR_P) $(dir $@)
	@$(CXX) $(CXXFLAGS) $< -o $@
