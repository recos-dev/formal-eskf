.DEFAULT_GOAL := build

PYTHON ?= python3
CONFIG_FILE ?= .config
BUILD_DIR ?= build/demo
JOBS ?= 2
CMAKE_ARGS ?=
PX4_DIR ?=
PACKAGE_DIR ?= output/px4-package

.PHONY: menuconfig defconfig configure build test px4-install px4-export help

menuconfig defconfig:
	$(PYTHON) scripts/configure.py $@ --config "$(CONFIG_FILE)"

configure:
	$(PYTHON) scripts/configure.py configure --config "$(CONFIG_FILE)" --build-dir "$(BUILD_DIR)" -- $(CMAKE_ARGS)

build: configure
	cmake --build "$(BUILD_DIR)" --parallel "$(JOBS)"

test: build
	ctest --test-dir "$(BUILD_DIR)" --output-on-failure --no-tests=error

px4-install:
	@test -n "$(PX4_DIR)" || { echo 'error: specify PX4_DIR=/path/to/PX4-Autopilot' >&2; exit 2; }
	$(PYTHON) integrations/px4/install.py --px4 "$(PX4_DIR)" --config "$(CONFIG_FILE)"

px4-export:
	$(PYTHON) integrations/px4/install.py --export "$(PACKAGE_DIR)" --config "$(CONFIG_FILE)"

help:
	@printf '%s\n' \
	    'make menuconfig  Edit and save .config' \
	    'make defconfig   Reset .config to project defaults' \
	    'make configure   Apply .config to CMake (creates defaults if absent)' \
	    'make [build]     Configure and build' \
	    'make test        Build and run CTest' \
	    'make px4-install PX4_DIR=/path/to/PX4-Autopilot  Install configured module' \
	    'make px4-export  Export configured module to PACKAGE_DIR=output/px4-package' \
	    'Options: CONFIG_FILE=.config BUILD_DIR=build/demo JOBS=2 CMAKE_ARGS="..."'
