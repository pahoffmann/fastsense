# FastSense

## How to build
* Phidgets api installed (see below)
* Xilinx prerequisites installed, see [Confluence Wiki](https://confluence.informatik.uni-osnabrueck.de/display/FAS/Petalinux+Installation)
* Xilinx platform already generated/generate in `base_design/platform/FastSense_platform/export/FastSense_platform`. For info look at [README.md](./base_design/README.md)
* environment variables
    * `XILINX_VIVADO`: Vivado install dir **with** version, e.g. /tools/xilinx/Vivado/2019.2/
    * `XILINX_VITIS`: Vitis install dir **with** version, e.g. /tools/xilinx/Vitis/2019.2/
    * source Vitis tools: `source $XILINX_VITIS/settings.64.sh`
    * source Xilinx runtime: `source /opt/xilinx/xrt/setup.sh`
* Install board files in `base_design/board_files`: `cp -r base_design/board_files/* ${XILINX_VIVADO}/data/boards/board_files/`
* source Vitis and XRT scripts: `source <VITIS_INSTALL_DIR>/settings64.sh` and `source /opt/xilinx/xrt/setup.sh`
* run `make`

## PhidgetsImu
* run `phidgets.sh`
    * installs `libusb`
    * installs Phidgets C Driver
    * updates `udev` rules