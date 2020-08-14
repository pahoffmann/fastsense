# FastSense

## How to build
* prerequisites installed, see [Confluence Wiki](https://confluence.informatik.uni-osnabrueck.de/display/FAS/Petalinux+Installation)
* platform already generated/generate in `base_design`. For info look at [README.md](./base_design/README.md)
* source Vitis and XRT scripts: `source <VITIS_INSTALL_DIR>/setting64.sh` and `source /opt/xilinx/xrt/setup.sh`
* run `make`

## PhidgetsImu
* run `phidgets.sh`
    * installs `libusb`
    * installs Phidgets C Driver
    * updates `udev` rules