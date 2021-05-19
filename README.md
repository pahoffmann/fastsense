# HATSDF SLAM

**INSERT SUPER AWESOME GIF**

- [HATSDF SLAM](#hatsdf-slam)
  - [Dependencies](#dependencies)
  - [Build](#build)
  - [Base Design](#base-design)
  - [Runtime Parameters](#runtime-parameters)
  - [Run](#run)
  - [Cite](#cite)

## Dependencies
* Ubuntu 18.04
* make
* Xilinx 2020.1
* Hardware Base Design [Pre-Built]() | [DIY]()
* qemu (Tests)
## Build
**Note**: `HW_TARGET=sw_emu` is now default, compile hardware with `HW_TARGET=hw`

1. Source Xilinx Tools: `source <XILINX_INSTALL_DIR>/settings64.sh`
2. Source Xilinx Runtime: `source /opt/xilinx/xrt/setup.sh`
3. Are specific board files copied? In FastSense repo, run `cp -r base_design/board_files/* <VIVADO_INSTALL_DIR>/data/boards/board_files/`
4. Compile project
    * software only? `make software -j4`
    * hardware only? `make hardware -j4`
    * tests (hw+sw)? `make test -j4`
    * test software only? `make test_software`
    * test hardware only? `make test_hardware`
5. Package SD card image (if HW_TARGET=sw_emu qemu launch script and image is generated)
    * HATSDF SLAM: `make package HW_TARGET=hw -j4`
    * Tests: `make package_test HW_TARGET=hw -j4`

## Base Design
You can either [download a pre built base-design]() for the specific board, or build it yourself. To do just that please follow the instructions in the [Base Design repository]().

## Runtime Parameters

Example:

```
{
    "imu": {
        "bufferSize": 16,
        "filterSize": 25
    },

    "lidar": {
        "bufferSize": 1,
        "port": 2368,
        "pointScale": 1.0,
        "rings": 16,
        "vertical_fov_angle": 30.0
    },

    "registration": {
        "max_iterations": 200,
        "it_weight_gradient": 0.1,
        "epsilon": 0.01
    },

    "gpio": {
        "button_chip": "gpiochip0",
        "button_line": 0,
        "led_chip": "gpiochip1",
        "led_line": 0
    },

    "bridge": {
        "use_from": true,
        "use_to": true,

        "send_original": false,
        "send_preprocessed": false,
        "send_after_registration": true,

        "host_from": "192.168.1.245",

        "imu_port_from": 4444,
        "imu_port_to": 5555,

        "pcl_port_from": 3333,
        "pcl_port_to": 7777,

        "transform_port_to": 8888,
        "tsdf_port_to": 6666
    },

    "slam": {
        "max_distance": 600,
        "map_size_x": 201,
        "map_size_y": 201,
        "map_size_z": 121,
        "max_weight": 10,
        "initial_map_weight": 0.0,
        "map_update_period": 100,
        "map_update_position_threshold": 500,
        "map_path": "/data"
    }
}
```

## Run

On the SoC:

```
cd /mnt
./FastSense.exe
```

## Cite

```

```