# Edge Impulse firmware for Raspberry Pi Pico (RP2040 & RP2350)

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Raspberry Pi Pico based development boards, specifically Raspberry Pi Pico and Arduino Nano RP2040 Connect. This device supports Edge Impulse device features, including ingestion and inferencing.

**Note: Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions [here](https://docs.edgeimpulse.com/docs/raspberry-pi-rp2040) for a prebuilt firmware and instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.**


## Requirements
### Hardware

- Raspberry Pi Pico based development boards, preferably Raspberry Pi Pico 1 or 2 or Arduino Nano RP2040 Connect.
- (Optional) If you are using Raspberry Pi Pico 1 or 2, [Grove Shield for Pi Pico](https://wiki.seeedstudio.com/Grove_Shield_for_Pi_Pico_V1.0/) makes it easier to connect external sensors for data collection/inference.

### Tools
The below instructions assume you are using Debian-based Linux distribution. Alternative instructions for those
using Microsoft Windows or Apple macOS are provided in [Getting started with Pico guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) (Sections 9.1 and 9.2).

To build firmware, you will need pico-sdk, CMake, a
cross-platform tool used to build the software, and the GNU Embedded Toolchain for Arm. You can install both these via apt from the command line.

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

**NOTE**
Ubuntu and Debian users might additionally need to also install ```libstdc++-arm-none-eabi-newlib```.

You'll need PICO SDK to compile the firmware. You can obtain it from https://github.com/raspberrypi/pico-sdk and then specify PICO_SDK_PATH environmental variable, that would point to exact PICO SDK location on your system.
E.g.

```bash
cd ~/
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk
export PICO_SDK_PATH="~/pico-sdk"
```

## Building the application

### Raspberry PI Pico (RP2040)
For projects based on the RP2040 build with:
```bash
mkdir build && cd build
cmake ..
make -j4
```

### Raspberry PI Pico 2 (RP2350)
Use following build commands to build for RP2350 based hardware:
```bash
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2
make -j4
```

### Raspberry PI Pico 2 WiFi (RP2350)
Building for the Pico 2 with WiFi controller use following build script:
```bash
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2_w
make -j4
```

### Debug enabled build
If you want to enable debug build (with lots of printing, specifically in data acquisition and flash read/write parts of the code), run
```bash
mkdir build && cd build
cmake .. -DDEFINE_DEBUG=ON
make -j4
```
Make sure you add the correct `-DPICO_BOARD` flag depending on the hardware.
Replace 4 with a number of parallel build processes suitable for your system.

## Flashing the binary
The fastest method to load firmware onto a Pico board for the first time is by mounting it as a USB Mass
Storage Device. Doing this allows you to drag a file onto the board to program the flash. Go ahead and connect the
Raspberry Pi Pico to your computer using a micro-USB cable, making sure that you hold down the BOOTSEL button as you do so, to force it into USB Mass Storage Mode. Drag the ei_rp2040_firmware.uf2 or ei_rp2350_firmware.uf2 file from build folder to newly appeared USB Mass Storage device.

## Troubleshooting

- For Arduino RP2040 Connect, if you need to keep using Arduino IDE after you finished using Edge Impulse firmware, just connect the board to your computer and flash any code to the board.

- If you are using Windows and experience isssue described in https://github.com/edgeimpulse/firmware-pi-rp2040/issues/1, you need to upgrade the tinyusb submodule of pico-sdk at least to commit https://github.com/hathach/tinyusb/commit/6ec5174d8b24d1f01a443c74a0a3dbee00523efc. After upgrade, remember to re-run cmake command.

