## About
This project is a port of the [ESPectrum](https://github.com/EremusOne/ESPectrum) ZX Spectrum emulator to the Lilygo T-Display board (ESP32).

[![Ganzfeld demo](https://img.youtube.com/vi/NA3OUMB8lso/0.jpg)](https://youtube.com/shorts/NA3OUMB8lso)

## Motivation
The original ZX Spectrum features a display resolution of 256x192 pixels (active area), with the full video output (including borders) typically being 320x240 or 320x200 pixels. The Lilygo T-Display, with its 240x135 pixel screen, covers over 90% of the ZX Spectrum's width and about 70% of its height. While some vertical cropping is necessary, the majority of classic ZX Spectrum content including [demos](https://www.youtube.com/watch?v=lWUDqy2VHw8) remains visually impressive and enjoyable on this compact display.

A key part of the ZX demo scene is music, so this port also implements robust Bluetooth A2DP audio playback for wireless sound output. The result is a portable, all-in-one ZX Spectrum experience that fits in your pocket!

## Game/Demo Files

ZX Spectrum games and demos in `.z80` format are included in the `/data` directory. These files are bundled into a SPIFFS filesystem image (`spiffs.bin`) during the build process. After flashing the main firmware, you can upload `spiffs.bin` to the device using `esptool.py` at the correct SPIFFS partition offset. For this project, the SPIFFS partition offset is `0x240000` (see `ESPecpart2.csv`).

Example command:

    esptool.py --chip esp32 --baud 921600 --port COM3 write_flash -z 0x240000 bin/spiffs.bin

Once uploaded, the `.z80` files will be available for loading and running directly from the emulator menu.

## Flashing

To flash the firmware onto your Lilygo T-Display board:

1. Install [esptool](https://github.com/espressif/esptool) if you haven't already:

   pip install esptool

2. Connect your board via USB and determine its COM port (e.g., COM3).

3. Run the following command in your terminal (replace COM3 with your port if different):

   esptool.py --chip esp32 --baud 921600 --port COM3 write_flash -z 0x1000 bin/firmware.bin

- Make sure the board is in bootloader mode (usually by holding the BOOT button while pressing RESET, then releasing BOOT).
- After flashing, press RESET to start the emulator.