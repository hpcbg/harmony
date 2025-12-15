# Arduino firmware for connection of M5StickCPlus2 to FIWARE

This repository contains Arduino firmware for the M5StickCPlus2 which will broadcast information from M5 Dual Button and M5 Angle Sensor PORT.B.I/O which are connected to ports 0 and 5 of the Pb.HUB. The hub is connected to the GROVE by I2C at address 0x61.

Before compilation and upload of the firmware you need to create `config.h` file in `M5StickCPlus2-to-FIWARE` folder. Sample configuration file is the `M5StickCPlus2-to-FIWARE/config.h.tpl`. You need to set the Wi-Fi and the FIWARE parameters. The `config.h` is not tracked by the repository.

The firmware is compiled with Arduino IDE 2.3.6 with M5Stack library version 0.4.6 and the board manager is M5Stack version 3.2.5.
