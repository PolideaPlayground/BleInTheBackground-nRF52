# BleInTheBackground - for nRF52

This project implements nRF52 application for the [React Native mobile app](https://github.com/PolideaPlayground/BleInTheBackground-iOS) testing BLE functionality in the background mode.

## Compiling

Place this project's folder into `YOUR_NRF_SDK/examples/ble_peripheral/` and use your favorite method for compilation. This project used nRF5_SDK_15.3.0 during development.

## Programming

1) [Download nRF Connect application for desktop](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-desktop).
2) Launch application and open "Programmer".
3) Select your device (e.g. PCA10040).
4) Click "Erase all".
5) Add application's hex file from `./hex` folder suitable for your device's model. (e.g. ble_app_background_test_pca10040_s132.hex).
6) Add SoftDevice hex file from `YOUR_NRF_SDK/components/softdevice/sXXX/hex` folder matching above SoftDevice version. (e.g. `YOUR_NRF_SDK/components/softdevice/s132/s132_nrf52_6.1.1_softdevice.hex`).
7) Click "Write".
8) App should be running now.

