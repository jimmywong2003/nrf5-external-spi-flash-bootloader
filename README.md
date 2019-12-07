# nrf5-external-spi-flash-bootloader

This example is to demo how to use the external SPI flash for Device Firmware Upgrade (DFU) on the nRF52 Series.  The target is to minimize the size of the bootloader particular on the nRF52810. The total size of the bootloader + its setting is only 12KB comparing to original bootloader SDK (32KB).

It is target to do the application firmware upgrade only.

The proto buffer file format is as 

| Size | Description |
| --- | --- |
| Word | Data field length = 16 |
| Word | Firmware version |
| Word | Firmware Size |
| Word | Firmware CRC - 32 |
| Word | CRC32 for the 12-byte data |

All the details are placed at [my blog](https://jimmywongbluetooth.wordpress.com/2019/03/12/slim-spi-bootloader-on-nrf52-series-for-device-firmware-upgrade/).


## Requirement

* NRF52840 DK 
* SDK 15.2 
* IDE: Segger Embedded Studio

## Note
This is the demo source code.  It doesn't have warranty all the qualify of this source code.
