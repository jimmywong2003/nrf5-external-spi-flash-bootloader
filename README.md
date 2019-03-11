# nrf5-slim-spi-flash-dfu

Slim SPI External Data flash DFU (bootloader = 12KB, including bootloader Setting) on the nRF5 SDK 

This example is to show how to use the external spi data flash for the DFU (device firmware upgrade).  If you are using the nRF52810 for development, it would have a limited flash size for the application.

## Size for Application level

For example, the size of the softdevice S112v5.1 is 96KB (the smallest size Bluetooth Stack).  This version is working with SDK 14.2.

The size of original serial UART bootloader (secured bootloader) is 24KB + 2 extra pages (1 page for bootloader setting + 1 page for the MBR setting).

The remainder flash size for the application is only 192KB - 96KB (softdevice) - 24KB (bootloader) - 8KB (2 pages for setting) = 64KB.

However, if you are using the slim SPI bootloader (target to do the application upgrade), the size of such bootloader is only 8KB.

The flash size for application level is changed to 192KB - 96KB (Stack) - 12KB (bootloader) = 84KB.

As this example is required to use the external data flash, the nRF52840 (including the QSPI flash to act as external data flash) is used for testing.

## Requirement

* NRF52840 DK 
* SDK version: It works with all Nordic SDK version
* IDE: Segger Embedded Studio

## Note
This is the demo source code.  It doesn't have warranty all the qualify of this source code.
