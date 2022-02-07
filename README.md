# VLC-Electronic-photo-frame Project

![VLC-Electronic-photo-frame Demo picture](/'STL 3D Files'/Demo.jpg"VLC-Electronic-photo-frame")

This is a project using led to transmit pictures to STM32F469DISCO kit through UART protocol.

> Copy these ST generate file -> STM32Cube_FW_F4_V1.21.0\Projects\STM32469I-Discovery\Applications\LibJPEG\LibJPEG_Decoding -> to my project folder

## Files

### STM32F469 Code

- /Inc
- /Src
- /SW4STM32

### PCTool

- PCTool.py -> This is a Python script for PC to test the transmit function.

### STL 3D Files

- This folder contain 3d files and Demo picture.

## Architecture

|Name|Layer|
|---|---|
|LIB JPEG    |Middleware|
|FATFS       |Middleware|
|UART        |Drivers|
|GPIO        |Drivers|
|DMA         |Drivers|
|DMA2D       |Drivers|
|DSI         |Drivers|
|LTDC        |Drivers|
|SDRAM       |Drivers|
|Otm8009a LCD|Drivers|
|NVIC        |Drivers|
|STM32F469   |Hardware|

