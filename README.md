# VLC-Electronic-photo-frame Project

This is a project using led to transmit pictures to STM32F469DISCO kit through UART protocol.

> Copy these ST generate file -> STM32Cube_FW_F4_V1.21.0\Projects\STM32469I-Discovery\Applications\LibJPEG\LibJPEG_Decoding

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

