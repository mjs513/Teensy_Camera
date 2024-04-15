# Teensy_Camera
Camera class to support Omnivision oV2640, OV7670 and OV7675, Himax HM01B0 and HM0360, and GalaxyCore GC2145 

The driver is primarily based on the Arduino Giga, OpenMV, ESP32_camera Sensor drivers. "The OpenMV project aims at making machine vision more accessible to beginners by developing a user-friendly, open-source, low-cost machine vision platform."  The developers were kind enough to make their hard work open-source under MIT License:

>The MIT License (MIT)
>
>Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
>Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
>
>Permission is hereby granted, free of charge, to any person obtaining a copy
>of this software and associated documentation files (the "Software"), to deal
>in the Software without restriction, including without limitation the rights
>to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
>copies of the Software, and to permit persons to whom the Software is
>furnished to do so, subject to the following conditions:
>
>The above copyright notice and this permission notice shall be included in
>all copies or substantial portions of the Software.
>....
>

## CAMERA COMMAND SET
https://github.com/mjs513/TeensyMM_Camera/blob/main/docs/Camera_sensors.xlsx


Notes on camera commands:
1. The yellow highlighted commands are common across all cameras.
2. Th orange hightlighted commands for the OV2640 are unique to that camera and is accessed by using the device constructor
3. The unhiglighted commands are those that unique to that camera


## IMAGE CAPTURE COMMANDS


## CAMERA SETUP COMMANDS
| Command | Description |
| --- | --- |
|   void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin, uint8_t en_pin, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4 = 0xff, uint8_t g5 = 0xff, uint8_t g6 = 0xff, uint8_t g7 = 0xff, TwoWire &wire = Wire); | Pin configuration set up |
| void debug(bool debug_on) | Turn library debug on and off |
| bool debug()  | Returns whether debug is set |
| usingGPIO()  | Lib suports GPIO and FLEXIO, this tels the lib to use GPIO to capture image |
| void useDMA(  | Tells FLEXIO to use DMA to capture image |
| bool useDMA()   | Returns whether DMA is set |
| void timeout(uint32_t timeout_ms) | Sets library timeouts to other than default for FLEXIO |
| uint32_t timeout()   | Returns timeout value |
| int16_t width   | Returns current camera width settings as modified by setWindow if supported and used |
| int16_t width   | Returns current camera height settings as modified by setWindow if supported and used |
| int16_t frameWidth(void)    | Returns framewidth as set by Framesize |
| int16_t frameHeight(void) | Returns frameheight as set by Framesize |

## EXAMPLES

