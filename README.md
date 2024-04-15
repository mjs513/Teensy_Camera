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

NOTES ON ```setPins``` command:
1.  The Arduino HM01B0 only supports 4 pin access to the camera.  The upper 4 bit lines are not connected
2.  The Sparkfun Micromod ML board will support board 4-bit and 8=bit modes
3.  4-bit mode is determined by only defining 4-pins in the command.

For the teensy micromod board the following pin assignments are used so Flexio can use dma if using all 8 data pins.
```
camera.setPins(29, 10, 33, 32, 31, 40, 41, 42, 43, 44, 45, 6, 9);
```
on the experimental SDRAM board
```
camera.setPins(7, 8, 21, 46, 23, 40, 41, 42, 43, 44, 45, 6, 9);
```

### Camera Selection/Constructor
This follows the method used by the arduino giga camera library:
```
//#define ARDUCAM_CAMERA_HM01B0
//#define ARDUCAM_CAMERA_HM0360
#define ARDUCAM_CAMERA_OV2640
//#define ARDUCAM_CAMERA_OV7670
//#define ARDUCAM_CAMERA_OV7675
//#define ARDUCAM_CAMERA_GC2145

#if defined(ARDUCAM_CAMERA_HM0360)
#include "TMM_HM0360/HM0360.h"
HM0360 himax;
Camera camera(himax);
#define CameraID 0x0360

#elif defined(ARDUCAM_CAMERA_HM01B0)
#include "TMM_HM01B0/HM01B0.h"
HM01B0 himax;
Camera camera(himax);
#define CameraID 0x01B0

#elif defined(ARDUCAM_CAMERA_OV2640)
#include "TMM_OV2640/OV2640.h"
OV2640 omni;
Camera camera(omni);
#define CameraID 0x2642

#elif defined(ARDUCAM_CAMERA_OV7670)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID 0x7676

#elif defined(ARDUCAM_CAMERA_OV7675)
#include "TMM_OV767X/OV767X.h"
OV767X omni;
Camera camera(omni);
#define CameraID 0x7673

#elif defined(ARDUCAM_CAMERA_GC2145)
#include "TMM_GC2145/GC2145.h"
GC2145 galaxycore;
Camera camera(galaxycore);
#define CameraID 0x2145

#endif
```

## EXTRAS
Arducam Windows App: https://docs.arducam.com/Arduino-SPI-camera/Legacy-SPI-camera/Software/Host-Debug-Tools/
![image](https://github.com/mjs513/TeensyMM_Camera/assets/5366213/1a47f1f6-d65f-4d1d-9d19-ce5a6f507a92)

Processing App:

## EXAMPLES

