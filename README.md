# Teensy_Camera
Camera class to support several cameras 

The driver is primarily ported from the OpenMV and ESP32_camera Sensor drivers. "The OpenMV project aims at making machine vision more accessible to beginners by developing a user-friendly, open-source, low-cost machine vision platform."  The developers were kind enough to make their hard work open-source under MIT License:

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

## Omnivision OV2640, OV7670 and OV7675 Camera Commands
![image](https://github.com/mjs513/TeensyMM_Camera/assets/5366213/741eec49-0e48-4f5c-aae8-9802df28ca7a)

![image](https://github.com/mjs513/TeensyMM_Camera/assets/5366213/c8dba7e0-88c6-4081-ad6f-6886267fdaa9)

##Himax HM01B0 and HM0360 Camera Commands

![image](https://github.com/mjs513/TeensyMM_Camera/assets/5366213/f5080383-8d9d-49d1-9302-b86390d79124)

## GalaxyCore GC2145

![image](https://github.com/mjs513/TeensyMM_Camera/assets/5366213/9aa58744-274a-4e27-a9ce-174408b577d7)

Notes on camera commands:
1. The yellow highlighted commands are common across all cameras.
2. Th orange hightlighted commands for the OV2640 are unique to that camera and is accessed by using the device constructor
3. The unhiglighted commands are those that unique to that camera


## 
