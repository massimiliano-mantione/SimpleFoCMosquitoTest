# SimpleFoCMosquitoTest

This is a PlatformIO project for the [Mosquito SimpleFOC](https://community.simplefoc.com/t/mosquito-board-new-design/1621/79) board.

Due to the STM32 board definitions in PlatformIO not having the STM32G031K8Ux MCU, I had to create the generic_g031K8Ux.json to allow platformio to compile, but with that done everything seems fine.

Programming uses an ST-link v2 (although other stm programmers should work), via the swd interface. 
