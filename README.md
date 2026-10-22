# observer01

STM32G431CBUx motor control firmware project.

## Project structure

- `Src/`, `Inc/`: application and motor control sources and headers.
- `Drivers/`: STM32 HAL and CMSIS dependencies.
- `MCSDK_v6.4.1-Full/`: motor control SDK dependencies.
- `MDK-ARM/observer01.uvprojx`: Keil uVision project.
- `observer01.ioc`: STM32CubeMX configuration.
- `ftl/`: code generation templates.

## Build

Open `MDK-ARM/observer01.uvprojx` in Keil MDK. The project specifies ARM Compiler 6.21 and Keil STM32G4xx device pack 2.0.0. Build the `observer01` target after installing the required toolchain and device pack.

Generated build outputs and local IDE state are excluded from Git. Third-party components retain their original license terms.
