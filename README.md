# X-Drive

STM32F407VET6-based motor controller firmware for an X-Drive (holonomic) robot chassis.

## Hardware
- **MCU**: STM32F407VETx (ARM Cortex-M4, 168 MHz)
- **Drive**: X-Drive (4-wheel holonomic)

## Toolchain
- STM32CubeIDE 1.19.x
- STM32CubeMX (`.ioc` configuration in `X-Drive.ioc`)
- ARM GCC (arm-none-eabi-gcc)

## Project Structure

```
X-Drive/
├── Core/
│   ├── Inc/          # Application headers
│   ├── Src/          # Application source (main.c, gpio.c, tim.c, ...)
│   └── Startup/      # Startup assembly
├── Drivers/
│   ├── CMSIS/        # CMSIS core headers
│   └── STM32F4xx_HAL_Driver/  # ST HAL driver
├── X-Drive.ioc       # CubeMX project configuration
├── STM32F407VETX_FLASH.ld  # Flash linker script
└── STM32F407VETX_RAM.ld    # RAM linker script
```

## Building
1. Open STM32CubeIDE
2. File → Import → Existing Projects into Workspace
3. Select the `X-Drive` folder
4. Build with Ctrl+B (Debug or Release configuration)
