# STM32F103C8T6 + ICM-20608-G (I2C 400 kHz, 100 Hz stream)

## What is included
- `main.c`: replacement for the CubeIDE-generated `Core/Src/main.c`.
- `python/plot_icm20608.py`: Windows live plotter and CSV logger.

This firmware reads the ICM-20608-G over **I2C1 only**. It sends the readings to the laptop through **USART1** and a USB-to-TTL serial adapter. UART is not used to communicate with the IMU.

## What I checked in your generated files
- `MX_I2C1_Init()` already sets `ClockSpeed = 400000`, 7-bit addressing and duty-cycle 2.
- `stm32f1xx_hal_msp.c` already configures PB6 as I2C1_SCL and PB7 as I2C1_SDA in alternate-function open-drain mode and enables the I2C1 peripheral clock. **You do not need to manually toggle these pins.** Calling `MX_I2C1_Init()` triggers the HAL MSP setup.
- TIM2 prescaler 7199 and period 99 give 100 Hz when the timer clock is 72 MHz (the supplied clock setup uses an 8 MHz HSE and PLL x9; APB1 is /2, so the timer clock is doubled to 72 MHz).
- `stm32f1xx_it.c` already calls `HAL_TIM_IRQHandler(&htim2)`, so no interrupt-file change is needed.
- USART1 is PA9 TX / PA10 RX and was configured at 115200 baud. The replacement `main.c` changes it to 230400 baud to leave more transmission margin for 100 Hz CSV output.

## Wiring

### ICM-20608-G to Blue Pill
- SCL -> PB6
- SDA -> PB7
- GND -> GND
- VDD and VDDIO -> 3.3 V-compatible supply, according to your breakout board schematic
- AD0 -> GND for I2C address 0x68; AD0 high selects 0x69 (change `ICM20608_ADDR` if high)
- CS must be strapped to the level required for I2C mode on the actual module (normally high); check the breakout/module documentation.
- INT is not needed for this polling implementation.
- SDA/SCL need pull-ups to 3.3 V. Some breakout boards already have them.

Do not apply 5 V to bare-chip supply pins. Verify the breakout board before powering it.

### Blue Pill to Windows laptop
Use a **USB-to-TTL UART adapter**:
- Blue Pill PA9 (USART1_TX) -> adapter RX
- Blue Pill GND -> adapter GND
- Optional adapter TX -> Blue Pill PA10 (USART1_RX), not needed for this one-way stream
- Use 3.3 V logic on the adapter UART pins. Do not connect an RS-232 voltage-level port.
- Open the adapter's COM port in the Python app.

ST-LINK is for programming/debugging and is not automatically a serial data port.

## CubeIDE integration
1. Make a backup of your project.
2. Replace only `Core/Src/main.c` with this package's `main.c`.
3. Keep your existing `main.h`, `stm32f1xx_hal_msp.c`, `stm32f1xx_it.c`, HAL config and startup files.
4. Ensure the project is for STM32F103C8Tx and the HAL I2C, TIM and UART modules are enabled.
5. Build, flash with ST-LINK, then connect the USB-to-TTL adapter.
6. If your board's HSE is not 8 MHz, update the clock configuration/HSE value to match the actual crystal before trusting the 400 kHz bus or 100 Hz timer timing.

## Sensor setup used by the firmware
- `WHO_AM_I` register `0x75` must return `0xAF`.
- I2C address defaults to 0x68 (AD0 low).
- Accelerometer range: +/-2 g, conversion 16384 LSB/g.
- Gyroscope range: +/-250 degrees/second, conversion 131 LSB/(degree/second).
- DLPF configured; sample-rate divider 9 gives 1 kHz / (1 + 9) = 100 Hz.
- Reads the 14-byte burst from `0x3B`: accel XYZ, temperature, gyro XYZ.
- TIM2 triggers a 10 ms sample flag. I2C reads and UART writes happen in the main loop, not inside the interrupt callback.

## Python plotting
In PowerShell, from the `python` folder:
```powershell
py -m pip install pyserial matplotlib
py -m serial.tools.list_ports
py plot_icm20608.py --port COM5
```
Replace `COM5` with the actual COM port. Close other serial terminals first; only one application can normally own the port. The script saves `icm20608_log.csv` in the current directory.

## Expected serial format
Lines beginning with `#` are diagnostic messages. Data rows contain:
`timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C`

## Troubleshooting
- `WHO_AM_I` not `0xAF`: verify supply, common ground, SDA/SCL wiring, AD0 address, CS/I2C-mode strap, and pull-ups.
- I2C HAL errors: start at 100 kHz temporarily if wiring is long or pull-ups are wrong; once stable, return to 400 kHz.
- Values frozen or nonsensical: verify the exact chip/module and ensure the IMU is not held in reset.
- No COM port: the USB-to-TTL adapter must have a Windows driver; ST-LINK alone does not create a COM port.
- No plotted data: match baud to 230400, close other serial applications and confirm diagnostic text appears.

This is raw IMU streaming/validation firmware. It does not perform step detection or 3D position reconstruction.
