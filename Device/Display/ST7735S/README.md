# ST7735S 1.8-inch TFT

`Device_ST7735S` implements the supplied 128×160 module's write-only 4-wire
SPI protocol. The module uses RGB565 pixels and the supplied panel-specific
ST7735S initialization/gamma sequence. Rotation-dependent GRAM offsets follow
the supplied sample: portrait `x+2/y+1`, landscape `x+1/y+2`.

The Device owns no MCU pins. G3519 maps display transport 0 to SPI0 Mode 0,
MSB first, 10 MHz on PB3/PB2, with PC9 CS, PC8 DC, PB23 active-low RESET and
PA30 active-high backlight in BSP/SysConfig. The module requires 3.0–3.3 V;
do not power it from 5 V.

Initialization is a bounded blocking startup operation of about 420 ms.
Drawing uses a 64-pixel stack chunk and no framebuffer; full-screen RGB565
therefore does not consume about 40 KiB of SRAM. Font storage, text layout,
widgets and product refresh policy belong above Device and are not included.
