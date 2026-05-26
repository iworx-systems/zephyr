.. zephyr:board:: numaker_pfm_m483

Overview
********

The NuMaker-PFM-M483KG is a development board featuring the Nuvoton
M483KGCAE MCU, a Cortex-M4F running at up to 192 MHz with 512 KB of
internal flash and 160 KB of SRAM.  An onboard Nu-Link2-Me ICE provides
USB-to-SWD debug and flashing.

Hardware
********

- Nuvoton M483KGCAE (Cortex-M4F @ 192 MHz, 512 KB flash, 160 KB SRAM, LQFP128)
- Three user LEDs on PH0 / PH1 / PH2
- Two user buttons on PG15 (SW2) and PF11 (SW3)
- High-speed USB device controller (HSUSBD) and full-speed USB OTG (USBD)
- Onboard Nu-Link2-Me debugger

Supported Features
******************

The default board configuration enables UART0 as the Zephyr console
(PB12 RX / PB13 TX) and exposes the standard set of GPIO, system clock,
and reset peripherals provided by the upstream Nuvoton M48x SoC support.

Programming and Debugging
*************************

Connect the onboard Nu-Link2-Me debugger over USB and use either OpenOCD
or the Nu-Link runner to flash and debug:

.. code-block:: console

   west build -b numaker_pfm_m483 samples/hello_world
   west flash

References
**********

- `NuMaker-PFM-M483KG product page <https://www.nuvoton.com/>`_
- `M483 series technical reference manual <https://www.nuvoton.com/>`_
