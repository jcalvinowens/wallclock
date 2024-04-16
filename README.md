NTP Wall Clock
==============

This repository contains the hardware design and firmware for a simple digital
wall clock which synchronizes with NTP servers over wifi.

https://www.youtube.com/shorts/xX9GlqPRe6Y

![](https://static.wbinvd.org/img/wallclock/clock.jpg)

TL;DR:

* [Schematic](schematic.pdf)
* [Layout](layout.pdf)
* [Firmware](main/main.c)
* [Gerber](prod/v012-GERBER.zip), [BOM](prod/v012-BOM.csv), and [CPL](prod/v012-CPL.csv)

Design
------

The primary goal of this project was to replace several "atomic" clocks in my
house which never successfully synchronize to [WWVB](https://en.wikipedia.org/wiki/WWVB)
with clocks that sync to NTP servers. Just for fun, a secondary goal was to
accurately display milliseconds.

The clock is designed around the ESP32-C3, a low power single core RISCV SoC
with wifi and bluetooth support. Four chained shift registers drive small
MOSFETs to switch each of the 32 LED segments individually, to avoid rolling
shutter artifacts in photos and videos.

The board is powered by a 1A 12V power brick, through a series diode for reverse
polarity protection. A small 3.3V LDO powers the esp32 and registers, and a tiny
PWM-capable constant current regulator powers each LED segment.

The clock can really count milliseconds, as demonstrated by
[this video](https://www.youtube.com/watch?v=3n7UssnawdA) taken with a 328fps
ov9281 Y8 image sensor attached to the CSI input of a Raspberry Pi 4b. The
exposure time is much less than a millisecond, even though the framerate isn't
close to 1khz.

A minimal web interface allows the user to change brightness and clock mode. The
clock also has a static mode that allows you to display arbitrary 4-character
messages via HTTP POST requests.

![](https://static.wbinvd.org/img/wallclock/webui.png)

I more or less followed the [ESP32-C3 design guidelines](https://www.espressif.com/sites/default/files/documentation/esp32-c3_hardware_design_guidelines_en.pdf)
when laying out the board, with the exception of RF impedence matching. The RF
trace and my antennas are all 50 Ohm, which, with the 35 Ohm LNA output, results
in a VSWR of ~1.4:1. The loss (versus a matched line) is less than an S-unit, so
it wasn't worth the money for a prototype. Because wifi uses PSK and QAM, there
is additional loss due to reflections, but for a prototype it is negligable.

Datasheets
----------

* MCU [ESP32-C3FH4](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf)
* 40MHz Crystal [Q22FA12800332](https://datasheet.lcsc.com/lcsc/1810171117_Seiko-Epson-Q22FA12800332_C255899.pdf)
* 32.768KHz Crystal [M332768PWNAC](https://datasheet.lcsc.com/lcsc/2202131930_JYJE-M332768PWNAC_C2838414.pdf)
* Status LED [HL-PC-3216S9AC](https://datasheet.lcsc.com/lcsc/2009091206_HONGLITRONIC-Hongli-Zhihui--HONGLITRONIC--HL-PC-3216S9AC_C499470.pdf)
* 7-Segment LED [SM453001L3-2](https://datasheet.lcsc.com/lcsc/1809291541_ARKLED-Wuxi-ARK-Tech-Elec-SM453001L3-2_C164873.pdf)
* 3.3V LDO [SPX3819M5-L-3-3/TR](https://datasheet.lcsc.com/lcsc/1810181735_MaxLinear-SPX3819M5-L-3-3-TR_C9055.pdf)
* 20mA LED Driver [NSI45020T1G](https://datasheet.lcsc.com/lcsc/2102202232_onsemi-NSI45020T1G_C129159.pdf)
* Protection diode [M7](https://datasheet.lcsc.com/lcsc/1811051611_BORN-M7_C266550.pdf)

Cost
----

I had five prototypes manufactured and assembled by JLCPCB in Feburary 2023. The
total cost per prototype was $49.66, broken down as follows:

* PCB (4-layer, ENIG): $12.84
* 4x Large LEDs: $9.56
* Rest of BOM and JLC SMT assembly: $22.19
* Shipping: $5.07

Obviously, that cost would decrease substantially if you made a larger order.

Only the "front" of the board (which is actually the back of the clock) has SMT
components: the other side is entirely covered by the large LEDs. This allows
you to pay for cheaper one-side assembly, and order the LEDs separately to
solder them on yourself.
