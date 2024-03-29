# DUCO CO2 sensor

## Connector

See [connector](https://github.com/CowboyTim/home-iot/blob/main/duco-co2/kicad/duco_co2.svg)

## Software

This is a tool to read out the CO2 Sensor for the DUCO Box Silent that's based
on Sensair's S8 sensor, see [Sensair
S8](https://senseair.com/product/senseair-s8-residential/) and the datasheet
[Sensair S8
Datasheet](https://rmtplusstoragesenseair.blob.core.windows.net/docs/publicerat/PSP126.pdf).
It also has a SE95 for temperature reading/compensation in the same package.

This uses the S8\_UART library that's in the Arduino Library, see
[S8_UART](https://github.com/jcomas/S8_UART) and
[s8_uart](https://www.arduino.cc/reference/en/libraries/s8_uart/)

To build and deploy this, using an esp32-c3 super mini board, run:
```
DEV_PORT=/dev/ttyACM0 bash ./build.sh deploy
```
