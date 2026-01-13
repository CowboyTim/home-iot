# DHT11, BME280, and BMP280 sensor

## Software

### Supported Sensors

- **DHT11**: Temperature and Humidity
- **BME280**: Temperature, Humidity, and Pressure
- **BMP280**: Temperature and Pressure
- **LDR**: Light/Illuminance
- **MQ-135**: Air Quality (CO2)
- **APDS-9930**: Illuminance and Color
- **SenseAir S8**: CO2 (NDIR)
- **SE95**: Temperature

### Required Libraries

For BME280 and BMP280 support, install the following Arduino libraries:
- Adafruit Unified Sensor
- Adafruit BME280 Library
- Adafruit BMP280 Library

Install via Arduino IDE Library Manager or PlatformIO.

## Build

```
DEV_PORT=/dev/ttyACM0 bash ./build.sh build
```

## Deploy

```
DEV_PORT=/dev/ttyACM0 bash ./build.sh deploy
```

## Config

For e.g. the DUCO S8 SE95, you can use the following command to set up the device:

```
bash setup_s8_se95_duco.sh /dev/ttyACM0 <SSID> <PASS> <dest IP> <dest PORT> <some KEY name, e.g.: homeoffice>
```

## Config over with AT Commands
```
picocom /dev/ttyACM0 --imap lfcrlf  --echo --omap crcrlf

