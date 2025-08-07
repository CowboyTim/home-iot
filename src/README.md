# DHT11 and BME280 sensor

## Software

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

