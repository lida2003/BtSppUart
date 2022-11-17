# BtSppUart

BtSppUart is try to connect BetaFlight Configurator remotely though bluetooth virtual com port.

# Solution

## Hardware

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-H2 | ESP32-S3 |
| :---------------: | :---: | :------: | :------: | :------: | :------: |
| TBD | Tested | TBD | TBD | TBD | TBD |

## Software 

ESP-IDF GATT SERVER SPP Example is based.
>[esp-idf-v5.0](https://github.com/espressif/esp-idf/tree/release/v5.0) \examples\bluetooth\bluedroid\classic_bt\ [bt_spp_acceptor](https://github.com/espressif/esp-idf/tree/release/v5.0/examples/bluetooth/bluedroid/classic_bt/bt_spp_acceptor)

# How to Dev & Test

## Step 1: create a working dir

> $ mkdir test
> 
> $ cd test

## Step 2: clone ESP IDF v5.0

> $ git clone git@github.com:espressif/esp-idf.git
> 
> $ cd esp-idf
> 
> $ git checkout release/v5.0

## Step 3: setup dev environment

> $ ./install.sh

## Step 4: clone BleSppUart

> $ cd ..
> 
> $ git clone git@github.com:lida2003/BtSppUart.git

## Step 5: enter dev environment

> $ cd esp-idf
> 
> $ . ./export.sh

## Step 6: build BleSppUart

> $ cd BtSppUart
> 
> $ idf.py set-target ESP32
> 
> $ idf.py build

*Note: try idf.py set-target target, if you have other module or dev boards.*

## Step 7: update firmware

> $ idf.py -p /dev/ttyUSB0 flash monitor

*Note: change ttyUSBx, x=serial port of the ttl module.*

# Summary

It works, detailed info see [Connecting SpeedyBee or BetaFlightConfigurator with self make SPP device](https://blog.csdn.net/lida2003/article/details/127901773)