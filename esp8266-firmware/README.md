# About

This project has the firmware for a finger haptic feedback device. It uses micropython runnin on Mini ESP8266 board  (NodeMCU ESP8266, CH340, 4m Flash 12F chip.)

### Setup

* Ensure you have usb drivers installed for the CH340 chip.

* [Installation Instructions](https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html)

```
python -m venv .env
# active the venv
pip install esptool
```

### Flashing the firmware

* You can look in device manager to find the port of the microcontroller.
```
esptool.py --port COM7 erase_flash
esptool.py.exe --port COM7 --baud 460800 write_flash --flash_size=detect 0 .\esp8266-20210902-v1.17.bin
```


### Connect to the python REPL

* Over USB
  * Use Putty or a similar tool to connect to the device, using the COM port listed in device manager. Serial connection with 115200 baud rate and 8 data bits, no parity, no stop bits.
* Over WiFi
  * <https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html#deploying-the-firmware>:
    > After a fresh install and boot the device configures itself as a WiFi access point (AP) that you can connect to. The ESSID is of the form **MicroPython-xxxxxx** where the x’s are replaced with part of the MAC address of your device (so will be the same everytime, and most likely different for all ESP8266 chips). The password for the WiFi is **micropythoN** (note the upper-case N). Its IP address will be **192.168.4.1** once you connect to its network. 

## Coding

Recommended: use VSCode and the Pymakr extension. Tutorial on using this extension is available [here](https://randomnerdtutorials.com/micropython-esp32-esp8266-vs-code-pymakr/#connecting).
* In particular, you can use the "run this file" and "upload project" commands.
* Edit `pymakr.json` to include "Silicon Labs" as an autoconneect_comport_manufacturer.

## Pinout

You must use the GPIO pin numbers in micropython.
<!-- ![firmware pinout](pinout.webp) -->
![figs/pinout.png](https://i2.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/05/ESP8266-NodeMCU-kit-12-E-pinout-gpio-pin.png?w=817&quality=100&strip=all&ssl=1)

  
## Haptic Effects

**Actually, we aren't using the haptic interface boards - instead, we're directly controlling it with a transister.**
[Link](https://learn.adafruit.com/adafruit-drv2605-haptic-controller-breakout/python-circuitpython)
![](https://cdn-learn.adafruit.com/assets/assets/000/072/594/medium800/adafruit_products_DRV_Waveforms.png?1552347698)