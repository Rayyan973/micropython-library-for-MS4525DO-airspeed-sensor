## MicroPython library for MS4525DO Differential Pressure Airspeed Sensor

This library was based on the original C++ implementation by [bolderflight](https://github.com/bolderflight/ms4525do) and was re-written in micropython for use mainly in the Raspberry Pi Pico.
The AMS 5915 sensor outputs a typical I2C packet, consisting of a 14-bit pressure and 11-bit temperature reading.

Typical usage would be:
```
from machine import I2C, Pin
import ms4525

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
sensor = ms4525.MS4525DO(i2c, addr=0x28, p_max=1.0, p_min=-1.0)  # ±1 psi sensor
ok = sensor.begin()
if ok:
    if sensor.read():
        print(sensor.pres_pa, sensor.die_temp_c, sensor.status_str())
```

More details can be found in the code.
To use, simply copy the ms4525.py file into your micropython environment.
