"""
micropython driver for the ms4525do differential pitot tube sensor
recreated from the original library meant for c++/arduino (https://github.com/bolderflight/ms4525do)

the sensor outputs a typical i2c string where 14bits correspond to pressure reading, following 11bits for temperature

typical usage:
from machine import I2C, Pin
import ms4525

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
sensor = ms4525.MS4525DO(i2c, addr=0x28, p_max=1.0, p_min=-1.0)  # ±1 psi sensor
ok = sensor.begin()
if ok:
    if sensor.read():
        print(sensor.pres_pa, sensor.die_temp_c, sensor.status_str())
"""

from machine import I2C
import time

#conversion constants
PSI_TO_PA = 6894.757  # 1 psi = 6894.757 Pa

#typical device mapping constants (per datasheet examples)
_OUTPUT_MIN = 1638    # ~10% of 16383
_OUTPUT_MAX = 14745   # ~90% of 16383

#raw counts maxima from header semantics
_P_CNT = 16383  # 14-bit counts
_T_CNT = 2047   # 11-bit counts

#temperature conversion bounds in datasheet
_T_MIN = -50.0
_T_MAX = 150.0


class MS4525DO:
    STATUS_GOOD = 0x00
    STATUS_STALE = 0x02
    STATUS_FAULT = 0x03

    #default address is 0x28
    def __init__(self, i2c, addr=0x28, p_max=1.0, p_min=-1.0, output_min=_OUTPUT_MIN, output_max=_OUTPUT_MAX):

        if not isinstance(i2c, I2C):
            #micropython I2C class check may vary; skip strict check to be flexible
            pass

        self.i2c = i2c
        self.addr = addr
        self.p_max = float(p_max)
        self.p_min = float(p_min)
        self.output_min = int(output_min)
        self.output_max = int(output_max)

        #internal storage
        self._pres_cnts = 0
        self._temp_cnts = 0
        self._status = self.STATUS_FAULT
        self.pres_psi = 0.0
        self.pres_pa = 0.0
        self.temp_c = float('nan')

        #precalc linear conversion factors for counts->psi
        #avoid division by zero
        denom = float(self.output_max - self.output_min)
        if denom == 0:
            self._psi_scale = 0.0
        else:
            self._psi_scale = (self.p_max - self.p_min) / denom
        self._psi_offset = self.p_min - self.output_min * self._psi_scale

    def begin(self):
        """
        Probe the device once to see if it responds on the I2C bus.
        Returns True if a read can be performed; False otherwise.
        """
        try:
            #a simple read attempt: read 4 bytes. device might not like repeated reads,
            #but most MS4525 devices will respond with the 4-byte frame.
            data = self.i2c.readfrom(self.addr, 4)
            #if no exception, assume present. We don't parse here.
            return True
        except Exception:
            return False

    def read(self):
        """
        Read the 4-byte packet from the sensor and update internal attributes.

        Returns True if status is OK (STATUS_GOOD), False otherwise.
        """
        try:
            raw = self.i2c.readfrom(self.addr, 4)
        except Exception:
            #I2C error: preserve last values, mark fault
            self._status = self.STATUS_FAULT
            return False

        if len(raw) != 4:
            self._status = self.STATUS_FAULT
            return False

        b0 = raw[0]
        b1 = raw[1]
        b2 = raw[2]
        b3 = raw[3]

        #status is top 2 bits of first byte
        status = (b0 & 0xC0) >> 6
        #pressure is 14 bits: lower 6 bits of b0 and full b1
        pres_cnts = ((b0 & 0x3F) << 8) | b1
        #temperature is 11 bits: b2:b3 >> 5
        temp_cnts = ((b2 << 8) | b3) >> 5

        #store raw
        self._status = status
        self._pres_cnts = pres_cnts
        self._temp_cnts = temp_cnts

        #convert pressure counts to psi using linear scaling:
        #psi = (pres_cnts - OUTPUT_MIN) * (p_max - p_min) / (OUTPUT_MAX - OUTPUT_MIN) + p_min
        #using precomputed scale/offset:
        self.pres_psi = (pres_cnts * self._psi_scale) + self._psi_offset

        #convert to Pascals
        self.pres_pa = self.pres_psi * PSI_TO_PA

        #temperature conversion (11-bit counts): map (0-T_CNT) to (T_MIN-T_MAX)
        #temp_c = (temp_cnts / T_CNT) * (T_MAX - T_MIN) + T_MIN
        try:
            self.temp_c = (float(temp_cnts) / float(_T_CNT)) * (float(_T_MAX) - float(_T_MIN)) + float(_T_MIN)
        except Exception:
            self.temp_c = float('nan')

        return self._status == self.STATUS_GOOD

    #properties for external access (names similar to C++ header)
    @property
    def pres_pa(self):
        return self._pres_pa if hasattr(self, "_pres_pa") else getattr(self, "pres_pa", self.pres_pa)

    @pres_pa.setter
    def pres_pa(self, v):
        #allow external reads/writes if needed (keeps attribute)
        self._pres_pa = float(v)

    @property
    def die_temp_c(self):
        return getattr(self, "temp_c", float('nan'))

    @property
    def status(self):
        return int(self._status)

    def status_str(self):
        if self._status == self.STATUS_GOOD:
            return "GOOD"
        elif self._status == self.STATUS_STALE:
            return "STALE"
        elif self._status == self.STATUS_FAULT:
            return "FAULT"
        else:
            return "UNKNOWN"

    #convenience: get tuple (pres_pa, temp_c, status)
    def get(self):
        return (self.pres_pa if hasattr(self, "_pres_pa") else getattr(self, "pres_pa", 0.0),
                self.die_temp_c,
                self._status)

    #string representation
    def __repr__(self):
        return "<MS4525DO addr=0x{:02X} pres_pa={:.2f} Pa temp_c={:.2f}C status={}>".format(
            self.addr,
            (self._pres_pa if hasattr(self, "_pres_pa") else getattr(self, "pres_pa", 0.0)),
            self.temp_c,
            self.status_str()
        )