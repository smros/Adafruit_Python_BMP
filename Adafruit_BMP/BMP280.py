# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import time

import Adafruit_GPIO.I2C as I2C


# BMP280 default address, as set on my system
BMP280_I2CADDR           = 0x76

# Operating Modes
BMP280_ULTRALOWPOWER     = 0
BMP280_STANDARD          = 1
BMP280_HIGHRES           = 2
BMP280_ULTRAHIGHRES      = 3

# BMP280 Registers
BMP280_T1           	 = 0x88  # R   Calibration data (16 bits)
BMP280_T2           	 = 0x8A  # R   Calibration data (16 bits)
BMP280_T3                = 0x8C  # R   Calibration data (16 bits)
BMP280_P1          	 = 0x8E  # R   Calibration data (16 bits)
BMP280_P2                = 0x90  # R   Calibration data (16 bits)
BMP280_P3                = 0x92  # R   Calibration data (16 bits)
BMP280_P4                = 0x94  # R   Calibration data (16 bits)
BMP280_P5                = 0x96  # R   Calibration data (16 bits)
BMP280_P6                = 0x98  # R   Calibration data (16 bits)
BMP280_P7                = 0x9A  # R   Calibration data (16 bits)
BMP280_P8                = 0x9C  # R   Calibration data (16 bits)
BMP280_P9                = 0x9E  # R   Calibration data (16 bits)

BMP280_CONTROLMEAS       = 0xF4
BMP280_TEMPDATA          = 0xFA
BMP280_PRESSUREDATA      = 0xF7
BMP280_ID				 = 0xD0
BMP280_RESET			 = 0XE0
BMP280_CONFIG	 		 = 0XF5
BMP280_STATUS		=0xF3

# Commands
BMP280_READTEMPCMD       = 0x2E
BMP280_READPRESSURECMD   = 0x34

OSRT_MODES={
		0:0,
		1:0x01,
		2:0x02,
		4:0x03,
		8:0x04,
		16:0x07
		}
		
OSRP_MODES={
		0:0,
		1:0x01,
		2:0x02,
		4:0x03,
		8:0x04,
		16:0x07
		}

POWER_MODES={	'SLEEP':0X0,
				'FORCED':0X01,
				'NORMAL':0X03
			}

IIR_MODES={	'OFF':0,
		2:0x01,
		4:0x02,
		8:0x03,
		16:0x04
		}		
		
TIME_SB={	0.5:0x0,
		62.5:0x01,
		125:0x02,
		250:0x03,
		500:0x04,
		1000:0x05,
		2000:0x06,
		4000:0x07
		}
		

class BMP280(object):
	def __init__(self, mode=BMP280_STANDARD, address=BMP280_I2CADDR, 
							 busnum=I2C.get_default_bus()):
		self._logger = logging.getLogger('Adafruit_BMP.BMP280')
		# Check that mode is valid.
		if mode not in [BMP280_ULTRALOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, BMP280_ULTRAHIGHRES]:
			raise ValueError('Unexpected mode value {0}.  Set mode to one of BMP280_ULTRALOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, or BMP280_ULTRAHIGHRES'.format(mode))
		self._mode = mode
		# Create I2C device.
		self._device = I2C.Device(address, busnum)
		# Load calibration values.
		self._load_calibration()
		
	def set_control_register(self, OSRT=0, OSRP=0,POWER_MODE='NORMAL'):
		cntl_reg=0
		print 'Arguments:'
		print OSRT_MODES[OSRT]
		print OSRP_MODES[OSRP]
		POWER_MODES[POWER_MODE]
		cntl_reg+= (OSRT_MODES[OSRT]<<5) + (OSRP_MODES[OSRP]<<2) + (POWER_MODES[POWER_MODE])
		print 'Control Reg setting:'
		print cntl_reg
		self._device.write8(BMP280_CONTROLMEAS, cntl_reg)
		# save in object
		self.cntl_register=cntl_reg
		
	def set_config_register(self, TIME_STANDBY=4000, IIR='OFF'):
		cfg_reg=0
		cfg_reg= (TIME_SB[TIME_STANDBY]<<5) + (IIR_MODES[IIR]<<2)
		print 'My time standby is:'
		print TIME_SB[TIME_STANDBY]
		print 'My IIR Mode is:'
		print IIR_MODES[IIR]
		print 'Config Reg Setting:'
		print cfg_reg
		#print '{0:08b}'.format(ord(str(cfg_reg)))
		self._device.write8(BMP280_CONFIG, cfg_reg)
		# save in object
		self.config_register=cfg_reg
		

	def _load_calibration(self):
		self.cal_T1 = self._device.readU16LE(BMP280_T1)   # INT16
		self.cal_T2 = self._device.readS16LE(BMP280_T2)   # INT16
		self.cal_T3 = self._device.readS16LE(BMP280_T3)   # INT16
		self.cal_P1 = self._device.readU16LE(BMP280_P1)   # UINT16
		self.cal_P2 = self._device.readS16LE(BMP280_P2)   # UINT16
		self.cal_P3 = self._device.readS16LE(BMP280_P3)   # UINT16
		self.cal_P4 = self._device.readS16LE(BMP280_P4)     # INT16
		self.cal_P5 = self._device.readS16LE(BMP280_P5)     # INT16
		self.cal_P6 = self._device.readS16LE(BMP280_P6)     # INT16
		self.cal_P7 = self._device.readS16LE(BMP280_P7)     # INT16
		self.cal_P8 = self._device.readS16LE(BMP280_P8)     # INT16
		self.cal_P9 = self._device.readS16LE(BMP280_P9)     # INT16
		self._logger.debug('T1 = {0:6d}'.format(self.cal_T1))
		self._logger.debug('T2 = {0:6d}'.format(self.cal_T2))
		self._logger.debug('T3 = {0:6d}'.format(self.cal_T3))
		self._logger.debug('P1 = {0:6d}'.format(self.cal_P1))
		self._logger.debug('P2 = {0:6d}'.format(self.cal_P2))
		self._logger.debug('P3 = {0:6d}'.format(self.cal_P3))
		self._logger.debug('P4 = {0:6d}'.format(self.cal_P4))
		self._logger.debug('P5 = {0:6d}'.format(self.cal_P5))
		self._logger.debug('P6 = {0:6d}'.format(self.cal_P6))
		self._logger.debug('P7 = {0:6d}'.format(self.cal_P7))
		self._logger.debug('P8 = {0:6d}'.format(self.cal_P8))
		self._logger.debug('P9 = {0:6d}'.format(self.cal_P9))

	def _load_datasheet_calibration(self):
		# Set calibration from values in the datasheet example.  Useful for debugging the
		# temp and pressure calculation accuracy.
		self.cal_T1 = 27504
		self.cal_T2 = 26435
		self.cal_T3 = -1000
		self.cal_P1 = 36477
		self.cal_P2 = -10685
		self.cal_P3 = 3024
		self.cal_P4 = 2855
		self.cal_P5 = 140
		self.cal_P6 = -7
		self.cal_P7 = 15500
		self.cal_P8 = -14600
		self.cal_P9= 6000

	def read_raw_temp(self):
		"""Reads the raw (uncompensated) temperature from the sensor."""
		# Wait for the measurement to be ready.  Check for measurement ready, 
		while True:
			if not(self.get_status()):
				break
		msb = self._device.readU8(BMP280_TEMPDATA)
                lsb = self._device.readU8(BMP280_TEMPDATA+1)
                xlsb = self._device.readU8(BMP280_TEMPDATA+2)
                raw = ((msb << 12) + (lsb << 4) + (xlsb>>4))
		self._logger.debug('Raw temp 0x{0:X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_raw_pressure(self):
		"""Reads the raw (uncompensated) pressure level from the sensor."""
		msb = self._device.readU8(BMP280_PRESSUREDATA)
		lsb = self._device.readU8(BMP280_PRESSUREDATA+1)
		xlsb = self._device.readU8(BMP280_PRESSUREDATA+2)
		raw = ((msb << 12) + (lsb << 4) + (xlsb>>4) )
		self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_temperature(self):
		"""Gets the compensated temperature in degrees celsius."""
		UT = self.read_raw_temp()
		# Datasheet value for debugging:
		#UT = 27898
		# Calculations below are taken straight from section 3.5 of the datasheet.
		var1=((((UT>>3)-(self.cal_T1<<1))) * (self.cal_T2))>>11
		var2=(((((UT>>4) - (self.cal_T1)) * ((UT>>4) - (self.cal_T1))) >>12) * (self.cal_T3)) >>14
		t_fine= var1 + var2
		T = (t_fine * 5 + 128) >>8
		self._logger.debug('Calibrated temperature {0} C'.format(T))
		return T

	def read_pressure(self):
		"""Gets the compensated pressure in Pascals."""
		print 'Using REAL UT/UP:'
		UT = self.read_raw_temp()
		UP = self.read_raw_pressure()

		# Debug override
		#print 'Using DEBUG UT and UP'
		#UT=519888
		#UP=415148
		
		print 'UT:'
		print UT

		print 'UP'
		print UP

        	var1=( (UT/16384.0) -(self.cal_T1/1024.0) ) * self.cal_T2
		print 'UT var1:'
		print var1
        	var2= (((UT/131072.0) - (self.cal_T1/8192.0)) * ( (UT/131072.0) - (self.cal_T1/8192.0))) * self.cal_T3
        	print 'UT var2:'
		print var2

		t_fine= var1 + var2
		print 'Tfine:'
		print t_fine
		var1=(t_fine/2.0)-64000.0
		print 'Line1, var1'
		print var1
		var2= var1 * var1 * self.cal_P6 / 32768.0
		print 'Line2, var2'
		print var2
		var2= var2 + (var1*self.cal_P5*2.0)
		print 'Line3, var2'
		print var2
		var2= (var2/4.0) + (self.cal_P4 * 65536.0)
		print 'line4, var2:'
		print var2
		var1= (var1 * var1 * self.cal_P3 / 524288.0 + (var1*self.cal_P2) )/524288.0
		print 'line5, var1:'
		print var1
		var1=(1.0 + var1/32768.0 ) * self.cal_P1
		print 'line6, var1:'
		print var1
		if var1 == 0:
			return 0
		p = 1048576.0 - UP
		p = (p-(var2/4096.0))* 6250.0 /var1
		print 'line8, p'
		print p
		var1 = self.cal_P9 * p * p /2147483648.0
		print 'line9, var1:'
		print var1
		var2 = self.cal_P8 * p / 32768.0
		print 'line10, var2:'
		print var2
		p = p + (var1 + var2 + self.cal_P7)/16.0
		print 'line11, p:'
		print p

		# Datasheet values for debugging:
		#UT = 27898
		#UP = 23843
		# Calculations below are taken straight from section 3.5 of the datasheet.
		# Calculate true temperature coefficient B5.
		

		self._logger.debug('Pressure {0} Pa'.format(p))
		return p

	def read_altitude(self, sealevel_pa=101325.0):
		"""Calculates the altitude in meters."""
		# Calculation taken straight from section 3.6 of the datasheet.
		pressure = float(self.read_pressure())
		altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
		self._logger.debug('Altitude {0} m'.format(altitude))
		return altitude

	def read_sealevel_pressure(self, altitude_m=0.0):
		"""Calculates the pressure at sealevel when given a known altitude in
		meters. Returns a value in Pascals."""
		pressure = float(self.read_pressure())
		self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
		p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
		return p0
		
	def read_control_meas_register(self):
		"""Get the current control register"""
		current_cntl_meas_reg=self._device.readU8(BMP280_CONTROLMEAS)
		return current_cntl_meas_reg
		
	def read_config_register(self):
		current_config_reg=self._device.readU8(BMP280_CONFIG)
		return current_config_reg

	def get_status(self):
		
		measuring =False
		current_status_register=self._device.readU8(BMP280_STATUS)
		print 'In Get status:'
		print current_status_register
		measuring_status=current_status_register & 0x08
		if measuring_status>0:
			measuring=True
		return measuring

	def read_id(self):
		return self._device.readU8(BMP280_ID)

	def reset(self):
		self._device.write8(BMP280_RESET, 0xB6)

if __name__ == "__main__":

	test=BMP280()
	test.reset()
	print 'ID:'
	print test.read_id()


	print test.cal_T1
	print test.cal_T2
	print test.cal_T3
	print test.cal_P1
	print test.cal_P2
	print test.cal_P3
	print test.cal_P4
	print test.cal_P5
	print test.cal_P6
	print test.cal_P7
	print test.cal_P8
	print test.cal_P9

	# Read Control Meas

	
	print 'Current C.Meas. Reg:'
	print test.read_control_meas_register()
	
	print 'Current Config Reg:'
	print test.read_config_register()

	print 'Setting Config Reg'
	test.set_config_register(TIME_STANDBY=4000,IIR='OFF')

	print 'Current Config Reg:'
	print test.read_config_register()
	
	print 'Setting Control Meas. Reg'
	test.set_control_register(OSRT=4, OSRP=4,POWER_MODE='NORMAL' )

	print 'Current C.Meas. Reg:'
	print test.read_control_meas_register()
	
		
#	print 'Checking status:'
#	while 1:
	
#		print test.get_status()
#		time.sleep(0.1)

	print 'Getting raw temp:'
	print test.read_raw_temp()

	print 'Getting Temp:'
	print test.read_temperature()

	print 'Getting raw pressure:'
	print test.read_raw_pressure()

#	print 'Loading DEBUG caltable:'
#	test._load_datasheet_calibration()
	
	print 'Getting pressure:'
	print test.read_pressure()

	
	
