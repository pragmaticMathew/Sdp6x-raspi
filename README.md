# Sdp6x-raspi
Read out program to retrieve sensor data from Sensirion SDP6x sensors. Based on work of Martin Steppuhn for SHT21.

# Purpose
This program is intended to read out one or more SDP610 differential pressure Sensors from Sensirion and to write the sensor values either to STDOUT or a an output file / queue. 
Reference to the sensor: https://sensirion.com/de/produkte/katalog/SDP610-25Pa
Please note that the sensor is not recommended for new designs, however when I started this project i found it very well suited and I'm still impressed by the accuracy and performance of this sensor.

# wiring
In any case consult the data sheet and other available information. It's possible to connect the sensor with it's four wires to the raspberry PI's I2C bus.
Notes:
- Do not use the internal I2C for the shields
- Raspberry PI has already Pull-up resistors. Do not add extra resistiors unless you know what you are doing.

# SW setup of the raspberry pi
You need to enable the I2c-dev kernel space driver. This may be done on the console of the raspberry pi, type:
	
sudo raspi-config

# output format
The program writes the value of each set up sensor in a single output text line in milli Pascals (mPA), separated by semicolon ';'.

# compiling, running
use the given makefile: make clean, make
For changing I2C Adress of sensor you need to compile the code with enabled command line option (see main.c)
If you want to start the application as daemon it is recommended to use sudo to be able to write the log file.

# how to use
Usage:Sdp6x options] [outputfilename]
  outputfilename is optional, if omitted, stdout will be used

# options
-h print help
-delay:<x>                                      Delays sampling by x ms - note: no spaces in option!
-daemon                                         Start program as daemon. Note that you need to have root rights for this. Terminate it by the following commands: ps aux | grep %s and kill -15 <pid>
-sample-resolution:<x>                          Specifies sampling resolution in bits. Sample rate is influcenced by resolution. Possible values are from 9 to 16. Note: no spaces in option!
-changeI2CAddress:<currentAddr>:<newAddress>    Use this parameter to change the I2C Address from a specific sensor. Make sure that only one sensor is attached to the bus with the same currentAddress. Note this call may destroy a sensor by overwriting calibration data. Do not use on incompatible device, no warranty / liability.
-I2CAdr:<d1>:<d2>                               Defines the device address of the sensors. Use hex values, e.g. 0x40:0x41. At maximum two sensors are supported. If you have only one sensor write e.g. 0x40:0x00. If the parameter is omitted 0x40 will be used
# example
  
  Sdp6x -sample-resolution:16 -daemon outputfile
	

# open issues, upcoming improvements

TODO: Command line parser supports until up to 2 sensors yet
TODO: First 1-2 measurements always fail 
IMPROVEMENT: Integrate proper logging framework, add posibility to log into logfile
