//--------------------------------------------------------------------------------------------------
//                                  _            _     
//                                 | |          | |    
//      ___ _ __ ___  ___ _   _ ___| |_ ___  ___| |__  
//     / _ \ '_ ` _ \/ __| | | / __| __/ _ \/ __| '_ \. 
//    |  __/ | | | | \__ \ |_| \__ \ ||  __/ (__| | | |
//     \___|_| |_| |_|___/\__, |___/\__\___|\___|_| |_|
//                         __/ |                       
//                        |___/    Engineering (www.emsystech.de)
//
// Filename:    
// Description: 
//
// Open Source Licensing 
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Dieses Programm ist Freie Software: Sie k�nnen es unter den Bedingungen
// der GNU General Public License, wie von der Free Software Foundation,
// Version 3 der Lizenz oder (nach Ihrer Option) jeder sp�teren
// ver�ffentlichten Version, weiterverbreiten und/oder modifizieren.
//
// Dieses Programm wird in der Hoffnung, dass es n�tzlich sein wird, aber
// OHNE JEDE GEW�HRLEISTUNG, bereitgestellt; sogar ohne die implizite
// Gew�hrleistung der MARKTF�HIGKEIT oder EIGNUNG F�R EINEN BESTIMMTEN ZWECK.
// Siehe die GNU General Public License f�r weitere Details.
//
// Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
// Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
//         
// Author:      Martin Steppuhn
// History:     01.01.2011 Initial version
//--------------------------------------------------------------------------------------------------

#ifndef SDP6X_H
#define SDP6X_H

//=== Includes =====================================================================================	

#include "std_c.h"

//=== Preprocessing directives (#define) ===========================================================

#define	ERROR_SDP6X_I2C				        1
#define	ERROR_SDP6X_CRC_PRESSURE	        2
#define	ERROR_SDP6X_CRC_HUMIDITY	        4
#define ERROR_SDP6X_CRC_USER_REGISTER       8
#define ERROR_SDP6X_CRC_ADV_USER_REGISTER   16
#define ERROR_SDP6X_CRC_EEPROM              32
#define ERROR_SDP6X_ADDRESS_INVALID         64

#define SENSOR_DEVICE_STATE_IDLE 0
#define SENSOR_DEVICE_STATE_CONTINOUS_READ 1




//=== Type definitions (typedef) ===================================================================

//=== Global constants (extern) ====================================================================

//=== Global variables (extern) ====================================================================

extern uint8	Sht21Error;

//=== Global function prototypes ===================================================================

uint8 Sdp6xRead(uint8 address, int16 *pressure);
int Sdp6xSensorInit(uint8 address, uint8 samplingResolutionNr);
uint8 Sdp6xSetupMeasurement(uint8 address, uint8 newState);
void Sdp6xDelayByMeasurementTime();

void Sdp6xPrintError(void);
uint8 Sdp6xGetNumberOfResolutions();
const char* Sdp6xGetResolution(uint8 index);
int Sdp6xChangeI2CAddress(uint8 currentAddress, uint8 newAddress);

#endif
