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
// Filename:    sht21.c / sdp6x.c
// Description: sht21.h / sdp6x.h
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
// Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
// der GNU General Public License, wie von der Free Software FouERROR_SHT21_CRC_TEMPdation,
// Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
// veröffentlichten Version, weiterverbreiten und/oder modifizieren.
//
// Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
// OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
// Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
// Siehe die GNU General Public License für weitere Details.
//
// Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
// Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
// Author:      Martin Steppuhn
// History:     01.01.2011 Initial version
//--------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------
// Change of the existing I2C handling a of SHT21 temperature sensor to handling of a Sensirion SDP 610
// Author:		Harries
// History:		01.08.2024 Fork from the original version of Martin Steppuhn
//				12.11.2024 Change to continous measurement with configurable resolution of the ADC
//				20.12.2024 Introduction of handling of more than one sensor of the same type
// -------------------------------------------------------------------------------------------------


//=== Includes =====================================================================================

#include "std_c.h"
#include "i2c.h"
#include "sdp6x.h"
#include "raspi.h"
#include <stdio.h>
#include <linux/i2c-dev.h>

//=== Preprocessing directives (#define) ===========================================================

#define MAX_IDENTIFIER_LENGTH 20
#define MAX_I2C_ADDRESS 256
#define INVALID_I2C_ADDRESS 0


//=== Type definitions (typedef) ===================================================================

typedef struct 
{
	char resolutionIdentifier[MAX_IDENTIFIER_LENGTH]; // how the resolution is defined as command line parameter
	uint8 pattern; // the bit pattern which has to be used for the sensor
	float maxMeasurementTime; // maximum time a measurement will take with the given resolution
}samplingOptions_t;


//=== Global constants =============================================================================


const samplingOptions_t g_sOptions[] =
{
  {"9", 0b000,0.9},
  {"10",0b001,1.5},
  {"11",0b010,2.6},
  {"12",0b011,4.9},
  {"13",0b100,9.4},
  {"14",0b101,18.5},
  {"15",0b110,36.7},
  {"16",0b111,73.2},
  {"default(14)", 0b101,18.5}
};

//=== Global variables =============================================================================

//=== Local constants  =============================================================================

//=== Local variables ==============================================================================

static uint8 g_Sdp6xError;
static uint8 g_resolutionNr;
static uint8 g_debugLevel = 0; // 0= originales logging, 1 = erweitertes Logging, 2 = detailiertes Logging
static uint8 g_currentDeviceAddr = INVALID_I2C_ADDRESS;
static uint8 g_deviceStatus[MAX_I2C_ADDRESS] = {SENSOR_DEVICE_STATE_IDLE}; 

//=== Local function prototypes ====================================================================

uint8 CalcSdp6xCrc(uint8 *data,uint8 nbrOfBytes);

// === implementation ===============================================================================

//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xGetNumberOfResolutions
// Function:  resturns the number of different resolutions the sensor supports
//--------------------------------------------------------------------------------------------------

uint8 Sdp6xGetNumberOfResolutions()
{
	return (sizeof(g_sOptions)/sizeof(g_sOptions[0])-1);
}

//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xGetResolution
// Function:  resturns the identifier of a specific resolution (e.g. for match against command line parameter)
//--------------------------------------------------------------------------------------------------

const char* Sdp6xGetResolution(uint8 index)
{
	if ((index<0) || (index >Sdp6xGetNumberOfResolutions())		)
	{
		return NULL;
	}
	else
	{
		return g_sOptions[index].resolutionIdentifier;
	}
}

//--------------------------------------------------------------------------------------------------
// Name:      validAddress
// Function:  resturns the validity of an address
//--------------------------------------------------------------------------------------------------

bool validAddress(uint8 address)
{
		if ((address<0x8) || (address>0x77))
		{
			return false;
		}
		else
		{
			return true;
		}

}

//--------------------------------------------------------------------------------------------------
// Name:      selectCorrectAddress
// Function:  Configures I2C user sprace driver to talk to the correct I2C Slave
//            
// Parameter: uint8 address
//             
// Return:    0, if successful, else I2C error
//--------------------------------------------------------------------------------------------------
int selectCorrectAddress(uint8 address)
{
	// Configure user space driver to talk to the correct I2C address
	if (address != g_currentDeviceAddr)
	{
		I2C_Setup(I2C_SLAVE, address);
		g_currentDeviceAddr = address;
		if(I2cError)
		{
			g_Sdp6xError |= ERROR_SDP6X_I2C;
			I2C_PrintError();
			return g_Sdp6xError;
		} 
	}
	return 0;
}



//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xSetupMeasurement
// Function:  Schaltet Sensor kontinuerliches lesen an / aus
//            
// Parameter: uint8 address
//            uint8 state (0= deactivate, 1 = activate
// Return:    0, if sucessfull else ERROR_Sdp6x_x
//--------------------------------------------------------------------------------------------------
uint8 Sdp6xSetupMeasurement(uint8 address, uint8 newState)
{			
	g_Sdp6xError = 0;
	if (!validAddress(address))
	{
		g_Sdp6xError |= ERROR_SDP6X_ADDRESS_INVALID;
		return 1;
	}
    
	// select proper address
	if (selectCorrectAddress(address)!=0)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		return 1;
	}

	if ((g_deviceStatus[address] == SENSOR_DEVICE_STATE_IDLE) && (newState == SENSOR_DEVICE_STATE_CONTINOUS_READ))
	{
		g_deviceStatus[address] = SENSOR_DEVICE_STATE_CONTINOUS_READ;
		I2C_Write1(0xF1);			// start pressure measurement
				// wait to pass first measurement
				// DelayMs(g_sOptions[g_resolutionNr].maxMeasurementTime);

		if (g_debugLevel>1)
		{
			fprintf(stderr, "Sdp6xSetupMeasurement: Command 0xF1 (start) sent\n");
		}
	}
	else if ((g_deviceStatus[address] == SENSOR_DEVICE_STATE_CONTINOUS_READ) && (newState == SENSOR_DEVICE_STATE_IDLE))
	{
		g_deviceStatus[address] = SENSOR_DEVICE_STATE_CONTINOUS_READ;
		I2C_Write1(0xF6);			// stop pressure measurement
		if (g_debugLevel>1)
		{
			fprintf(stderr, "Sdp6xSetupMeasurement: Command 0xF6 (stop) sent\n");
		}
	}
	return g_Sdp6xError;
}


//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xDelayByMeasurementTime
// Function:  Should be called after the readout of the last sensor to wait until the next measurement values are available
//            
// Parameter: none
// Returns  : None
//--------------------------------------------------------------------------------------------------
void Sdp6xDelayByMeasurementTime()
{
	// delay read out of measurement by maximum measurement time of the sensor by given resolution
	DelayMs((uint32)(g_sOptions[g_resolutionNr].maxMeasurementTime*1.1));
}


//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xRead
// Function:  polls a single differential pressure from SDP610
//            
// Parameter: address: IC2 Address of the device
//            pressure: differential pressure in milli pascal [mPa]
// Return:    0, if sucessfull, else ERROR_Sdp6x_x
//--------------------------------------------------------------------------------------------------
uint8 Sdp6xRead(uint8 address, int16 *pressure)

{
	int16	val;
	uint8	buf[4];
		
	g_Sdp6xError = 0;
	if (!validAddress(address))
	{
		g_Sdp6xError |= ERROR_SDP6X_ADDRESS_INVALID;
		return 1;
	}
    
	// select proper address
	if (selectCorrectAddress(address)!=0)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		return 1;
	}


	//=== Lese Druck =================================================
	I2C_Read(buf,3);			// read pressure data
	
	if (g_debugLevel>1)
	{
		fprintf(stderr, "Sdp6x_Read: I2C-Address: 0x%x, following values have been read for pressure: Byte 0:%x\t Byte 1:%x\t Byte 2:%x\n", address, buf[0], buf[1], buf[2]);
	}
	
	if(buf[2] == CalcSdp6xCrc(buf,2))  // check CRC
	{
		if (g_debugLevel>1)
		{
			fprintf(stderr, "### Sdp6x_Read: CRC ok\n");
		}
		val = buf[0];
		val <<= 8;
		val += buf[1];
        // @TODO: The value needs to be divided to device specific factor. This factor is hard coded for the SDP610 with +-25 pa measurement range
		*pressure = 1000*val/1200; // return in milli-Pascal
	}
	else
	{
		g_Sdp6xError |= ERROR_SDP6X_CRC_PRESSURE;
	}
	
	if(I2cError) g_Sdp6xError |= ERROR_SDP6X_I2C;

	return g_Sdp6xError;
}

//------------------------------------------------------------------------------
// Name: CalcSdp6xCrc
// Function: Calculates CRC Polynom for read values
//            
// Parameter: data = Pointer to first byte, nbOfBytes
// Return: calculated crc value
//------------------------------------------------------------------------------
uint8 CalcSdp6xCrc(uint8 *data,uint8 nbrOfBytes)
{
	// CRC
	//const u16t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001
	
	uint8 byteCtr,bit,crc;

	crc = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ 
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ 0x131;
				else 		crc = (crc << 1);
		}
	}
	return(crc);
}

//--------------------------------------------------------------------------------------------------
// Name:      PrintSdp6xError
// Function:  Print error flags as readable text.
//            
// Parameter: -
// Return:    -
//--------------------------------------------------------------------------------------------------
void Sdp6xPrintError(void)
{	
	if(g_Sdp6xError & ERROR_SDP6X_I2C)						fprintf(stderr, "ERROR I2C-Port\n");
	if(g_Sdp6xError & ERROR_SDP6X_CRC_PRESSURE)				fprintf(stderr, "ERROR Pressure CRC\n");
	if(g_Sdp6xError & ERROR_SDP6X_CRC_HUMIDITY)				fprintf(stderr, "ERROR Humidity CRC\n");
	if(g_Sdp6xError & ERROR_SDP6X_CRC_USER_REGISTER)		fprintf(stderr, "ERROR user register  CRC\n");
	if(g_Sdp6xError & ERROR_SDP6X_CRC_ADV_USER_REGISTER)	fprintf(stderr, "ERROR advanced user register CRC\n");
    if(g_Sdp6xError & ERROR_SDP6X_CRC_EEPROM)	            fprintf(stderr, "ERROR EEPROM CRC\n");
}

//--------------------------------------------------------------------------------------------------
// Name:      SetPatternInBuffer
// Function:  Sets the bit pattern for resolution in input buffer,  starting from Bit 1 based on samplingResolutionNr
// Parameters: buf - Array where pattern is to be set
//             samplingResolutionNr - Index to select the pattern from g_sOptions
// Return:    0 if successful, error code otherwise
//--------------------------------------------------------------------------------------------------
uint8 SetPatternInBuffer(uint8 *buf, uint8 samplingResolutionNr) {
    if (samplingResolutionNr > Sdp6xGetNumberOfResolutions()) {
        return 1; // Error: Invalid samplingResolutionNr
    }
    
    // Clear the bits 1 to 3 in buf (masking with 0b1110_0001 to keep other bits intact)
    buf[0] &= 0xE1;

    // Insert pattern shifted to start from bit 1
    buf[0] |= (g_sOptions[samplingResolutionNr].pattern << 1);

    return 0; // Success
}



//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xSensorInit
// Function:  Soft reset configure for ic2 polling
//            
// Parameter: -
// Return:    0 if init failed, >0 for success
//--------------------------------------------------------------------------------------------------
int Sdp6xSensorInit(uint8 address, uint8 samplingResolutionNr)
{
	uint8	buf[4];
	uint8 checksum;
    // check if resolution is valid, otherwise stop here
	if ((samplingResolutionNr <0) || (samplingResolutionNr>Sdp6xGetNumberOfResolutions()))
	{
		g_resolutionNr = Sdp6xGetNumberOfResolutions();
		fprintf(stderr, "sensorInit: Error: Invalid parameter samplingResolutionNr: %d\n", samplingResolutionNr); 
		return 0;
	}
	g_resolutionNr = samplingResolutionNr;

	g_Sdp6xError = 0;
	if (!validAddress(address))
	{
		g_Sdp6xError |= ERROR_SDP6X_ADDRESS_INVALID;
		return 1;
	}
    
	// select proper address
	if (selectCorrectAddress(address)!=0)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		return 1;
	}



	//============================== Softreset ==================================================
	
	I2C_Write1(0xFE);			// softreset < 15ms
	DelayMs(2000);

   
   // =========================== User register read  + change ==============================
   I2C_Write1(0xE3);		
   I2C_Read(buf,3); // 0 = MSB, 1 = LSB, 2 = CRC
   checksum = CalcSdp6xCrc(buf, 2);
	if (checksum != buf[3])
	{
		g_Sdp6xError |= ERROR_SDP6X_CRC_USER_REGISTER;
	}

   if (g_debugLevel>1)
   {
	fprintf(stderr, "sensorInit: I2C-Address 0x%x, User Register MSB: %x, LSB: %x, CRC: %x, calc. CRC: %x\n", address, buf[0], buf[1], buf[2], checksum);
	}

	// set now continous measurement mode 
	// read Sensirion Dokumentation SF_AN_Flow_DP_I2C_Continuous_Mode_With_Hold-Master_Disabled_V1_D2.pdf
	// Continous measurement: LSB Bit 3 set
	buf[1] |= 0x08;
	// user register, MSB, LSB
	I2C_Write3(0xE2,buf[0], buf[1]);

	//  Paranoia
   if (g_debugLevel>1)
   {
		DelayMs(2000);
		I2C_Write1(0xE3);			// command user register read
		I2C_Read(buf,3); // 0 = MSB, 1 = LSB, 2 = CRC
		checksum = CalcSdp6xCrc(buf, 2);
		if (checksum != buf[3])
		{
			g_Sdp6xError |= ERROR_SDP6X_CRC_USER_REGISTER;
		}

		fprintf(stderr, "sensorInit: I2C-Address 0x%x, User Register re-read after change: MSB: %x, LSB: %x, CRC: %x, calc. CRC: %x\n", address,  buf[0], buf[1], buf[2], checksum);
		DelayMs(500);
   }
	
   // ============================ Advanced User register read ==============================
   I2C_Write1(0xE5);			// Command advanced user register read
   I2C_Read(buf,3); // 0 = MSB, 1 = LSB, 2 = CRC
   
   checksum = CalcSdp6xCrc(buf, 2);
   if (checksum != buf[3])
	{
		g_Sdp6xError |= ERROR_SDP6X_CRC_ADV_USER_REGISTER;
	}

   if (g_debugLevel>1)
   {
	fprintf(stderr, "sensorInit: I2C-Address 0x%x, Advanced User Register MSB: %x, LSB: %x, CRC: %x, calc. CRC: %x\n", address, buf[0], buf[1], buf[2], checksum);
	}
	
	//=== advanced USER register set ==================================================
    // advanced user register, MSB, LSB
	
    // set now resolution pattern in MSB
	if (SetPatternInBuffer(buf, samplingResolutionNr) != 0)
	{
		// should not happen
		fprintf(stderr, "sensorInit: Error: Invalid parameter samplingResolutionNr: %d\n", samplingResolutionNr); 
		return 0;
	}

	// disable now HOLD-MASTER in LSB
	// see Sensirion Dokumentation SF_AN_Flow_DP_I2C_Continuous_Mode_With_Hold-Master_Disabled_V1_D2.pdf
	// HOLD-MASTER disable: LSB Bit 1 unset
	buf[1] &=~(1 << 1);  // 1 shift once to left and invert+and
	
    // Write back advanced user register
    I2C_Write3(0xE4,buf[0], buf[1]); // default 12 bit

	//  Paranoia
   if (g_debugLevel>1)
   {
		DelayMs(2000);
		I2C_Write1(0xE5);			// command user register read
		I2C_Read(buf,3); // 0 = MSB, 1 = LSB, 2 = CRC
		checksum = CalcSdp6xCrc(buf, 2);
		if (checksum != buf[3])
		{
			g_Sdp6xError |= ERROR_SDP6X_CRC_ADV_USER_REGISTER;
		}
		fprintf(stderr, "### sensorInit: I2C-Address 0x%x, Advanced User Register read again after changes: MSB: %x, LSB: %x, CRC: %x, calc. CRC: %x\n", address, buf[0], buf[1], buf[2], checksum);
   }
   DelayMs(2000);
   return 1;
}



//--------------------------------------------------------------------------------------------------
// Name:      updateI2CAddressPattern
// Function:  calculates updated eeprom values for new I2cAddress
//            
// Parameter: uint8 msb - Msb of the eeprom
//            uint8 lsb - lsb of the eeprom
//            uint8 pattern - the new address
//            uint8* out_msb - pointer to the updated msb
//            uint8* out_lsb - pointer to the updated lsb
//--------------------------------------------------------------------------------------------------


void updateI2CAddressPattern(uint8 msb, uint8 lsb, uint8 pattern, uint8* out_msb, uint8* out_lsb)
{
    // combine MSB and LSB to one 16-Bit-value
    uint16 combined = ((uint16)msb << 8) | lsb;

    // mask to set bit 3 to 9 to zero, all others keep the same
    uint16 mask = ~(((uint16)0x7F) << 3);

    // shift pattern on position 3-9
    uint16 shiftedPattern = ((uint16)pattern & 0x7F) << 3;

    // Bits 3-9 to zero and insert pattern 
    combined = (combined & mask) | shiftedPattern;

    // MSB and LSB extracted from combined value 
    *out_msb = (combined >> 8) & 0xFF;  // Oberes Byte
    *out_lsb = combined & 0xFF;         // Unteres Byte
}

//--------------------------------------------------------------------------------------------------
// Name:      uint8ToBinaryString
// Note: Pass an big enough string...
//--------------------------------------------------------------------------------------------------

char*  uint8ToBinaryString(uint8 number, char* outStr) {
    for (int i = 0; i < 8; i++) {
		// check each bit from left to right and put it into the string
        outStr[i] = (number & (1 << (7 - i))) ? '1' : '0';
    }
    // add terminator
    outStr[8] = '\0';
	return  outStr;
}


//--------------------------------------------------------------------------------------------------
// Name:      Sdp6xChangeI2CAddress
// Function:  changes the I2C address of a device
//            
// Parameter: currentAddress, newAddress
// Note: Only call this function if only one sensor currently uses this address (i.e. physcically separate all other sensors with this adress from the bus before you call this function)
// refer to following document https://sensirion.com/media/documents/CBEB368E/61FAB1B9/LQ_AN_LiquidFlowSensors_ImplementationGuideToI2C_EN_D1.pdf
// Return:    0 if it failed, > 0 else
//--------------------------------------------------------------------------------------------------
int Sdp6xChangeI2CAddress(uint8 currentAddress, uint8 newAddress)
{
	uint8	buf[4];
	uint8 checksum;
	uint8 original_MSB = 0x00;
	uint8 original_LSB = 0x00;
 	uint8 updated_MSB = 0x00;
	uint8 updated_LSB = 0x00;
	char binaryString_MSB[9];
	char binaryString_LSB[9];
 
 	if ((currentAddress<0x8) ||
		(currentAddress>0x77) ||
		(newAddress<0x8) ||
		(newAddress>0x77) ||
		(currentAddress == newAddress))
	{
		fprintf(stderr, "changeI2cAddress: Error, invaid addresses \n"); 
		return 0;
	}

	//============================== Softreset ==================================================
	
	I2C_Write1(0xFE);			// softreset < 15ms
	DelayMs(2000);

	// === 1. Set the EEPROM’s internal address pointer to address 0x2C2 by sending an I2C write-header followed by the EEPROM access command (0xFA) and the address from which to read. ====
	// === Full command string: 0x 80 FA 2C 20. (hexadecimal,assuming the default I2C address, see section 0) ===

	I2C_Write3(0xFA, 0x2C, 0x20);
	if (I2cError)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		fprintf(stderr, "changeI2cAddress: Error, could not apply step one. Current address=%x, new address=%x\n", currentAddress, newAddress);
		return 0;
	}
	
	// === 2. Read the EEPROM entry (1 word) and the CRC byte by sending an I2C header with R/_W=1.  ===
    I2C_Read(buf, 3); // 0 = MSB, 1 = LSB, 2 = CRC
	if (I2cError)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		fprintf(stderr, "changeI2cAddress: Error, could not apply step two (read of EEPROM). Current address=%x, new address=%x\n", currentAddress, newAddress);
		return 0;
	}
	checksum = CalcSdp6xCrc(buf, 2);
	if (checksum != buf[2])
	{
		g_Sdp6xError |= ERROR_SDP6X_CRC_EEPROM;
		fprintf(stderr, "changeI2cAddress: Error, checcksum incorrect. Received MSB=%x, LSB=%x, checksum=%x, expected checksum:%x\n", buf[0], buf[1], buf[2], checksum);
		return 0;
	}

    // === debug output ===
	if (g_debugLevel>1)
   {
		fprintf(stderr, "changeI2cAddress: Info: current contents of EEPROM: MSB=%x, LSB=%x, checksum=%x, expected checksum:%x\n", buf[0], buf[1], buf[2], checksum);
   }

	// === 3. Save the EEPROM entry to a variable in your program ===
	original_MSB = buf[0];
	original_LSB = buf[1];
	
	// === 4. Define the new EEPROM entry according to the desired setting. (see below) ===
	updateI2CAddressPattern(original_MSB, original_LSB, newAddress, &updated_MSB, &updated_LSB);
	if (g_debugLevel>1)
    {	
		fprintf(stderr, "changeI2cAddress: Info:\n");
		fprintf(stderr, "\toriginal EEPROM Contents:\tMSB=%s\tLSB=%s\n", uint8ToBinaryString(original_MSB, binaryString_MSB), uint8ToBinaryString(original_LSB, binaryString_LSB));
		fprintf(stderr, "\t updated EEPROM Contents:\tMSB=%s\tLSB=%s\n", uint8ToBinaryString(updated_MSB, binaryString_MSB), uint8ToBinaryString(updated_LSB, binaryString_LSB));
    }
	
	// === 5. Reset the EEPROM’s internal address register to address 0x2C2 by repeating step 1 (the internal pointer has been incremented by reading the word). ===
	// === 6. Write the new value to the EEPROM at the same address. (see 5.4 and example 6.7.2) 
	I2C_Write5(0xFA, 0x2C, 0x20, updated_MSB, updated_LSB);
	if (I2cError)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		fprintf(stderr, "changeI2cAddress: Error, could not apply step five and six integrated. Current address=%x, new address=%x\n", currentAddress, newAddress);
		return 0;
	}

	
	// === 7. Wait >10ms for the internal write cycle to complete. ===
	DelayMs(50);
	
	// === 8a. Reread the EEPROM entry by repeating steps 1 and 2 to verify that the new settings have been applied correctly and no communication error has occurred. 
	I2C_Write3(0xFA, 0x2C, 0x20);
	if (I2cError)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		fprintf(stderr, "changeI2cAddress: Error, could not apply step 8a. Current address=%x, new address=%x\n", currentAddress, newAddress);
		return 0;
	}
	
	// === 8b. Read the EEPROM entry (1 word) and the CRC byte by sending an I2C header with R/_W=1.  ===
    I2C_Read(buf, 3); // 0 = MSB, 1 = LSB, 2 = CRC
	if (I2cError)
	{
		g_Sdp6xError |= ERROR_SDP6X_I2C;
		fprintf(stderr, "changeI2cAddress: Error, could not apply step 8b (read of EEPROM). Current address=%x, new address=%x\n", currentAddress, newAddress);
		return 0;
	}
	checksum = CalcSdp6xCrc(buf, 2);
	if (checksum != buf[2])
	{
		g_Sdp6xError |= ERROR_SDP6X_CRC_EEPROM;
		fprintf(stderr, "changeI2cAddress: Error, checcksum incorrect. Received MSB=%x, LSB=%x, checksum=%x, expected checksum:%x\n", buf[0], buf[1], buf[2], checksum);
		return 0;
	}

    // === debug output ===
	if (g_debugLevel>1)
   {
		fprintf(stderr, "changeI2cAddress: Info: current contents of EEPROM (step8b): MSB=%x, LSB=%x, checksum=%x, expected checksum:%x\n", buf[0], buf[1], buf[2], checksum);
   }
	
	// === 9. Soft reset the sensor by sending 0xFE.  ===
	I2C_Write1(0xFE);			// softreset < 15ms
	
	// === 10. Wait for soft reset. (up to 31 ms., see section 4.3) ===
	DelayMs(2000);

	return 1;
} 

   


