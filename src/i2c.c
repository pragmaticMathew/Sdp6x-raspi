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
// Filename:   	i2c.c 
// Description: Functions for interfaceing I2C on Raspberry Pi
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
// der GNU General Public License, wie von der Free Software Foundation,
// Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
// veröffentlichten Version, weiterverbreiten und/oder modifizieren.
//
// Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
// OHNE JEDE GEWAHRLEISTUNG, bereitgestellt; sogar ohne die implizite
// Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
// Siehe die GNU General Public License für weitere Details.i2c.c.org/licenses/>.
//
// Author:      Martin Steppuhn
// History:     19.11.2012 Initial versionif (logLevel>0)
		
		
//--------------------------------------------------------------------------------------------------
// Erweiterungen des Loggings um Detailausgaben
// Author:		Harries
// History:		01.08.2024 Abspaltung von der Originalversion von Martin Seppuhn
// -------------------------------------------------------------------------------------------------



//static uint8_t i2c_address = 0;
 //if (i2c_address != address) {
//        ioctl(i2c_device, I2C_SLAVE, address);



//=== Includes =====================================================================================

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include "std_c.h"
#include "i2c.h"
#include "time.h"



//=== Preprocessing directives (#define) ===========================================================

//=== Type definitions (typedef) ===================================================================

//=== Global constants =============================================================================

//=== Global variables =============================================================================

uint8	I2cError;
uint8 logLevel = 0; // 0 = original logging, 1 = extended error logging, 2 = detailed logging

//=== Local constants  =============================================================================

//=== Local variables ==============================================================================

int		I2cDevHandle;

//=== Local function prototypes ====================================================================

//--------------------------------------------------------------------------------------------------
// Name:	  I2C_Open 
// Function:  Open device/port
//			  Raspberry Hardwarerevision 1.0	P1 = /dev/i2c-0
//			  Raspberry Hardwarerevision 2.0	P1 = /dev/i2c-1
//            
// Parameter: Devicename (String)
// Return:    
//--------------------------------------------------------------------------------------------------
void I2C_Open(char *dev)
{
	I2cError = 0;

	if ((I2cDevHandle = open(dev, O_RDWR)) < 0)
	{
		I2cError |= ERROR_I2C_OPEN;
		I2C_PrintError();
	}
	if (logLevel>1)
	{
		fprintf(stderr,"### i2copen: dev handle: %x\n", I2cDevHandle);
	}
}	

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Close
// Function:  Close port/device
//            
// Parameter: -
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Close(void)
{
	close(I2cDevHandle);
}	

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Setup
// Function:  Setup port for communication
//            
// Parameter: mode (typical "I2C_SLAVE"), Device address (typical slave address from device) 
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Setup(uint32 mode,uint8 addr)	
{	
	if(!I2cError)
	{
		if (ioctl(I2cDevHandle, mode,addr) < 0)
		{
			I2cError |= ERROR_I2C_SETUP;	
			I2C_PrintError();
		}
					
	}						
}	

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Write1
// Function:  Write a singel byte to I2C-Bus
//            
// Parameter: Byte to send
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Write1(uint8 d)
{
	
	if(!I2cError)
	{
		if((write(I2cDevHandle, &d, 1)) != 1)
		{
			I2cError |= ERROR_I2C_WRITE;
			I2C_PrintError();
		}
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Write1: Vor dem Schreiben von %x ist ein Fehler aufgetreten\n", d);	
		}
	}
	
}

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Write2
// Function:  Write two bytes to I2C
//            
// Parameter: First byte, second byte
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Write2(uint8 d0,uint8 d1)
{
	uint8 buf[2];
	
	if(!I2cError)
	{
		buf[0]=d0;
		buf[1]=d1;
		if ((write(I2cDevHandle, buf,2)) != 2) I2cError |= ERROR_I2C_WRITE;		
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Write2: Vor dem Schreiben von %x %x ist ein Fehler aufgetreten\n", d0, d1);	
		}
	}
}
//--------------------------------------------------------------------------------------------------
// Name:      I2C_Write3
// Function:  Write three bytes to I2C
//            
// Parameter: First byte, second byte, thrid
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Write3(uint8 d0,uint8 d1, uint8 d2)
{
	uint8 buf[3];
	
	if(!I2cError)
	{
		buf[0]=d0;
		buf[1]=d1;
		buf[2]=d2;
		if ((write(I2cDevHandle, buf,3)) != 3) I2cError |= ERROR_I2C_WRITE;		
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Write3: Vor dem Schreiben von %x %x %x ist ein Fehler aufgetreten\n", d0, d1, d2);	
		}
	}
}

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Write4
// Function:  Write four bytes to I2C
//            
// Parameter: First byte, second byte, thrid, fourth
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Write4(uint8 d0,uint8 d1, uint8 d2, uint8 d3)
{
	uint8 buf[4];
	
	if(!I2cError)
	{
		buf[0]=d0;
		buf[1]=d1;
		buf[2]=d2;
		buf[3]=d3;
		if ((write(I2cDevHandle, buf,4)) != 4) I2cError |= ERROR_I2C_WRITE;		
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Write4: Vor dem Schreiben von %x %x %x %x ist ein Fehler aufgetreten\n", d0, d1, d2, d3);	
		}
	}
}

//--------------------------------------------------------------------------------------------------
// Name:      I2C_Write5
// Function:  Write five bytes to I2C
//            
// Parameter: First byte, second byte, thrid, fourth, fifth
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Write5(uint8 d0,uint8 d1, uint8 d2, uint8 d3, uint8 d4)
{
	uint8 buf[5];
	
	if(!I2cError)
	{
		buf[0]=d0;
		buf[1]=d1;
		buf[2]=d2;
		buf[3]=d3;
		buf[4]=d4;
		if ((write(I2cDevHandle, buf,5)) != 5) I2cError |= ERROR_I2C_WRITE;		
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Write5: Vor dem Schreiben von %x %x %x %x %x ist ein Fehler aufgetreten\n", d0, d1, d2, d3, d4);	
		}
	}
}



//--------------------------------------------------------------------------------------------------
// Name:      I2C_Read
// Function:  Read a number of bytes
//            
// Parameter: Pointer to buffer, Number of bytes to read
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_Read(uint8 *data,uint8 length)
{
	int gelesen=0;
	if(!I2cError)
	{
		gelesen = read(I2cDevHandle, data,length);
		if (gelesen != length)
		{
			I2cError |= ERROR_I2C_READ;		
			if (logLevel>0)
			{
				fprintf(stderr,"### I2C_Read: Etwas stimmt mit der Länge nicht, ich wollte %d bytes  lesen, habe aber nur %d bytes bekommen\n",length,gelesen);
			}
		}
		else
		{
			if (logLevel>1)
			{
				fprintf(stderr,"### I2C_Read: Konnte %d bytes lesen\n",gelesen);
			}
		}
	}
	else
	{
		if (logLevel>0)
		{
			fprintf(stderr,"### I2C_Read: Vor dem Lesen gab es ein Problem\n");
		}
	}
}

//--------------------------------------------------------------------------------------------------
// Name:      I2C_PrintError
// Function:  Print error flags as readable text.
//            
// Parameter: -
// Return:    -
//--------------------------------------------------------------------------------------------------
void I2C_PrintError(void)
{	
	if(I2cError & ERROR_I2C_OPEN)	fprintf(stderr,"Failed to open I2C-Port\r\n");
	if(I2cError & ERROR_I2C_SETUP)	fprintf(stderr,"Failed to setup I2C-Port\r\n");
	if(I2cError & ERROR_I2C_READ)	fprintf(stderr,"I2C read error\r\n");
	if(I2cError & ERROR_I2C_WRITE)	fprintf(stderr,"I2C write error\r\n");
}
