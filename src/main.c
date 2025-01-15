// License: This program is published under the GNU GENERAL PUBLIC LICENSE
// Refer to license.txt in this project
// If you did not receive a copy of this license please visit: https://www.gnu.org/licenses/gpl-3.0.txt


//--------------------------------------------------------------------------------------------------

// Filename:           main.c
// Autor:	           Harries
// initial version:    01.08.2024
//
// Description: This program continously reads sensor values of I2C differential pressure sensors and writes them into a file or stdout.
// call program with option -h for more information
//
// The output contains lines with one / more sensor readings seprated by semicolon. Sensor values are in milli Pascal [mPA].
// Lines starting with '***' contain metadata and are not to be considered as sensor values.
// metadata are: Program name, build date, start of measurement, end of measurement, average sampling rate in [Hz/sec], number of measurement values, resolution, sensor I2C addresses
//--------------------------------------------------------------------------------------------------

//=== Includes =====================================================================================

#include "std_c.h"
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include "i2c.h"
#include "sdp6x.h"
#include "raspi.h"
#include <unistd.h>
#include <locale.h>
#include <signal.h>
#include <string.h>
#include <math.h>

#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "stdbool.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


//=== Preprocessing directives (#define) ===========================================================
#define MAX_SENSORS_SUPPORTED 2 // could be extended if needed --> parser needs to be extended
#undef YOU_KNOW_WHAT_YOURE_DOING

//=== Type definitions (typedef) ===================================================================

//=== Global constants =============================================================================

//=== Global variables =============================================================================

//=== Local constants  =============================================================================

static  char STANDARD_OUT[] ="Standard out";

// TODO: general logging to file or std out
static const char *log_file = "/var/log/Sdp6x_daemon.log"; 


//=== Local variables ==============================================================================

static volatile sig_atomic_t keepRunning = 1;
static uint64_t  g_startMeasurement = 0L;
static unsigned long g_number_of_samples_taken = 0L;

//=== Local function prototypes ====================================================================
static int prepareOutFile(char * outputFileName, FILE	**fp, uint8 samplingResolutionNr, uint8* addresses);
static int writePressureValues(FILE *fp, char* deviceListString);
static int closeOutFile(FILE *fp);

//=== helper functions  ============================================================================

// *******************************************************
// * get_posix_clock_time_ms()                           *
// *******************************************************

// get current time in milliseconds
uint64_t get_posix_clock_time_ms ()
{
    struct timespec ts;

    if (clock_gettime (CLOCK_MONOTONIC, &ts) == 0)
        return (uint64_t) (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    else
        return 0;
}

// *******************************************************
// * splitHexString()                                    *
// *******************************************************

// splits string and converts hex values in integers 
int splitHexString(const char *input, uint8 *value1, uint8 *value2) {
    if (!input || !value1 || !value2) {
        return -1; // Error: invalid input 
    }

    // search for separator ':'
    const char *separator = strchr(input, ':');
    if (!separator) {
        return -2; // error: Could not find separator
    }

    // first substring before ':'
    char firstPart[16];
    size_t firstLength = separator - input;
    if (firstLength >= sizeof(firstPart)) {
        return -3; // Error: String too long
    }
    strncpy(firstPart, input, firstLength);
    firstPart[firstLength] = '\0';

    // Second substring (after ':')
    const char *secondPart = separator + 1;

    // Convert hex strings to integer
    *value1 = (uint8)strtol(firstPart, NULL, 16);
    *value2 = (uint8)strtol(secondPart, NULL, 16);

    return 0; // success
}



// *******************************************************
// * log_message()  for daemon mode                      *
// *******************************************************
void log_message(const char *message)
{
    FILE *log_fp = fopen(log_file, "a");
    if (log_fp != NULL) {
        time_t now = time(NULL);
        fprintf(log_fp, "[%s] %s\n", ctime(&now), message);
        fclose(log_fp);
    }
}


// *******************************************************
// * sig_handler(...)                                    *
// *******************************************************
static void sig_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        fprintf(stderr, "Signal (%d) received. Programm will be termiated soon.\n", sig);
		log_message("sig handler received termination signal. Program  will be terminated\n");
        keepRunning = 0;
    }
}

// *******************************************************
// * daemonize()                                         *
// *******************************************************
static void daemonize()
{
	log_message("daemonize starts\n");
    pid_t pid;

    pid = fork();
    if (pid < 0) {
        perror("Error during first fork.\n");
        exit(EXIT_FAILURE);
    }
    if (pid > 0) {
        // Elternprozess beenden
        exit(EXIT_SUCCESS);
    }

    // Session-ID ceation
    if (setsid() < 0) {
        perror("Error during setsid");
        exit(EXIT_FAILURE);
    }

    // second fork: Avoids that daemon inherits a terminal
    pid = fork();
    if (pid < 0) {
        perror("Error during second fork");
        exit(EXIT_FAILURE);
    }
    if (pid > 0) {
        // terminate parent process
        exit(EXIT_SUCCESS);
    }

    // change working directory to root directory
    if (chdir("/") < 0) {
        perror("Error during change to root directory\n");
        exit(EXIT_FAILURE);
    }

    // sets mask for files - all will be able to read
    umask(0);

    // close file handles
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);

    // re-direct von std in-out to /dev/null
    open("/dev/null", O_RDONLY);  // stdin
    open("/dev/null", O_WRONLY);  // stdout
    open("/dev/null", O_WRONLY);  // stderr
}

// *******************************************************
// * main                                                *
// *******************************************************

int main(int argc, char **argv)
{ 
	int8 res = 0;
	int16 pressureInMilliPascal;
	uint8 HwRev;
	int delayBtwSamplesMs = 0;	
	uint8 samplingResolutionNr=Sdp6xGetNumberOfResolutions();
	FILE *fp;
	char *outputFilename = NULL;
	
	bool o_help = false;
	bool o_daemon_mode = false;
	bool o_changeI2CAddress = false;
	
	uint8 currentAddress = 0x40;
	uint8 newAddress = 0x00;
	uint8 I2CAdresses[MAX_SENSORS_SUPPORTED]={0x40,0x00};

	char deviceListString[4096];
	char singleAddressString[16];

	// Signalhandler register signal handler to avoid broken I2C communication
	signal(SIGINT, sig_handler);   
    signal(SIGTERM, sig_handler);  

	HwRev = GetRaspberryHwRevision();

	// --------------------------------------------------------------------------------------------------------------------
	// Parse command line
	// --------------------------------------------------------------------------------------------------------------------
	for (int i = 1; i < argc;i++)
	{
		if (strcmp(argv[i], "-h") == 0)
		{
			o_help = true;
			break;
		}

		if (strncmp(argv[i], "-delay:", 7) == 0)
		{
			delayBtwSamplesMs = atoi(argv[i] + 7);
			if (delayBtwSamplesMs <= 0)
			{
					fprintf(stderr, "Error: Invalid delay value '%s'.\n", argv[i] + 7);
					delayBtwSamplesMs = 0;
			}
			continue;
		}
		
		if (strncmp(argv[i], "-sample-resolution:", 19) == 0)
		{
			samplingResolutionNr=Sdp6xGetNumberOfResolutions();
			for (int x=0;x<Sdp6xGetNumberOfResolutions();x++)
			{
				if (strcmp(argv[i]+19, Sdp6xGetResolution(x)) == 0)
				{
					samplingResolutionNr = x;
					break;
				}
			}
			continue;
		}

		if (strcmp(argv[i], "-daemon") == 0)
		{
				o_daemon_mode = true;
				continue;
		}

		if (strncmp(argv[i], "-changeI2CAddress:", 18) == 0)
		{
			int result = splitHexString(argv[i]+18, &currentAddress, &newAddress);
			if ((currentAddress<0x8) ||
				(currentAddress>0x77) ||
				(newAddress<0x8) ||
				(newAddress>0x77) ||
				(currentAddress == newAddress))
			{
				fprintf(stderr, "Error: Invalid addresses given for changeAddress. Result=%d, parsed commandlineString=%s, CurrentAddress=0x%x, newAddress=0x%x\n", result, argv[i]+18, currentAddress, newAddress);
				exit(1);	
			}
			o_changeI2CAddress = true;
			break;
		}

		if (strncmp(argv[i], "-I2CAdr:", 8) == 0)
		{
			int result = splitHexString(argv[i]+8, &I2CAdresses[0], &I2CAdresses[1]);
			
			if ((I2CAdresses[0]<0x8) ||
				(I2CAdresses[0]>0x77) ||
				((I2CAdresses[1]<0x8) && (I2CAdresses[1]!=0x00)) ||
				(I2CAdresses[1]>0x77) ||
				(I2CAdresses[0] == I2CAdresses[1]))
			{
				fprintf(stderr, "Error: Invalid addresses given for I2CAdresses, Result=%d, parsed commandlineString=%s, Address Sensor 1=0x%x, Address Sensor 2=0x%x -> will set default values\n", result, argv[i]+8, I2CAdresses[0], I2CAdresses[1]);
				I2CAdresses[0]=0x40;
				I2CAdresses[1]=0x00;
			}
			continue;
		}

		// Assume it is the output filename if it does not match any known option
		// keep this always as last rule to match!
		if ((outputFilename == NULL) && (argv[i]!= NULL))
		{
			outputFilename = argv[i];
			continue;
		}
	}
		
	if(o_help == true) 
	{
		printf("Pressure sensor read out program  build date: [" __DATE__ " " __TIME__"]\n");
		printf("*********************************************************************\n");
		printf("RaspberryHwRevision=%i\r\n",HwRev);
		printf("Known issue: if I2C communication breaks down the sensor may no react even on softreset, try running once or more i2cdump -y 1 <address>\n");

		printf("\nUsage:%s [options] [outputfilename]\n",argv[0]);
		printf("outputfilename is optional, if omitted, stdout will be used\n");
		printf("Possible options are:\n");
		printf("-h\n\t\tPrints this help\n");
		printf("-delay:<x>\n\t\tDelays sampling by x ms - note: no spaces in option!\n");
		printf("-daemon\n\t\tStart program as daemon. Note that you need to have root rights for this. Terminate it by the following commands: ps aux | grep %s and kill -15 <pid>\n", argv[0]);
		printf("-sample-resolution:<x>\n\t\tSpecifies sampling resolution in bits. Sample rate is influcenced by resolution. Possible values are from 9 to 16. Note: no spaces in option!\n");
		printf("-changeI2CAddress:<currentAddr>:<newAddress> \n\t\tUse this parameter to change the I2C Address from a specific sensor. Make sure that only one sensor is attached to the bus with the same currentAddress. Note this call may destroy a sensor by overwriting calibration data. Do not use on incompatible device, no warranty / liability.\n");
		printf("-I2CAdr:<d1>:<d2>\n\t\tDefines the device address of the sensors. Use hex values, e.g. 0x40:0x41. At maximum two sensors are supported. If you have only one sensor write e.g. 0x40:0x00. If the parameter is omitted 0x40 will be used\n");
		printf("\nExample %s -delay:0 -sample-resolution:16 -daemon outputfile\n", argv[0]);
		exit (0);
	}	

	if (o_changeI2CAddress == true)
	{
		#ifdef YOU_KNOW_WHAT_YOURE_DOING
			if(HwRev < 2) 	I2C_Open("/dev/i2c-0");	 // Hardware Revision 1.0
				else		I2C_Open("/dev/i2c-1");  // Hardware Revision 2.0
			I2C_Setup(I2C_SLAVE, I2CAdresses[0]); 
			
			if(I2cError)
			{	
				I2C_PrintError();
				exit(1);
			}
			fprintf(stderr, "Main: I2C Initialisation ok\n");
			int result = Sdp6xChangeI2CAddress(currentAddress, newAddress);
			I2C_Close();

			if (result >0)
			{
				printf("Ic2Address sucessfully changed.\n");
				exit (0);
			}
			else
			{
				printf("Ic2Address not sucessfully changed.\n");
				exit(1);
			}
		#else
			printf("The option to change I2CAddress is disabled by default. Re-compile with enabled section if you know what you're doing.\n");
			exit(1);
		#endif
	}

	printf("Program started. Call with -h for further information\n");

	// avoid compiler warning
	char *displayName;
	if (outputFilename == NULL)
	{
		displayName = STANDARD_OUT;
	}
	else
	{
		displayName = outputFilename;
	}

	printf("Started %s with delayBtwSamplesMs=%d, Resolution=%s and outfile=%s, daemon mode=%d, I2CAddress1=0x%x, I2CAddress2=0x%x\n", argv[0], delayBtwSamplesMs, Sdp6xGetResolution(samplingResolutionNr), displayName, o_daemon_mode, I2CAdresses[0], I2CAdresses[1]);

	// Check for daemon option
	if (o_daemon_mode == true)
	{
		log_message("Start as daemon...\n");
        daemonize();
		log_message("Daemon start sucessfull.\n");
    } else
	{
        printf("Start as normal program ...\n");
    }

	if(HwRev < 2) 	I2C_Open("/dev/i2c-0");	 // Hardware Revision 1.0
		else		I2C_Open("/dev/i2c-1");  // Hardware Revision 2.0
	
	// inherit locale from environment
    setlocale(LC_NUMERIC, "");
	

	// --------------------------------------------------------------------------------------------------------------------
	// setup sensors
	// --------------------------------------------------------------------------------------------------------------------

    for (int s=0; s<MAX_SENSORS_SUPPORTED;s++)
	{
		if (I2CAdresses[s]==0x00)
		{
			break;
		}
		int tryInit = Sdp6xSensorInit(I2CAdresses[s], samplingResolutionNr);
	    int tryStart = Sdp6xSetupMeasurement(I2CAdresses[s], SENSOR_DEVICE_STATE_CONTINOUS_READ);
        if ((tryInit == 0) || (tryStart != 0))
		{
			fprintf(stderr, "Main: Error during sensor initalization for address: 0x%x\n", I2CAdresses[s]);		
			exit(1);
		}
	}

	// give the sensors time before we start first reading
	Sdp6xDelayByMeasurementTime();
	
	prepareOutFile(outputFilename,&fp, samplingResolutionNr, I2CAdresses);
    	
	// --------------------------------------------------------------------------------------------------------------------
	// main task
	// --------------------------------------------------------------------------------------------------------------------
	log_message("Initialization sucessful. Started endless measurement loop. Stop daemon by sending signal 15 to the process.\n");
	

	while(keepRunning)
	{
        strcpy(deviceListString,"");
		for (int s=0; s<MAX_SENSORS_SUPPORTED;s++)
		{
			if (I2CAdresses[s]==0x00)
			{
				break;
			}
			if (Sdp6xRead(I2CAdresses[s], &pressureInMilliPascal) !=0)
			{
				if (s==0)
				{
					sprintf(singleAddressString, "1"); 
				}
				else
				{
					sprintf(singleAddressString, ";1");
				}
				Sdp6xPrintError();
				I2cError = 0;
			}
			else
			{
				if (s==0)
				{
					sprintf(singleAddressString, "%d", pressureInMilliPascal);
				}
				else
				{
					sprintf(singleAddressString, ";%d", pressureInMilliPascal);
				}
			}
			strcat(deviceListString, singleAddressString);
		}

		Sdp6xDelayByMeasurementTime();
		
		res = writePressureValues(fp, deviceListString);
		if (res < 1)
		{			
			fprintf( stderr, "Main: Fehler beim Schreiben in Datei (%d)\n",res);
		}
		 
		if (delayBtwSamplesMs > 0)
		{
			DelayMs(delayBtwSamplesMs);
		}
	}
	
	// --------------------------------------------------------------------------------------------------------------------
	// tear down
	// --------------------------------------------------------------------------------------------------------------------

 	for (int s=0; s<MAX_SENSORS_SUPPORTED;s++)
	{
		if (I2CAdresses[s]==0x00)
		{
			continue;
		}
		
	    int tryStop = Sdp6xSetupMeasurement(I2CAdresses[s], SENSOR_DEVICE_STATE_IDLE);
        if ((tryStop != 0))
		{
			fprintf(stderr, "Main: Error during sensor-stop-measurement for address: 0x%x\n", I2CAdresses[s]);		
		}
	}
    
	I2C_Close();
	closeOutFile(fp);
	log_message("Teardown of measurement finished.\n");
	return(0);
}

// *******************************************************
// *  prepareOutFile                                     *
// *******************************************************
static int prepareOutFile(char * outputFileName, FILE	**fp, uint8 samplingResolutionNr, uint8* addresses)
{
	char deviceListString[4096];
	char singleAddressString[6];
	time_t TimeCounter;
	time_t TimeCounterLocal;
	struct tm * Time;

	time ( &TimeCounter );
	Time = localtime (&TimeCounter);   // time zone handling
	TimeCounterLocal = TimeCounter - timezone;			
	if(Time->tm_isdst) TimeCounterLocal += 3600;	// Sommer time
    if (outputFileName == NULL)
	{
		*fp = stdout;
		outputFileName = STANDARD_OUT;
	}
	else
	{
		*fp = fopen(outputFileName,"w");
		// enable buffering to minimize impact on storage card
		setvbuf(*fp, NULL, _IOFBF, 4096); // @TODO: Disable for writing e.g. to memory FIFO if you want to have real time sensor output
	}
    
	g_startMeasurement = get_posix_clock_time_ms ();

    // list active devices
    strcpy(deviceListString,"");
	for (int s=0; s<MAX_SENSORS_SUPPORTED;s++)
	{
		if (addresses[s]==0x00)
		{
			break;
		}
		if (s==0)
		{
			sprintf(singleAddressString, "0x%x", addresses[s]);
		}
		else
		{
			sprintf(singleAddressString, ";0x%x", addresses[s]);
		}
		strcat(deviceListString, singleAddressString);
	}
	
	if (*fp != NULL)
	{
		fprintf(*fp,"*** Pressure sensor read out program  build date: [" __DATE__ " " __TIME__"]\n");
		fprintf(*fp,"*** output to file: %s, local time: %02d.%02d.%d;%02d:%02d.%02d, values in milliPascal [mP] ***\n", outputFileName, Time->tm_mday,Time->tm_mon+1,Time->tm_year+1900,Time->tm_hour, Time->tm_min, Time->tm_sec);
		fprintf(*fp,"*** Sampling resolution: %s bit\n", Sdp6xGetResolution(samplingResolutionNr));
		fprintf(*fp,"*** Output contains following I2CAddresses: %s\n", deviceListString);
		return 1;
	}
	else
	{
		fprintf(stderr,"Error during creation of output file\n");
		return -1;
	}
}

// *******************************************************
// *  writePressureValues                                *
// *******************************************************
static int writePressureValues(FILE *fp, char* deviceListString)
{

	g_number_of_samples_taken++;

	if (fp != NULL)
	{
	    fprintf(fp, "%s\n", deviceListString);
		return 1;
	}
	else
	{
		return -1;
	}
}


// *******************************************************
// *  closeOutFile                                       *
// *******************************************************
static int closeOutFile(FILE *fp)
{
	time_t TimeCounter;
	time_t TimeCounterLocal;
	struct tm * Time;

	time ( &TimeCounter );
	Time = localtime (&TimeCounter); 
	TimeCounterLocal = TimeCounter - timezone;			
	if(Time->tm_isdst) TimeCounterLocal += 3600;
    
	if (fp != NULL)
	{
		uint64_t time_value;
		uint64_t time_diff;
		
		time_value = get_posix_clock_time_ms ();
		time_diff = time_value - g_startMeasurement;
		
		double mediumSampleFreq = (double) ((g_number_of_samples_taken * 1000)/time_diff);

		fprintf(fp,"*** Output format: ###;sample median sample frequency;number of samples;duration of sampling in seconds\n");
		fprintf(fp,"*** ###;%f;%lu;%f\n", mediumSampleFreq, g_number_of_samples_taken,(double)(time_diff/1000));
		
        fprintf(fp,"*** output ends, Measurement stops at local time: %02d.%02d.%d;%02d:%02d.%02d ***\n", Time->tm_mday,Time->tm_mon+1,Time->tm_year+1900,Time->tm_hour, Time->tm_min, Time->tm_sec);
		fflush(fp); 
		fclose(fp);

		return 1;
	}
	else
	{
		return -1;
	}

}



 