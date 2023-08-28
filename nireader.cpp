/*********************************************************************
*
* Description:
*    This example demonstrates how to Acquire a Finite amount of data
*    using an external clock.
*
* Instructions for Running:
*    1. Select the Physical Channel to correspond to where your
*       signal is input on the DAQ device.
*    2. Enter the Minimum and Maximum Voltage Ranges.
*    Note: For better accuracy try to match the Input Ranges to the
*          expected voltage level of the measured signal.
*    3. Select the Source of the Clock for the acquisition.
*    4. Select how many Samples to Acquire on Each Channel.
*    5. Set the approximate Rate of the external clock. This allows
*       the internal characteristics of the acquisition to be as
*       efficient as possible.
*    Note: The Rate should be AT LEAST twice as fast as the maximum
*          frequency component of the signal being acquired.
*
* Steps:
*    1. Create a task.
*    2. Create an Analog Input Voltage channel.
*    3. Define the parameters for an External Clock Source.
*       Additionally, define the sample mode to be finite.
*    4. Call the Start function to begin the acquisition.
*    5. Use the Read function to Measure Multiple Samples from N
*       Channels on the Data Acquisition Card. Set a timeout so an
*       error is returned if the samples are not returned in the
*       specified time limit.
*    6. Call the Clear Task function to clear the task.
*    7. Display an error if any.
*
* I/O Connections Overview:
*    Make sure your signal input terminal matches the Physical
*    Channel I/O Control. Also, make sure your external clock
*    terminal matches the Clock Source Control. For further
*    connection information, refer to your hardware reference manual.
*
*********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <NIDAQmx.h>
#include <iostream>
#include <windows.h>
#include <pthread.h>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define DATA_SAMPLE_SIZE 1
#define FREQACQUISITION 10.0

int32       error=0;
TaskHandle  taskHandle=0;
int32       read;
float64     niData[DATA_SAMPLE_SIZE];
char        errBuff[2048]={'\0'};
pthread_t   acquisitionThread;

using namespace std;

void readSensor()
{
    //int32       error=0;
    //TaskHandle  taskHandle=0;
    //int32       read;
    //float64     data[DATA_SAMPLE_SIZE];
    //char        errBuff[2048]={'\0'};

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
    DAQmxErrChk (DAQmxCreateAIVoltageChan(taskHandle,"Dev1/ai0","",DAQmx_Val_Cfg_Default,-5.0,5.0,DAQmx_Val_Volts,NULL));
    DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandle,"",FREQACQUISITION,DAQmx_Val_Rising,DAQmx_Val_ContSamps,DATA_SAMPLE_SIZE));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk (DAQmxStartTask(taskHandle));
        /*********************************************/
    while (true) {
        /*********************************************/
        // DAQmx Read Code
        /*********************************************/
        DAQmxErrChk (DAQmxReadAnalogF64(taskHandle,-1,10.0,DAQmx_Val_GroupByChannel,niData,DATA_SAMPLE_SIZE,&read,NULL));
//        if( read>0 )
//            for (int i=0; i <DATA_SAMPLE_SIZE; i++){
//                cout << niData[i] << endl;
//            }
}

Error:
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( taskHandle!=0 ) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
    return;
}

void* AcquiThreadFunc(void* arg)
{
    readSensor();
    return NULL;
}

void StartAcquisition()
{
   // pthread_t thread_id;
    int result = pthread_create(&acquisitionThread, NULL, &AcquiThreadFunc , NULL);
}

void StopAcquisition()
{
    pthread_t thread_id;
    pthread_join(acquisitionThread, NULL);
    exit(0);
}

