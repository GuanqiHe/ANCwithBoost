/*********************************************************************
 *
 * ANSI C Example program:
 *    ContGen-ExtClk.c
 *
 * Example Category:
 *    AO
 *
 * Description:
 *    This example demonstrates how to output a continuous periodic
 *    waveform using an external clock.
 *
 * Instructions for Running:
 *    1. Select the Physical Channel to correspond to where your
 *       signal is output on the DAQ device.
 *    2. Enter the Minimum and Maximum Voltage Ranges.
 *    3. Specify the external sample clock source (typically a PFI or
 *       RTSI pin) in the timing function.
 *
 * Steps:
 *    1. Create a task.
 *    2. Create an Analog Output Voltage Channel.
 *    3. Define the parameters for an External Clock Source.
 *       Additionally, define the sample mode to be Continuous.
 *    4. Write the waveform to the output buffer.
 *    5. Call the Start function.
 *    6. Wait until the user presses the Stop button.
 *    7. Call the Clear Task function to clear the Task.
 *    8. Display an error if any.
 *
 * I/O Connections Overview:
 *    Make sure your signal output terminal matches the Physical
 *    Channel I/O Control. Also, make sure your external clock
 *    terminal matches the Clock Source Control. For further
 *    connection information, refer to your hardware reference manual.
 *
 *********************************************************************/

#include "NIDAQmx.h"
#include <cstdio>
#include <cmath>

#define DAQmxErrChk(functionCall)            \
	if (DAQmxFailed(error = (functionCall))) \
		goto Error;                          \
	else

#define PI 3.1415926535

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

int main(void)
{
	int32 error = 0;
	TaskHandle taskHandle = 0;
	float64 data[8][1000];
	float64 data_init[1000]={0};
	float64 frequency[8] = {220.00, 246.94, 261.64, 293.66, 329.63, 349.23, 392.00, 440.00};
	char errBuff[2048] = {'\0'};
	float64 sampleFs = 100000.0;

	for (int num_freq = 0; num_freq < 8; num_freq++)
	{
		for (int i = 0; i < 1000; i++)
		{
			data[num_freq][i] = 2.0 * sin((double)i * frequency[num_freq] * PI / sampleFs);
		}
	}

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle, "Mod1/ao0", "ContGen-ExtClk-Update", -4.0, 4.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", sampleFs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000));

	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, 1000, 0, 10.0, DAQmx_Val_GroupByChannel, data_init, NULL, NULL));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));



	for (int i = 0; i < 8; i++)
	{
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle, 1000, 0, 10.0, DAQmx_Val_GroupByChannel, data[i], NULL, NULL));
	}

	// printf("Generating voltage continuously. Press Enter to interrupt\n");
	// getchar();

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0)
	{
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
	return 0;
}

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32 error = 0;
	char errBuff[2048] = {'\0'};

	// Check to see if an error stopped the task.
	DAQmxErrChk(status);

Error:
	if (DAQmxFailed(error))
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
	return 0;
}
