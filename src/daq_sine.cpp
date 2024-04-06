#include <cstdio>
#include <cstring>
#include <ctime>
#include <cmath>
#include <iostream>
#include <fstream>
#include <chrono>

#include "NIDAQmx.h"
#include "yaml.h"
#include "msgpack.hpp"

#define DAQmxErrChk(functionCall)            \
	if (DAQmxFailed(error = (functionCall))) \
		goto Error;                          \
	else

typedef std::vector<double> array_t;

struct datapack_t
{
	array_t t;
	array_t y;
	array_t d;
	array_t u;
	MSGPACK_DEFINE_MAP(t, y, d, u);
};

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

static int32 GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);

void (*controlTaskRun)(const float64 &y, float64 &u) = NULL;

int main(int argc, char *argv[])
{
	std::string config_path = "config.yaml";
	if (argc != 1)
	{
		config_path = std::string(argv[1]);
	}

	// read parameter from config file
	YAML::Node config = YAML::LoadFile(config_path);
	const float64 sampleFs = config["sample_fs"].as<float64>();
	const float64 dist_freq = config["dist_freq"].as<float64>();
	const float64 dist_gain = config["dist_gain"].as<float64>();
	const float64 runTime = config["run_time"].as<float64>();	  // sec
	const float64 warmUp = config["warm_up"].as<float64>();		  // sec
	const float64 startWait = config["start_wait"].as<float64>(); // sec
	const float64 analog_read_bias = config["analog_read_bias"].as<float64>();
	const float64 analog_read_gain = config["analog_read_gain"].as<float64>();
	// const std::string log_path =  config["log_path"].as<std::string>();

	printf("config file path: %s \n", config_path.c_str());
	// printf("log file path: %s \n", log_path.c_str());

	int32 error = 0;
	char errBuff[2048] = {'\0'};
	TaskHandle taskHandle_w = 0, taskHandle_r = 0;
	char trigName[256] = {0};

	const int32 samplesPerChan = 1;

	const int32 writeNumChan = 2;
	float64 write_origin[samplesPerChan * writeNumChan + 100] = {0};
	float64 *writeChan0 = &write_origin[0], *writeChan1 = &write_origin[samplesPerChan];

	const int32 readNumChan = 1;
	float64 read_origin[samplesPerChan * readNumChan] = {0};
	float64 *readChan0 = &read_origin[0];

	const int32 totalNumSamples = (runTime + warmUp) * sampleFs + sampleFs;

	std::cout << "run time: " << runTime << " "
			  << "total data points: " << totalNumSamples << " "
			  << "Hz: " << sampleFs / samplesPerChan << std::endl;

	int64 index = 0;
	float64 globalTime = 0;
	float64 timeout = 10.0 / sampleFs;
	CVIAbsoluteTime t;

	std::ofstream stream(config["device_log_path"].as<std::string>(), std::ios::binary);
	msgpack::packer<std::ofstream> packer(stream);
	datapack_t data;
	data.t.reserve(totalNumSamples);
	data.d.reserve(totalNumSamples);
	data.u.reserve(totalNumSamples);
	data.y.reserve(totalNumSamples);

	std::chrono::_V2::system_clock::time_point start, end;

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("write", &taskHandle_w));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle_w, "Mod1/ao0:1", "", -4.0, 4.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle_w, "OnboardClock", sampleFs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle_w, 0, DoneCallback, NULL));

	DAQmxErrChk(DAQmxCreateTask("read", &taskHandle_r));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle_r, "Mod2/ai0", "", DAQmx_Val_PseudoDiff, -4.0, 4.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle_r, "OnboardClock", sampleFs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle_r, 0, DoneCallback, NULL));

	t.cviTime.lsb = 0;
	t.cviTime.msb = time(NULL) + int(startWait) + 2082844800;
	DAQmxCfgTimeStartTrig(taskHandle_w, t, DAQmx_Val_HostTime);
	DAQmxCfgTimeStartTrig(taskHandle_r, t, DAQmx_Val_HostTime);

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan + 50, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
	// DAQmxErrChk(DAQmxWriteAnalogScalarF64(taskHandle_w, 0, 10.0, 0.0, NULL));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle_w));
	DAQmxErrChk(DAQmxStartTask(taskHandle_r));

	DAQmxErrChk(DAQmxReadAnalogScalarF64(taskHandle_r, 10.0, &readChan0[0], NULL));

	start = std::chrono::system_clock::now();

	printf("start warm up\n");
	for (int32 i = 0; i < warmUp * sampleFs; i++)
	{

		float64 d = dist_gain * sin(globalTime * dist_freq * 2 * M_PI) + 0.3 * dist_gain * sin(globalTime * 2 * dist_freq * 2 * M_PI);
		float64 u = 0;
		float64 y = readChan0[0] * analog_read_gain + analog_read_bias;

		writeChan0[0] = d;
		writeChan1[0] = u;

		index += 1;
		globalTime += 1 / sampleFs;

		data.t.push_back(globalTime);
		data.d.push_back(d);
		data.y.push_back(y);
		data.u.push_back(u);

		// DAQmxErrChk(DAQmxWriteAnalogScalarF64(taskHandle_w, 0, timeout, writeChan0[0], NULL));
		DAQmxErrChk(DAQmxReadAnalogScalarF64(taskHandle_r, timeout, &readChan0[0], NULL));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, timeout, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		// DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 1e-3, DAQmx_Val_GroupByChannel, read_origin, samplesPerChan * readNumChan, &samplesPerChanWrite, NULL));
		if (i >= warmUp * sampleFs)
		{
			break;
		}
	}
	end = std::chrono::system_clock::now();
	std::cout << "Warm up time: " << double(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << std::endl;

	start = std::chrono::system_clock::now();

	printf("start run main loop\n");
	for (int i = 0; i < runTime * sampleFs; i++)
	{

		float64 d = dist_gain * sin(globalTime * dist_freq * 2 * M_PI) + 0.3 * dist_gain * sin(globalTime * 2 * dist_freq * 2 * M_PI);
		float64 y = readChan0[0] * analog_read_gain + analog_read_bias;
		float64 u = 0;

		u = std::min(std::max(u, -4.0), 4.0);

		writeChan0[0] = d;
		writeChan1[0] = u;

		index += 1;
		globalTime += 1 / sampleFs;

		data.t.push_back(globalTime);
		data.d.push_back(d);
		data.y.push_back(y);
		data.u.push_back(u);

		// DAQmxErrChk(DAQmxWriteAnalogScalarF64(taskHandle_w, 0, timeout, writeChan0[0], NULL));
		DAQmxErrChk(DAQmxReadAnalogScalarF64(taskHandle_r, timeout, &readChan0[0], NULL));
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, timeout, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		// DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 1e-3, DAQmx_Val_GroupByChannel, read_origin, samplesPerChan * readNumChan, &samplesPerChanWrite, NULL));

		if (i >= runTime * sampleFs)
		{
			break;
		}
	}
	end = std::chrono::system_clock::now();
	std::cout << "Run time: " << double(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << std::endl;

	// printf("Generating voltage continuously. Press Enter to interrupt\n");
	// getchar();

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle_w != 0 || taskHandle_r != 0)
	{
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle_w);
		DAQmxClearTask(taskHandle_w);
		DAQmxStopTask(taskHandle_r);
		DAQmxClearTask(taskHandle_r);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program\n");
	packer.pack(data);
	stream.close();

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

static int32 GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[])
{
	int32 error = 0;
	char device[256];
	int32 productCategory;
	uInt32 numDevices, i = 1;

	DAQmxErrChk(DAQmxGetTaskNumDevices(taskHandle, &numDevices));
	while (i <= numDevices)
	{
		DAQmxErrChk(DAQmxGetNthTaskDevice(taskHandle, i++, device, 256));
		DAQmxErrChk(DAQmxGetDevProductCategory(device, &productCategory));
		if (productCategory != DAQmx_Val_CSeriesModule && productCategory != DAQmx_Val_SCXIModule)
		{
			*triggerName++ = '/';
			strcat(strcat(strcpy(triggerName, device), "/"), terminalName);
			break;
		}
	}

Error:
	return error;
}