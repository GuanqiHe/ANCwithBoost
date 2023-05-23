#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>

#include "NIDAQmx.h"
#include "msgpack.hpp"

#include "controller/default_controller.hpp"
#include "logger/data_logging.hpp"

#define _USE_MATH_DEFINES 


void (*controlTaskRun)(const float64 &y, float64 &u) = NULL;

float64 input2dB(float64 data) { return 20 * log10f64(data * 1000 / 50 / (2 * 1e-5)); }

void controlTaskInit()
{
}

int main(void)
{
	int32 error = 0;
	char errBuff[2048] = {'\0'};
	TaskHandle taskHandle_w = 0, taskHandle_r = 0;

	const int32 samplesPerChan = 1000;
	const float64 sampleFs = 25000.0, dist_freq = 35.0;

	const int32 writeNumChan = 2;
	float64 write_origin[samplesPerChan * writeNumChan] = {0};
	float64 *writeChan0 = &write_origin[0], *writeChan1 = &write_origin[samplesPerChan];

	const int32 readNumChan = 1;
	float64 read_origin[samplesPerChan * readNumChan] = {0};
	float64 *readChan0 = &read_origin[0];

	const float runTime = 10, warmUp = 1.0; // sec
	const int32 totalNumSamples = (runTime + warmUp) * sampleFs + sampleFs;
	std::cout << "run time: " << runTime << " "
			  << "total data points: " << totalNumSamples << std::endl;

	std::ofstream stream("distRejTemp.bin", std::ios::binary);
	msgpack::packer<std::ofstream> packer(stream);
	datapack_t data;
	data.d.reserve(totalNumSamples);
	data.t.reserve(totalNumSamples);
	data.u.reserve(totalNumSamples);
	data.y.reserve(totalNumSamples);

	int64 index = 0;
	float64 globalTime = 0;

	DefaultController controller(dist_freq * M_2_PI, -15.0, 0.0, 1.0 / sampleFs);


	for (int i = 0; i < warmUp * sampleFs / samplesPerChan; i++)
	{

		for (int i = 0; i < samplesPerChan; i++)
		{

			float64 d = 2.0 * sin(globalTime * dist_freq * M_2_PI);
			float64 u = 0;
			float64 y = (read_origin[i] - 0.00025) * 8000;

			data.t.push_back(globalTime);
			data.d.push_back(d);
			data.u.push_back(u);
			data.y.push_back(y);

			writeChan0[i] = d;
			writeChan1[i] = u;

			index += 1;
			globalTime += 1 / sampleFs;
		}

        //set write_origin, read_origin
		// DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		// DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 10.0, DAQmx_Val_GroupByChannel, read_origin, 1000, NULL, NULL));
	}

	for (int i = 0; i < runTime * sampleFs / samplesPerChan; i++)
	{

		for (int i = 0; i < samplesPerChan; i++)
		{

			float64 d = 2.0 * sin(globalTime * dist_freq * 2 * PI);
			float64 u = 0;
			float64 y = (read_origin[i] - 0.00025) * 8000;

			DefaultController::defaultController(&controller, y, u);

			data.t.push_back(globalTime);
			data.d.push_back(d);
			data.u.push_back(u);
			data.y.push_back(y);

			writeChan0[i] = d;
			writeChan1[i] = u;

			index += 1;
			globalTime += 1 / sampleFs;
		}

		// DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		// DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 10.0, DAQmx_Val_GroupByChannel, read_origin, 1000, NULL, NULL));
	}

	packer.pack(data);
	stream.close();
	return 0;
}
