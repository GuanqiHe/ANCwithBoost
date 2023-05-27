#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>

#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>

#include "NIDAQmx.h"
#include "msgpack.hpp"

#define DAQmxErrChk(functionCall)            \
	if (DAQmxFailed(error = (functionCall))) \
		goto Error;                          \
	else

#define PI 3.1415926535

typedef std::vector<double> array_t;

struct datapack_t
{
	array_t t;
	array_t y;
	array_t d;
	array_t u;
	array_t phi;
	MSGPACK_DEFINE_MAP(t, y, d, u);
};

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

class DefaultController
{
public:
	typedef std::vector<double> state_type;
	float64 out, tw, w_star_, dt_, bound_;
	Eigen::MatrixXd G;
	state_type w;
	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

	DefaultController(float64 w_star, float64 g1, float64 g2, float64 dt, float64 bound = 3.95) : w(6), w_star_(w_star), G(2, 1), dt_(dt), bound_(bound)
	{
		tw = 0.0;
		out = 0.0;
		w[0] = 0.0;
		w[1] = 0.0;
		G << g1, g2;
	}

	void equations(const state_type &y, state_type &dy, double _x)
	{
		Eigen::MatrixXd S(2, 2);
		S << 0, w_star_,
			-w_star_, 0;

		Eigen::Vector2d w_hat;
		w_hat << y[0], y[1];
		if (w_hat.norm() >= bound_)
		{
			w_hat = w_hat / w_hat.norm() * bound_;
		}

		Eigen::Vector2d dot_w_hat = S * w_hat + G * (out);

		dy[0] = dot_w_hat(0);
		dy[1] = dot_w_hat(1);
	}

	void setInput(float64 y) { out = y; }

	float64 computeOutput()
	{
		auto func = std::bind(&DefaultController::equations, &(*this), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		stepper.do_step(func, w, tw, dt_);
		return w[0];
	}
};

void defaultController(void *ptr_controller, const float64 &y, float64 &u)
{
	DefaultController *ptr = (DefaultController *)(ptr_controller);
	ptr->setInput(y);
	u = ptr->computeOutput();
}

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

	const int32 samplesPerChan = 100;
	const float64 sampleFs = 5000.0, dist_freq = 35.0;

	const int32 writeNumChan = 2;
	float64 write_origin[samplesPerChan * writeNumChan] = {0};
	float64 *writeChan0 = &write_origin[0], *writeChan1 = &write_origin[samplesPerChan];

	const int32 readNumChan = 1;
	float64 read_origin[samplesPerChan * readNumChan] = {0};
	float64 *readChan0 = &read_origin[0];

	const float runTime = 30, warmUp = 1.0; // sec
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

	DefaultController controller(21.5 * 2 * PI, -30.0, 0.0, 1.0 / sampleFs);

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle_w));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle_w, "Mod1/ao0:1", "", -4.0, 4.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle_w, "OnboardClock", sampleFs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle_w, 0, DoneCallback, NULL));

	DAQmxErrChk(DAQmxCreateTask("", &taskHandle_r));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle_r, "Mod2/ai0", "", DAQmx_Val_PseudoDiff, -4.0, 4.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle_r, "OnboardClock", sampleFs, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesPerChan));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle_w));
	DAQmxErrChk(DAQmxStartTask(taskHandle_r));

	for (int i = 0; i < warmUp * sampleFs / samplesPerChan; i++)
	{

		for (int i = 0; i < samplesPerChan; i++)
		{

			float64 d = 1.5 * sin(globalTime * dist_freq * 2 * PI);
			float64 u = 0;
			float64 y = (read_origin[i]) * 8000 - 1.7;

			data.t.push_back(globalTime);
			data.d.push_back(d);
			data.u.push_back(u);
			data.y.push_back(y);

			writeChan0[i] = d;
			writeChan1[i] = u;

			index += 1;
			globalTime += 1 / sampleFs;
		}

		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 10.0, DAQmx_Val_GroupByChannel, read_origin, 1000, NULL, NULL));
	}

	for (int i = 0; i < runTime * sampleFs / samplesPerChan; i++)
	{

		for (int i = 0; i < samplesPerChan; i++)
		{

			float64 d = 1.5 * sin(globalTime * dist_freq * 2 * PI);
			float64 u = 0;
			float64 y = (read_origin[i]) * 8000 - 1.7;

			defaultController(&controller, y, u);

			data.t.push_back(globalTime);
			data.d.push_back(d);
			data.u.push_back(u);
			data.y.push_back(y);

			writeChan0[i] = d;
			writeChan1[i] = u;

			index += 1;
			globalTime += 1 / sampleFs;
		}

		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle_w, samplesPerChan, 0, 10.0, DAQmx_Val_GroupByChannel, write_origin, NULL, NULL));
		DAQmxErrChk(DAQmxReadAnalogF64(taskHandle_r, samplesPerChan, 10.0, DAQmx_Val_GroupByChannel, read_origin, 1000, NULL, NULL));
	}

	// printf("Generating voltage continuously. Press Enter to interrupt\n");
	// getchar();

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle_w != 0)
	{
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle_w);
		DAQmxClearTask(taskHandle_w);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
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
