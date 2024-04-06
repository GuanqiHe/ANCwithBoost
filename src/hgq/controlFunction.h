#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>

#include "yaml.h"
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include "msgpack.hpp"

#include "logger/data_logging.hpp"


class MTController
{
public:
	typedef std::vector<double> state_type;
	double out, tw, w_star_, dt_, bound_;
	Eigen::MatrixXd G;
	state_type w;
	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

	MTController(double w_star, double g1, double g2, double dt, double bound = 3.95) : w(6), w_star_(w_star), G(2, 1), dt_(dt), bound_(bound)
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
		// if (w_hat.norm() >= bound_)
		// {
		// 	w_hat = w_hat / w_hat.norm() * bound_;
		// }

		Eigen::Vector2d dot_w_hat = S * w_hat + G * (out);

		dy[0] = dot_w_hat(0);
		dy[1] = dot_w_hat(1);
	}

	void setInput(double y) { out = y; }

	double computeOutput()
	{
		auto func = std::bind(&MTController::equations, &(*this), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		stepper.do_step(func, w, tw, dt_);
		tw += dt_;
		return w[0];
	}
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double y);

void controllerFinish(void *ctrl_ptr);