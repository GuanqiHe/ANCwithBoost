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

class RobustAFC
{
public:
	typedef std::vector<double> state_type;
	double out, tw, w_star_, dt_, bound_;
	double alpha, epsilon, gamma;
	Eigen::Matrix2d S;
	Eigen::Vector2d G, theta_hat;
	state_type w;
	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

	RobustAFC(double w_star, double dt, double theta1, double theta2, double alpha_, double eps, double gamma_, double bound = 3.95) : w(8), w_star_(w_star), dt_(dt), bound_(bound), alpha(alpha_), epsilon(eps), gamma(gamma_)
	{
		tw = 0.0;
		out = 0.0;

		w = {0, 0, 0, 0, 0, 0, 0, 0};

		theta_hat << theta1, theta2;

		G << 1, 0;
		S << 0, w_star_,
			-w_star_, 0;
	}

	void equations(const state_type &y, state_type &dy, double t)
	{
		Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
		w_hat << y[0], y[1];
		zeta_hat << y[2], y[3];
		xi_hat << y[4], y[5];
		eta << y[6], y[7];

		Eigen::Vector2d dot_w_hat, dot_zeta_hat, dot_xi_hat, dot_eta;
		double u_a = -epsilon * theta_hat.transpose() * zeta_hat;
		dot_w_hat = S * w_hat + G * (u_a);
		dot_zeta_hat = S * zeta_hat + theta_hat * u_a - alpha * G * (G.transpose() * zeta_hat - out);
		dot_xi_hat = (S - alpha * G * G.transpose()).transpose() * xi_hat + G * u_a;
		dot_eta = -gamma * xi_hat * (G.transpose() * zeta_hat - out - (theta_hat - eta).transpose() * xi_hat);

		dy = {dot_w_hat(0), dot_w_hat(1), dot_zeta_hat(0), dot_zeta_hat(1), dot_xi_hat(0), dot_xi_hat(1), dot_eta(0), dot_eta(1)};
	}

	void setInput(double y) { out = y; }

	double computeOutput()
	{
		auto func = std::bind(&RobustAFC::equations, &(*this), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		stepper.do_step(func, w, tw, dt_);
		tw += dt_;
		return w[0];
	}
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double y);

void controllerFinish(void *ctrl_ptr);