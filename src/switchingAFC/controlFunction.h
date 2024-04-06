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

#include <utils.h>

class SwitchingAFC
{
public:
	typedef std::vector<double> state_type;
	state_type w;
	double w_star_, dt_, h, d1, d2, out, tw, bound;
	double alpha, epsilon, gamma;
	double switching_signal, reset_signal;
	std::vector<double> switching_signal_stack;
	Eigen::Matrix2d S;
	Eigen::Vector2d G, theta_hat;
	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

	SwitchingAFC(double w_star, double dt, double theta1, double theta2, double alpha_, double eps, double gamma_, double h_, double d1_ = 0.01, double d2_ = 5.0, double bound_ = 3.95) : w(8), w_star_(w_star), dt_(dt), h(h_), d1(d1_), d2(d2_), out(0), tw(0), bound(bound_), alpha(alpha_), epsilon(eps), gamma(gamma_), reset_signal(0), switching_signal_stack(10)
	{
		theta_hat << theta1, theta2;

		w = {0, 0, 0, 0, 0, 0, theta1, theta2};

		G << 1, 0;
		S << 0, w_star_,
			-w_star_, 0;
	}

	void state_split(const state_type &y, Eigen::Vector2d &w_hat, Eigen::Vector2d &zeta_hat, Eigen::Vector2d &xi_hat, Eigen::Vector2d &eta)
	{
		w_hat << y[0], y[1];
		zeta_hat << y[2], y[3];
		xi_hat << y[4], y[5];
		eta << y[6], y[7];
	}

	state_type state_merge(const Eigen::Vector2d &w_hat, const Eigen::Vector2d &zeta_hat, const Eigen::Vector2d &xi_hat, const Eigen::Vector2d &eta)
	{
		return {w_hat(0), w_hat(1), zeta_hat(0), zeta_hat(1), xi_hat(0), xi_hat(1), eta(0), eta(1)};
	}

	void equations(const state_type &y, state_type &dy, double t)
	{
		Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta, eta_inner_proj;
		state_split(y, w_hat, zeta_hat, xi_hat, eta);

		eta_inner_proj = discreteProjection2dOut(eta, d1);

		Eigen::Vector2d dot_w_hat, dot_zeta_hat, dot_xi_hat, dot_eta;
		double u_a = -epsilon * theta_hat.transpose() * zeta_hat;
		dot_w_hat = S * w_hat + G * (u_a);
		dot_zeta_hat = S * zeta_hat + theta_hat * u_a - alpha * G * (G.transpose() * zeta_hat - out);
		dot_xi_hat = (S - alpha * G * G.transpose()).transpose() * xi_hat + G * u_a;
		dot_eta = -gamma * xi_hat * (G.transpose() * zeta_hat - out - (theta_hat - eta).transpose() * xi_hat);

		if ((eta - theta_hat).norm() > h)
		{
			theta_hat = eta;
			switching_signal_stack.emplace_back(1.0);
		}
		else
		{
			switching_signal_stack.emplace_back(0);
		}

		dy = std::move(state_merge(dot_w_hat, dot_zeta_hat, dot_xi_hat, dot_eta));
	}

	void setInput(double y) { out = y; }

	double computeOutput()
	{
		auto func = std::bind(&SwitchingAFC::equations, &(*this), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		stepper.do_step(func, w, tw, dt_);
		tw += dt_;

		switching_signal = *std::max_element(switching_signal_stack.begin(), switching_signal_stack.end());
		switching_signal_stack.clear();

		Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
		state_split(w, w_hat, zeta_hat, xi_hat, eta);
		if (w_hat.norm() > bound)
		{
			w_hat.setZero();
			zeta_hat.setZero();
			xi_hat.setZero();
			reset_signal = 1.0;
		}
		else
		{
			reset_signal = 0;
		}
		w = std::move(state_merge(w_hat, zeta_hat, xi_hat, eta));

		return w[0];
	}
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double y);

void controllerFinish(void *ctrl_ptr);