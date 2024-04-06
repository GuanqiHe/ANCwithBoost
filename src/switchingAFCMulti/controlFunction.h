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

template <class T>
inline void SIZE_CHECK(const std::vector<T> &v, int size, std::string err = "")
{
	#ifdef DEBUG
	if (v.size() != size)
	{
		std::cout << "Incompatible Dimension:" + err << std::endl;
		throw err;
	}
	#endif
}

class SingleAFC
{
public:
	static constexpr int state_dim = 8;
	typedef std::vector<double> state_type;
	double w_star, alpha, epsilon, gamma, h, d2;
	Eigen::Matrix2d S;
	Eigen::Vector2d G, theta_hat;

	SingleAFC(double w_star_, Eigen::Vector2d theta, double alpha_, double eps_, double gamma_, double h_, double d2_ = 5.0) : w_star(w_star_), alpha(alpha_), epsilon(eps_), gamma(gamma_), h(h_), d2(d2_), theta_hat(theta)
	{

		S << 0, w_star,
			-w_star, 0;

		G << 1, 0;
	}

	state_type get_init_state()
	{
		return {0, 0, 0, 0, 0, 0, theta_hat(0), theta_hat(1)};
	}

	static void state_split(const state_type &y, Eigen::Vector2d &w_hat, Eigen::Vector2d &zeta_hat, Eigen::Vector2d &xi_hat, Eigen::Vector2d &eta)
	{
		w_hat << y[0], y[1];
		zeta_hat << y[2], y[3];
		xi_hat << y[4], y[5];
		eta << y[6], y[7];
	}

	static state_type state_merge(const Eigen::Vector2d &w_hat, const Eigen::Vector2d &zeta_hat, const Eigen::Vector2d &xi_hat, const Eigen::Vector2d &eta)
	{
		return {w_hat(0), w_hat(1), zeta_hat(0), zeta_hat(1), xi_hat(0), xi_hat(1), eta(0), eta(1)};
	}

	void equations(const state_type &y, state_type &dy, double y_diff, double theta_reg)
	{
		Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
		state_split(y, w_hat, zeta_hat, xi_hat, eta);
		double u_a = -epsilon * theta_hat.transpose() * zeta_hat;

		Eigen::Vector2d dot_w_hat, dot_zeta_hat, dot_xi_hat, dot_eta;
		dot_w_hat = S * w_hat + G * (u_a);
		dot_zeta_hat = S * zeta_hat + theta_hat * u_a - alpha * G * y_diff;
		dot_xi_hat = (S - alpha * G * G.transpose()).transpose() * xi_hat + G * u_a;
		dot_eta = -gamma * xi_hat * (y_diff - theta_reg);

		if ((eta - theta_hat).norm() > h)
		{
			theta_hat = eta;
		}
		dy = std::move(state_merge(dot_w_hat, dot_zeta_hat, dot_xi_hat, dot_eta));
	}

	static double computeOutput(const state_type &y)
	{
		SIZE_CHECK(y, state_dim, "computeOutput y");
		return y[0];
	}

	state_type resetController(const state_type &y)
	{
		Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
		state_split(y, w_hat, zeta_hat, xi_hat, eta);
		w_hat.setZero();
		zeta_hat.setZero();
		xi_hat.setZero();
		return state_merge(w_hat, zeta_hat, xi_hat, eta);
	}
};

class SwitchingAFC
{
public:
	typedef std::vector<double> state_type;
	state_type w;
	int dist_num;
	double dt_, h, d1, d2, out, tw, bound;
	double alpha, epsilon, gamma;
	double reset_signal;
	Eigen::Vector2d G;

	std::vector<SingleAFC> subController;

	boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

	SwitchingAFC(int dist_num_, std::vector<double> w_star, double dt, std::vector<Eigen::Vector2d> theta_hat, double alpha_, double eps, double gamma_, double h_, double d1_ = 0.01, double d2_ = 5.0, double bound_ = 3.95) : dist_num(dist_num_), dt_(dt), h(h_), d1(d1_), d2(d2_), out(0), tw(0), bound(bound_), alpha(alpha_), epsilon(eps), gamma(gamma_), reset_signal(0)
	{

		SIZE_CHECK(w_star, dist_num, "w_star");

		for (int i = 0; i < dist_num; i++)
		{
			subController.emplace_back(SingleAFC(w_star[i], theta_hat[i], alpha, epsilon, gamma, h, d2));
			auto sub_x_init = subController[i].get_init_state();
			for (double x : sub_x_init)
			{
				w.push_back(x);
			}
		}

		SIZE_CHECK(subController, dist_num, "subController");
		SIZE_CHECK(w, dist_num * SingleAFC::state_dim, "w");

		G << 1, 0;
	}

	void state_split(const state_type &y, std::vector<SingleAFC::state_type> &x)
	{
		SIZE_CHECK(x, dist_num, "split_x");
		int index = 0;
		for (int i = 0; i < dist_num; i++)
		{
			SingleAFC::state_type p(SingleAFC::state_dim);
			for (int j = 0; j < SingleAFC::state_dim; j++)
			{
				p[j] = y[index++];
			}
			SIZE_CHECK(p, SingleAFC::state_dim, "p");
			x[i]=std::move(p);
		}
	}

	state_type state_merge(const std::vector<SingleAFC::state_type> &x)
	{
		SIZE_CHECK(x, dist_num, "merge_dx_vector");
		state_type p(SingleAFC::state_dim * dist_num);
		int index=0;
		for (const auto &i : x)
		{
			for (const auto &j : i)
			{
				p[index++]=j;
			}
		}
		SIZE_CHECK(p, SingleAFC::state_dim * dist_num, "merge_ret");
		return p;
	}

	void equations(const state_type &y, state_type &dy, double t)
	{
		std::vector<SingleAFC::state_type> x(dist_num, SingleAFC::state_type(SingleAFC::state_dim)), dx(dist_num, SingleAFC::state_type(SingleAFC::state_dim));
		state_split(y, x);
		double y_diff = -out;
		double theta_reg = 0;
		for (int i = 0; i < dist_num; i++)
		{
			Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
			SingleAFC::state_split(x[i], w_hat, zeta_hat, xi_hat, eta);
			y_diff += (G.transpose() * zeta_hat).value();
			theta_reg += (subController[i].theta_hat - eta).transpose() * xi_hat;
		}

		for (int i = 0; i < dist_num; i++)
		{
			Eigen::Vector2d w_hat, zeta_hat, xi_hat, eta;
			SingleAFC::state_split(x[i], w_hat, zeta_hat, xi_hat, eta);
			subController[i].equations(x[i], dx[i], y_diff, theta_reg);
		}

		dy = std::move(state_merge(dx));
	}

	void setInput(double y) { out = y; }

	double computeOutput()
	{
		auto func = std::bind(&SwitchingAFC::equations, &(*this), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		stepper.do_step(func, w, tw, dt_);
		tw += dt_;

		std::vector<SingleAFC::state_type> x(dist_num, SingleAFC::state_type(SingleAFC::state_dim));
		state_split(w, x);
		double output = 0;
		for (const auto &i : x)
		{
			output += SingleAFC::computeOutput(i);
		}

		if (output > bound)
		{
			for (int i = 0; i < dist_num; i++)
			{
				x[i] = std::move(subController[i].resetController(x[i]));
			}

			reset_signal = 1.0;

			output = 0.0;
		}
		else
		{
			reset_signal = 0;
		}
		w = std::move(state_merge(x));

		return output;
	}
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double y);

void controllerFinish(void *ctrl_ptr);


//未测试