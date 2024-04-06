#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

#include "yaml.h"

struct controller_t
{
    double epsilon;
    double rho;
    double beta;
    double delta1;
    double delta2;
    double h;
    Eigen::Vector2d g;
    Eigen::RowVector2d gamma;
    Eigen::Matrix<double, 4, 2> G;
    Eigen::RowVector4d Gamma;
    Eigen::Matrix<double, 4, 4> S;
    Eigen::Matrix<double, 4, 4> E;
    Eigen::VectorXd zetao;
    Eigen::VectorXd theta;
    Eigen::Vector4d hatw;
    int sigma;
    Eigen::Vector4d eta1;
    double m;
    double y;
    double u;
    Eigen::Vector2d ua;
    Eigen::VectorXd J;
    Eigen::Vector2d x;
    Eigen::Vector4d v;
    double omega1;
    double omega2;
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Vector2d B;
    Eigen::RowVector2d C;
    Eigen::Matrix<double, 4, 2> theta_bar;

    controller_t() : zetao(36), theta(36), J(9)
    {
    }
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double d);

void controllerFinish(void *ctrl_ptr);