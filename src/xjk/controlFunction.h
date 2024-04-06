//
//  controlFunction.hpp
//  proj_1
//
//  Created by Jacob on 2023/5/27.
//
#pragma once


#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>

#include "yaml.h"



struct controller_t
{
    double epsilon;
    double rho;
    double beta;
    double delta1;
    double delta2;
    double h;
    std::vector<std::vector<double>> G;
    double ua_1;
    double ua_2;
    double ua_3;
    std::vector<std::vector<double>> zetao_1;
    std::vector<std::vector<double>> zetao_2;
    std::vector<std::vector<double>> zetao_3;
    std::vector<std::vector<double>> theta_1;
    std::vector<std::vector<double>> theta_2;
    std::vector<std::vector<double>> theta_3;
    std::vector<std::vector<double>> hatw;
    int sigma;
    std::vector<std::vector<double>> eta1;
    double ma;
    double mb;
    double mc;
    double y;
    double u;
    double ua_sigma;
    double J_1;
    double J_2;
    double J_3;
    std::vector<std::vector<double>> J;
    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> v;
    double omega1;

    // Define the discrete system
    std::vector<std::vector<double>> A;
    std::vector<std::vector<double>> B;
    std::vector<std::vector<double>> C;
    std::vector<std::vector<double>> S;
    std::vector<std::vector<double>> G_transpose;
    std::vector<std::vector<double>> GG_transpose;
    std::vector<std::vector<double>> F_epsilon;
};



void* controllerInit(int argc, char *argv[]);

double controllerCompute(void * ctrl_ptr, double d);


void controllerFinish(void* ctrl_ptr);