//
//  controlFunction.cpp
//  proj_1
//
//  Created by Jacob on 2023/5/27.
//

#include "controlFunction.h"

void *controllerInit(int argc, char *argv[])
{
    controller_t *ctrl_ptr = new controller_t();


	YAML::Node config = YAML::LoadFile("config.yaml");

    ctrl_ptr->epsilon = config["epsilon"].as<double>();
    ctrl_ptr->rho = 0.5;
    ctrl_ptr->beta = 1;
    ctrl_ptr->delta1 = 0.1;
    ctrl_ptr->delta2 = 3;
    ctrl_ptr->h = 0.5;
    ctrl_ptr->G = {{1}, {0}};

    // Initialization
    ctrl_ptr->ua_1 = 0;
    ctrl_ptr->ua_2 = 0;
    ctrl_ptr->ua_3 = 0;
    ctrl_ptr->zetao_1 = {{0}, {0}};
    ctrl_ptr->zetao_2 = {{0}, {0}};
    ctrl_ptr->zetao_3 = {{0}, {0}};
    ctrl_ptr->theta_1 = {{1}, {1}};
    ctrl_ptr->theta_2 = {{-1}, {1}};
    ctrl_ptr->theta_3 = {{0}, {-1}};
    ctrl_ptr->hatw = {{0}, {-1}};
    ctrl_ptr->sigma = 0;
    ctrl_ptr->eta1 = {{0}, {0}};
    ctrl_ptr->ma = 0;
    ctrl_ptr->mb = 0;
    ctrl_ptr->mc = 0;
    ctrl_ptr->y = 0;
    ctrl_ptr->u = 0;
    ctrl_ptr->ua_sigma = 0;
    ctrl_ptr->J_1 = 0;
    ctrl_ptr->J_2 = 0;
    ctrl_ptr->J_3 = 0;
    ctrl_ptr->J = {{ctrl_ptr->J_1}, {ctrl_ptr->J_2}, {ctrl_ptr->J_3}};
    ctrl_ptr->x = {{0}, {0}};
    ctrl_ptr->v = {{0}, {2}};
    ctrl_ptr->omega1 = 2 * M_PI * config["dist_freq"].as<double>() / 5000;

    // Define the discrete system
    ctrl_ptr->A = {{1.4, -0.728}, {0.728, 0}};
    ctrl_ptr->B = {{1}, {0}};
    ctrl_ptr->C = {{2, -3.0219}};
    ctrl_ptr->S.push_back({std::cos(ctrl_ptr->omega1), std::sin(ctrl_ptr->omega1)});
    ctrl_ptr->S.push_back({-std::sin(ctrl_ptr->omega1), std::cos(ctrl_ptr->omega1)});

    ctrl_ptr->G_transpose = {{ctrl_ptr->G[0][0]}, {ctrl_ptr->G[1][0]}};
    ctrl_ptr->GG_transpose = {{ctrl_ptr->G[0][0] * ctrl_ptr->G_transpose[0][0], ctrl_ptr->G[0][1] * ctrl_ptr->G_transpose[0][1]},
                              {ctrl_ptr->G[1][0] * ctrl_ptr->G_transpose[1][0], ctrl_ptr->G[1][1] * ctrl_ptr->G_transpose[1][1]}};

    ctrl_ptr->F_epsilon = std::vector<std::vector<double>>(2, std::vector<double>(2));

    for (int m = 0; m < 2; m++)
    {
        for (int n = 0; n < 2; n++)
        {
            ctrl_ptr->F_epsilon[m][n] = ctrl_ptr->S[m][n] - ctrl_ptr->epsilon * ctrl_ptr->GG_transpose[m][n];
        }
    }

    return (void *)(ctrl_ptr);
}

// 计算矩阵的 Frobenius 范数
double matrixNorm(const std::vector<std::vector<double>> &matrix)
{
    double norm = 0.0;

    for (const auto &row : matrix)
    {
        for (double element : row)
        {
            norm += element * element;
        }
    }
    return std::sqrt(norm);
}

double controllerCompute(void *ctrl_ptr, double d)
{
    // Constants

    //  std::ofstream outputFile("/Users/jacob/Library/CloudStorage/OneDrive-个人/Service/output.csv");

    controller_t *ptr = (controller_t *)(ctrl_ptr);

    // Loop
    //  v = {{ptr->S[0][0] * v[0][0] + ptr->S[0][1] * v[1][0]}, {ptr->S[1][0] * v[0][0] + ptr->S[1][1] * v[1][0]}};
    //  double d = ptr->G[0][0] * v[0][0] + ptr->G[1][0] * v[1][0];

    ptr->hatw = {{ptr->S[0][0] * ptr->hatw[0][0] + ptr->S[0][1] * ptr->hatw[1][0] + ptr->G[0][0] * ptr->ua_sigma}, {ptr->S[1][0] * ptr->hatw[0][0] + ptr->S[1][1] * ptr->hatw[1][0] + ptr->G[1][0] * ptr->ua_sigma}};
    ptr->u = ptr->hatw[0][0];
    ptr->x = {{ptr->A[0][0] * ptr->x[0][0] + ptr->A[0][1] * ptr->x[1][0] + ptr->B[0][0] * (ptr->u - d)}, {ptr->A[1][0] * ptr->x[0][0] + ptr->A[1][1] * ptr->x[1][0] + ptr->B[1][0] * (ptr->u - d)}};

    ptr->y = ptr->C[0][0] * ptr->x[0][0] + ptr->C[0][1] * ptr->x[1][0];

    ptr->ua_1 = -ptr->epsilon * (ptr->theta_1[0][0] * ptr->zetao_1[0][0] + ptr->theta_1[1][0] * ptr->zetao_1[1][0]);
    ptr->ua_2 = -ptr->epsilon * (ptr->theta_2[0][0] * ptr->zetao_2[0][0] + ptr->theta_2[1][0] * ptr->zetao_2[1][0]);
    ptr->ua_3 = -ptr->epsilon * (ptr->theta_3[0][0] * ptr->zetao_3[0][0] + ptr->theta_3[1][0] * ptr->zetao_3[1][0]);

    switch (ptr->sigma)
    {
    case 0:
        ptr->ua_sigma = ptr->ua_1;
        break;
    case 1:
        ptr->ua_sigma = ptr->ua_2;
        break;
    case 2:
        ptr->ua_sigma = ptr->ua_3;
        break;
    }

    ptr->zetao_1 = {{ptr->S[0][0] * ptr->zetao_1[0][0] + ptr->S[0][1] * ptr->zetao_1[1][0] + ptr->theta_1[0][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[0][0] * (ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0]) - ptr->y)},
                    {ptr->S[1][0] * ptr->zetao_1[0][0] + ptr->S[1][1] * ptr->zetao_1[1][0] + ptr->theta_1[1][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[1][0] * (ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0]) - ptr->y)}};

    ptr->zetao_2 = {{ptr->S[0][0] * ptr->zetao_2[0][0] + ptr->S[0][1] * ptr->zetao_2[1][0] + ptr->theta_2[0][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[0][0] * (ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0]) - ptr->y)},
                    {ptr->S[1][0] * ptr->zetao_2[0][0] + ptr->S[1][1] * ptr->zetao_2[1][0] + ptr->theta_2[1][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[1][0] * (ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0]) - ptr->y)}};

    ptr->zetao_3 = {{ptr->S[0][0] * ptr->zetao_3[0][0] + ptr->S[0][1] * ptr->zetao_3[1][0] + ptr->theta_3[0][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[0][0] * (ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0]) - ptr->y)},
                    {ptr->S[1][0] * ptr->zetao_3[0][0] + ptr->S[1][1] * ptr->zetao_3[1][0] + ptr->theta_3[1][0] * ptr->ua_sigma - ptr->epsilon * (ptr->G[1][0] * (ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0]) - ptr->y)}};

    ptr->eta1 = {{ptr->F_epsilon[0][0] * ptr->eta1[0][0] + ptr->F_epsilon[0][1] * ptr->eta1[1][0] + ptr->G[0][0] * ptr->ua_sigma},
                 {ptr->F_epsilon[1][0] * ptr->eta1[0][0] + ptr->F_epsilon[1][1] * ptr->eta1[1][0] + ptr->G[1][0] * ptr->ua_sigma}};

    ptr->ma = 1 + matrixNorm(ptr->eta1) + std::pow(ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0] - ptr->y, 2);
    ptr->mb = 1 + matrixNorm(ptr->eta1) + std::pow(ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0] - ptr->y, 2);
    ptr->mc = 1 + matrixNorm(ptr->eta1) + std::pow(ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0] - ptr->y, 2);

    ptr->theta_1 = {{ptr->theta_1[0][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[0][0] * (ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0] - ptr->y) / ptr->ma},
                    {ptr->theta_1[1][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[1][0] * (ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0] - ptr->y) / ptr->ma}};

    ptr->theta_2 = {{ptr->theta_2[0][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[0][0] * (ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0] - ptr->y) / ptr->mb},
                    {ptr->theta_2[1][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[1][0] * (ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0] - ptr->y) / ptr->mb}};

    ptr->theta_3 = {{ptr->theta_3[0][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[0][0] * (ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0] - ptr->y) / ptr->mc},
                    {ptr->theta_3[1][0] - ptr->rho * std::pow(ptr->epsilon, 2) * ptr->eta1[1][0] * (ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0] - ptr->y) / ptr->mc}};

    // 检查条件并更新 ptr->theta_1
    if (std::sqrt(3) * ptr->theta_1[0][0] + ptr->theta_1[1][0] < ptr->delta1)
    {
        ptr->theta_1 = ptr->theta_1;
    }
    if (ptr->theta_1[0][0] / std::sqrt(3) + ptr->theta_1[1][0] < 0)
    {
        ptr->theta_1 = ptr->theta_1;
    }
    if (ptr->theta_1[0][0] < 0)
    {
        ptr->theta_1 = ptr->theta_1;
    }
    if (matrixNorm(ptr->theta_1) > ptr->delta2)
    {
        ptr->theta_1 = ptr->theta_1;
    }

    // 检查条件并更新 ptr->theta_2
    if (-std::sqrt(3) * ptr->theta_2[0][0] + ptr->theta_2[1][0] < ptr->delta1)
    {
        ptr->theta_2 = ptr->theta_2;
    }
    if (-ptr->theta_2[0][0] / std::sqrt(3) + ptr->theta_2[1][0] < 0)
    {
        ptr->theta_2 = ptr->theta_2;
    }
    if (ptr->theta_2[0][0] > 0)
    {
        ptr->theta_2 = ptr->theta_2;
    }
    if (matrixNorm(ptr->theta_2) > ptr->delta2)
    {
        ptr->theta_2 = ptr->theta_2;
    }

    // 检查条件并更新 ptr->theta_3
    if (ptr->theta_3[1][0] > -ptr->delta1 / 2)
    {
        ptr->theta_3 = ptr->theta_3;
    }
    if (ptr->theta_3[0][0] / std::sqrt(3) + ptr->theta_3[1][0] > 0)
    {
        ptr->theta_3 = ptr->theta_3;
    }
    if (-ptr->theta_3[0][0] / std::sqrt(3) + ptr->theta_3[1][0] > 0)
    {
        ptr->theta_3 = ptr->theta_3;
    }
    if (matrixNorm(ptr->theta_3) > ptr->delta2)
    {
        ptr->theta_3 = ptr->theta_3;
    }

    ptr->J_1 += ptr->beta * std::pow(ptr->G[0][0] * ptr->zetao_1[0][0] + ptr->G[1][0] * ptr->zetao_1[1][0] - ptr->y, 2);
    ptr->J_2 += ptr->beta * std::pow(ptr->G[0][0] * ptr->zetao_2[0][0] + ptr->G[1][0] * ptr->zetao_2[1][0] - ptr->y, 2);
    ptr->J_3 += ptr->beta * std::pow(ptr->G[0][0] * ptr->zetao_3[0][0] + ptr->G[1][0] * ptr->zetao_3[1][0] - ptr->y, 2);
    ptr->J[ptr->sigma][0] = ptr->J[ptr->sigma][0] - ptr->h;

    for (int k = 0; k < 3; k++)
    {
        if (int(ptr->J[k][0]) <= int(ptr->J[ptr->sigma][0]))
        {
            ptr->J[ptr->sigma][0] = ptr->J[k][0];
            ptr->sigma = k;
        }
    }

    // //     outputFile << ptr->y << std::endl;
    // std::cout << "ptr->y: " << ptr->y << std::endl;
    // std::cout << "ptr->sigma: " << ptr->sigma << std::endl;
    // std::cout << "ptr->J_1: " << ptr->J_1 << std::endl;
    // std::cout << "ptr->J_2: " << ptr->J_2 << std::endl;
    // std::cout << "ptr->J_3: " << ptr->J_3 << std::endl;

    //  std::this_thread::sleep_for(interval);

    // outputFile.close();

    return ptr->u;
}


void controllerFinish(void * ctrl_ptr)
{
    controller_t* ptr = (controller_t*)(ctrl_ptr);
    delete ptr;
}