#include "controlFunction.h"

void *controllerInit(int argc, char *argv[])
{
    controller_t *ctrl_ptr = new controller_t();

    std::string config_path = "config.yaml";
    if (argc >= 3)
    {
        config_path = std::string(argv[2]);
    }
    else if (argc == 2)
    {
        config_path = std::string(argv[1]);
    }

    YAML::Node config = YAML::LoadFile(config_path);

    ctrl_ptr->epsilon = config["epsilon"].as<double>();
    ctrl_ptr->rho = 0.5;
    ctrl_ptr->beta = 1;
    ctrl_ptr->delta1 = 0.1;
    ctrl_ptr->delta2 = 3;
    ctrl_ptr->h = 0.5;
    ctrl_ptr->y = 0;
    ctrl_ptr->A << 1.4, -0.728, 0.728, 0;
    ctrl_ptr->B << 1, 0;
    ctrl_ptr->C << 2, -3.0219;
    ctrl_ptr->x << 0, 0;
    ctrl_ptr->v << 0, 2, 0, 2;
    ctrl_ptr->omega2 = config["omega2"].as<double>();
    ctrl_ptr->omega1 = config["omega1"].as<double>();
    ctrl_ptr->g << 1, 0;
    ctrl_ptr->gamma << 1, 0;
    ctrl_ptr->G << 1, 0, 0, 0, 0, 1, 0, 0;
    ctrl_ptr->Gamma << 1, 0, 1, 0;
    ctrl_ptr->S << std::cos(ctrl_ptr->omega1), std::sin(ctrl_ptr->omega1), 0, 0, -std::sin(ctrl_ptr->omega1), std::cos(ctrl_ptr->omega1), 0, 0, 0, 0, std::cos(ctrl_ptr->omega2), std::sin(ctrl_ptr->omega2),
        0, 0, -std::sin(ctrl_ptr->omega2), std::cos(ctrl_ptr->omega2);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ctrl_ptr->E(i, j) = ctrl_ptr->S(i, j) - ctrl_ptr->epsilon * ctrl_ptr->Gamma(0, i) * ctrl_ptr->Gamma(0, j);
        }
    }
    ctrl_ptr->zetao.setZero();
    ctrl_ptr->theta << 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 0, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 0, -1, 0, -1, 1, 1, 0, -1, -1, 1, 0, -1, 0, -1;
    ctrl_ptr->hatw << 0, -1, 0, -1;
    ctrl_ptr->sigma = 0;
    ctrl_ptr->eta1 << 0.0, 0.0, 0.0, 0.0;
    ctrl_ptr->m = 0.0;
    ctrl_ptr->u = 0.0;
    ctrl_ptr->ua << 0.0, 0.0;
    ctrl_ptr->J.setZero();
    ctrl_ptr->theta_bar.setZero();

    return (void *)(ctrl_ptr);
}

double controllerCompute(void *ptr, double d)
{

    controller_t *ctrl_ptr = (controller_t *)(ptr);

    ctrl_ptr->ua = -ctrl_ptr->epsilon * ctrl_ptr->theta_bar.transpose() * ctrl_ptr->zetao.segment(ctrl_ptr->sigma * 4, 4);
    ctrl_ptr->hatw = ctrl_ptr->S * ctrl_ptr->hatw + ctrl_ptr->G * ctrl_ptr->ua;
    ctrl_ptr->u = ctrl_ptr->hatw(0) + ctrl_ptr->hatw(2);
    ctrl_ptr->x = ctrl_ptr->A * ctrl_ptr->x + (ctrl_ptr->u - d) * ctrl_ptr->B;
    ctrl_ptr->y = ctrl_ptr->C * ctrl_ptr->x;
    for (int j = 0; j < 9; j++)
    {
        ctrl_ptr->theta_bar.block(0, 0, 2, 2) = ctrl_ptr->theta.segment(j * 4, 2).replicate(1, 2);
        ctrl_ptr->theta_bar.block(2, 0, 2, 2) = ctrl_ptr->theta.segment(j * 4 + 2, 2).replicate(1, 2);
        ctrl_ptr->zetao.segment(j * 4, 4) = ctrl_ptr->S * ctrl_ptr->zetao.segment(j * 4, 4) + ctrl_ptr->theta_bar * ctrl_ptr->ua - ctrl_ptr->epsilon * ctrl_ptr->Gamma.transpose() * (ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4) - ctrl_ptr->y);
        ctrl_ptr->m = 1 + ctrl_ptr->eta1.norm() * ctrl_ptr->eta1.norm() + (ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4) - ctrl_ptr->y) * (ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4) - ctrl_ptr->y);
        ctrl_ptr->eta1 = (ctrl_ptr->S - ctrl_ptr->epsilon * ctrl_ptr->Gamma.transpose() * ctrl_ptr->Gamma) * ctrl_ptr->eta1 + ctrl_ptr->G * ctrl_ptr->ua;
        ctrl_ptr->theta.segment(j * 4, 4) = ctrl_ptr->theta.segment(j * 4, 4) - ctrl_ptr->rho * ctrl_ptr->epsilon * ctrl_ptr->epsilon * ctrl_ptr->eta1 * (ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4) - ctrl_ptr->y) / ctrl_ptr->m;
        ctrl_ptr->J(j) = ctrl_ptr->J(j) + ctrl_ptr->beta * (ctrl_ptr->y - ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4)) * (ctrl_ptr->y - ctrl_ptr->Gamma * ctrl_ptr->zetao.segment(j * 4, 4));
        ctrl_ptr->J(ctrl_ptr->sigma) = ctrl_ptr->J(ctrl_ptr->sigma) - ctrl_ptr->h;
        for (int n = 0; n < 9; n++)
        {
            if (int(ctrl_ptr->J(n)) <= int(ctrl_ptr->J(ctrl_ptr->sigma)))
            {
                ctrl_ptr->J(ctrl_ptr->sigma) = ctrl_ptr->J(n);
                ctrl_ptr->sigma = n;
            }
        }
    }

    return ctrl_ptr->u;
}

void controllerFinish(void *ctrl_ptr)
{
    controller_t *ptr = (controller_t *)(ctrl_ptr);
    delete ptr;
}
