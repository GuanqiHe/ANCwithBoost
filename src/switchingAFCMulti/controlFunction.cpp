#include "controlFunction.h"
#include "logger/data_logging.hpp"

data_logger logger;

void *controllerInit(int argc, char *argv[])
{
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

    const std::string log_folder = config["log_folder"].as<std::string>();
    const std::string controller_log_path = log_folder + '/' + config["controller_log_path"].as<std::string>();
    std::vector<double> w_star = config["algo_dist_freq"].as<std::vector<double>>();
    for (auto w = w_star.begin(); w != w_star.end(); w++)
    {
        *w = *w * M_PI * 2;
    }
    std::vector<double> theta_list = config["theta"].as<std::vector<double>>();
    std::vector<Eigen::Vector2d> theta_init;
    for (auto i = theta_list.begin(); i != theta_list.end();)
    {
        Eigen::Vector2d p;
        p(0) = *i;
        i++;
        p(1) = *i;
        i++;
        theta_init.emplace_back(p);
    }
    const double alpha = config["alpha"].as<double>();
    const double epsilon = config["epsilon"].as<double>();
    const double gamma = config["gamma"].as<double>();
    const double h = config["h"].as<double>();
    const double dt = 1 / config["sample_fs"].as<double>();

    SwitchingAFC *ctrl_ptr = new SwitchingAFC(w_star.size(), w_star, dt, theta_init, alpha, epsilon, gamma, h);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    logger.init({"t", "y", "w0", "w1", "zeta0", "zeta1", "xi0", "xi1", "eta00", "eta01", "eta10", "eta11", "theta00", "theta01", "theta10", "theta11", "reset"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

double controllerCompute(void *ctrl_ptr, double y)
{
    SwitchingAFC *ptr = (SwitchingAFC *)(ctrl_ptr);

    logger.log({ptr->tw, ptr->out, ptr->w[0], ptr->w[2], ptr->w[4], ptr->w[6], ptr->w[8], ptr->w[10], ptr->w[12], ptr->w[13], ptr->w[14], ptr->w[15], ptr->subController[0].theta_hat(0), ptr->subController[0].theta_hat(1), ptr->subController[1].theta_hat(0), ptr->subController[1].theta_hat(1), ptr->reset_signal});

    ptr->setInput(y);
    return ptr->computeOutput();
}

void controllerFinish(void *ctrl_ptr)
{
    logger.write();
    SwitchingAFC *ptr = (SwitchingAFC *)(ctrl_ptr);
    delete ptr;
}
