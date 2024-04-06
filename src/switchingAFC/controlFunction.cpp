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
    const double w_star = config["dist_freq"].as<double>() * M_PI * 2;
    const double theta1 = config["theta1"].as<double>();
    const double theta2 = config["theta2"].as<double>();
    const double alpha = config["alpha"].as<double>();
    const double epsilon = config["epsilon"].as<double>();
    const double gamma = config["gamma"].as<double>();
    const double dt = 1 / config["sample_fs"].as<double>();

    SwitchingAFC *ctrl_ptr = new SwitchingAFC(w_star, dt, theta1, theta2, alpha, epsilon, gamma, 0.1);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    logger.init({"t", "y", "w0", "w1", "zeta0", "zeta1", "xi0", "xi1", "eta0", "eta1", "theta0", "theta1", "switch", "reset"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

double controllerCompute(void *ctrl_ptr, double y)
{
    SwitchingAFC *ptr = (SwitchingAFC *)(ctrl_ptr);

    logger.log({ptr->tw, ptr->out, ptr->w[0], ptr->w[1], ptr->w[2], ptr->w[3], ptr->w[4], ptr->w[5], ptr->w[6], ptr->w[7], ptr->theta_hat(0), ptr->theta_hat(1), ptr->switching_signal, ptr->reset_signal});

    ptr->setInput(y);
    return ptr->computeOutput();
}

void controllerFinish(void *ctrl_ptr)
{
    logger.write();
    SwitchingAFC *ptr = (SwitchingAFC *)(ctrl_ptr);
    delete ptr;
}

