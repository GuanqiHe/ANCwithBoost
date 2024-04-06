#include "controlFunction.h"

data_logger software_logger;

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
    double w_star = config["dist_freq"].as<double>() * M_PI * 2;
    double g1 = config["g1"].as<double>();
    double g2 = config["g2"].as<double>();
    double dt = 1 / config["sample_fs"].as<double>();

    MTController *ctrl_ptr = new MTController(w_star, g1, g2, dt);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>() + 1;

    software_logger.init({"t", "y", "w0", "w1"}, sample_len, log_folder + '/' + config["controller_log_path"].as<std::string>());

    return (void *)(ctrl_ptr);
}

double controllerCompute(void *ctrl_ptr, double y)
{
    MTController *ptr = (MTController *)(ctrl_ptr);

    software_logger.log({ptr->tw, ptr->out, ptr->w[0], ptr->w[1]});

    ptr->setInput(y);
    return ptr->computeOutput();
}

void controllerFinish(void *ctrl_ptr)
{
    software_logger.write();
    MTController *ptr = (MTController *)(ctrl_ptr);
    delete ptr;
}
