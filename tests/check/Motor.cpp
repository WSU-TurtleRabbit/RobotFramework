#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <Telemetry/Telemetry.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    double velocity = 0.1;
    int duration = 2;
    bool log_telemetry = false;
    std::vector<int> selected_motors;

    // Parse positional arguments first
    if (argc > 1) velocity = std::stod(argv[1]);
    if (argc > 2) duration = std::stoi(argv[2]);

    // Parse remaining optional flags
    for (int i = 3; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-l" || arg == "--log")
        {
            log_telemetry = true;
        }
        else if ((arg == "-m" || arg == "--motors") && i + 1 < argc)
        {
            std::string motors_str = argv[++i];
            size_t pos = 0;
            while ((pos = motors_str.find(',')) != std::string::npos)
            {
                selected_motors.push_back(std::stoi(motors_str.substr(0, pos)));
                motors_str.erase(0, pos + 1);
            }
            if (!motors_str.empty())
                selected_motors.push_back(std::stoi(motors_str));
        }
        else
        {
            std::cerr << "Unknown flag or argument: " << arg << std::endl;
        }
    }

    Telemetry telemetry;

    // Load velocity limit from Safety.yaml
    double w_limit = 0.1; // default fallback
    try
    {
        YAML::Node safety_config = YAML::LoadFile("config/Safety.yaml");
        if (safety_config["velocityLimit"] && safety_config["velocityLimit"]["wLimit"])
        {
            w_limit = safety_config["velocityLimit"]["wLimit"].as<double>();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Warning: Could not load Safety.yaml: " << e.what() << std::endl;
    }

    // Cap velocity to wLimit
    if (velocity > w_limit)
    {
        std::cerr << "Velocity " << velocity << " m/s exceeds wLimit " << w_limit << " m/s. Capping to " << w_limit << " m/s." << std::endl;
        velocity = w_limit;
    }

    // Load motor mapping from YAML
    std::map<int, double> velocity_map;
    try
    {
        YAML::Node config = YAML::LoadFile("config/Motor.yaml");
        YAML::Node motor_map = config["motorMap"];

        for (const auto &pair : motor_map)
        {
            int motor_id = pair.first.as<int>();
            // Include only selected motors if specified
            if (!selected_motors.empty() &&
                std::find(selected_motors.begin(), selected_motors.end(), motor_id) == selected_motors.end())
                continue;

            velocity_map[motor_id] = velocity;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Motor.yaml: " << e.what() << std::endl;
        // fallback to all 4 motors
        for (int i = 1; i <= 4; ++i)
        {
            if (selected_motors.empty() || std::find(selected_motors.begin(), selected_motors.end(), i) != selected_motors.end())
                velocity_map[i] = velocity;
        }
    }

    // Initialize logger with a component for each motor
    Logger logger;
    std::vector<std::string> motor_components;
    for (const auto &pair : velocity_map)
    {
        motor_components.push_back("motor_" + std::to_string(pair.first));
    }
    if (!logger.initialize(motor_components))
    {
        std::cerr << "Failed to initialize logger" << std::endl;
        return 1;
    }
    for (const auto &comp : motor_components)
    {
        logger.log(comp, "Motor test started", LogLevel::INFO);
    }

    std::cout << "Running motors at " << velocity << " m/s for " << duration << " seconds..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    while (true)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - start_time)
                           .count();

        if (elapsed >= duration)
            break;

        auto telemetry_data = telemetry.cycle(velocity_map);

        // Log each motor's telemetry to its own file
        for (const auto &pair : telemetry_data)
        {
            std::string component = "motor_" + std::to_string(pair.first);
            std::map<std::string, double> data = {{"value", pair.second}};
            logger.log(component, data);
        }

        if (log_telemetry)
        {
            for (const auto &pair : telemetry_data)
                std::cout << "Motor " << pair.first << ": " << pair.second << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (auto &pair : velocity_map)
        pair.second = 0.0;
    telemetry.cycle(velocity_map);

    for (const auto &comp : motor_components)
    {
        logger.log(comp, "Motor test completed", LogLevel::DONE);
    }
    logger.closeAll();

    std::cout << "Done!" << std::endl;
    return 0;
}

