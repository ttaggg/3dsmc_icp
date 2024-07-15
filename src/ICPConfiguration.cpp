#include "ICPConfiguration.h"

void ICPConfiguration::_loadFromYaml(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    // Task
    if (config["runShapeICP"])
    {
        runShapeICP = config["runShapeICP"].as<bool>();
    }
    if (config["runSequenceICP"])
    {
        runSequenceICP = config["runSequenceICP"].as<bool>();
    }

    // ICP type
    if (config["useLinearICP"])
    {
        useLinearICP = config["useLinearICP"].as<bool>();
    }

    // ICP objective(s).
    if (config["usePointToPoint"])
    {
        usePointToPoint = config["usePointToPoint"].as<bool>();
    }
    if (config["weightPointToPoint"])
    {
        weightPointToPoint = config["weightPointToPoint"].as<double>();
    }
    if (config["usePointToPlane"])
    {
        usePointToPlane = config["usePointToPlane"].as<bool>();
    }
    if (config["weightPointToPlane"])
    {
        weightPointToPlane = config["weightPointToPlane"].as<double>();
    }
    if (config["useSymmetric"])
    {
        useSymmetric = config["useSymmetric"].as<bool>();
    }
    if (config["weightSymmetric"])
    {
        weightSymmetric = config["weightSymmetric"].as<double>();
    }

    // Whether to use color information.
    if (config["useColors"])
    {
        useColors = config["useColors"].as<bool>();
    }

    // Other setting.
    if (config["matchingMaxDistance"])
    {
        matchingMaxDistance = config["matchingMaxDistance"].as<float>();
    }
    if (config["nbOfIterations"])
    {
        nbOfIterations = config["nbOfIterations"].as<int>();
    }

    // Correspondence type.
    if (config["correspondenceMethod"])
    {
        std::string method = config["correspondenceMethod"].as<std::string>();
        if (method == "NN")
        {
            correspondenceMethod = NN;
        }
        else if (method == "SHOOT")
        {
            correspondenceMethod = SHOOT;
        }
        else
        {
            throw std::runtime_error("Unknown correspondence method: " + method);
        }
    }

    if (config["visualize"])
    {
        visualize = config["visualize"].as<bool>();
    }

    if (config["evaluate_rmse_naive"])
    {
        evaluate_rmse_naive = config["evaluate_rmse_naive"].as<bool>();
    }

    if (config["evaluate_rmse_nn"])
    {
        evaluate_rmse_nn = config["evaluate_rmse_nn"].as<bool>();
    }

    if (config["evaluate_rmse_nn_plane"])
    {
        evaluate_rmse_nn_plane = config["evaluate_rmse_nn_plane"].as<bool>();
    }

    if (config["evaluate_transforms"])
    {
        evaluate_transforms = config["evaluate_transforms"].as<bool>();
    }
    if (config["evaluate_time"])
    {
        evaluate_time = config["evaluate_time"].as<bool>();
    }

    if (config["experiment_name"])
    {
        experiment_name = config["experiment_name"].as<std::string>();
    }
    if (config["output_dir"])
    {
        output_dir = config["output_dir"].as<std::string>();
    }
}

void ICPConfiguration::loadFromYaml(const std::string &filename)
{
    try
    {
        _loadFromYaml(filename);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
    _sanityCheck();
}

void ICPConfiguration::show()
{
    std::cout << "============================" << std::endl;
    std::cout << "Current config values: " << std::endl;
    std::cout << std::endl;

    std::cout << "=========== Task ===========" << std::endl;
    if (runShapeICP)
    {
        std::cout << "Shape ICP" << std::endl;
    }
    else if (runSequenceICP)
    {
        std::cout << "Room Sequence ICP" << std::endl;
    }

    std::cout << "=========== Type ===========" << std::endl;
    if (useLinearICP)
    {
        std::cout << "Linearized ICP" << std::endl;
    }
    else
    {
        std::cout << "Nonlinear ICP" << std::endl;
    }

    std::cout << "======== Objectives ========" << std::endl;
    if (usePointToPoint)
    {
        std::cout << "usePointToPoint with weight: " << weightPointToPoint << std::endl;
    }
    if (usePointToPlane)
    {
        std::cout << "usePointToPlane with weight: " << weightPointToPlane << std::endl;
    }
    if (useSymmetric)
    {
        std::cout << "useSymmetric with weight: " << weightSymmetric << std::endl;
    }

    std::cout << "====== Correspondence ======" << std::endl;
    if (correspondenceMethod == NN)
    {
        std::cout << "correspondenceMethod: Nearest Neighbors" << std::endl;
    }
    else if (correspondenceMethod == SHOOT)
    {
        std::cout << "correspondenceMethod: Normal Shoot" << std::endl;
    }
    std::cout << "useColors: " << useColors << std::endl;

    std::cout << "====== Other settings ======" << std::endl;
    std::cout << "matchingMaxDistance: " << matchingMaxDistance << std::endl;
    std::cout << "nbOfIterations: " << nbOfIterations << std::endl;
    std::cout << "Visualization: " << visualize << std::endl;
    std::cout << "Evaluate RMSE naive: " << evaluate_rmse_naive << std::endl;
    std::cout << "Evaluate RMSE NN: " << evaluate_rmse_nn << std::endl;
    std::cout << "Evaluate transforms: " << evaluate_transforms << std::endl;

    std::cout << "============================" << std::endl;

    std::cout << std::endl;
}

void ICPConfiguration::_sanityCheck()
{

    assert(runShapeICP || runSequenceICP && "At least one task should be set to true.");

    if (useLinearICP)
    {
        assert(usePointToPoint + usePointToPlane + useSymmetric == 1 && "For linearized ICP we do not combine several methods.");
        assert(weightPointToPoint == weightPointToPlane == weightSymmetric == 1 && "Weight should not be set for a linearized ICP.");
    }
    else
    {
        assert(usePointToPoint + usePointToPlane + useSymmetric > 0 && "At least one objective should be set to true.");
        assert(weightPointToPoint + weightPointToPlane + weightSymmetric > 0 && "At least one objective weight should be nonzero.");
        assert(0 <= weightPointToPoint && weightPointToPoint <= 1 && "weightPointToPoint should be in [0, 1].");
        assert(0 <= weightPointToPlane && weightPointToPlane <= 1 && "weightPointToPlane should be in [0, 1].");
        assert(0 <= weightSymmetric && weightSymmetric <= 1 && "weightSymmetric should be in [0, 1].");

        // temp restriction due to a uncertainty in symmetric icp implementation
        assert(usePointToPoint + usePointToPlane + useSymmetric == 1 && "For linearized ICP we do not combine several methods.");
    }

    if (useColors)
    {
        assert(correspondenceMethod != SHOOT && "Color is not supported for normal shoot.");
    }
}
