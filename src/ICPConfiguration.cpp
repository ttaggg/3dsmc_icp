#include <iostream>
#include <cassert>
#include <yaml-cpp/yaml.h>

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
    if (config["useColor"])
    {
        useColor = config["useColor"].as<bool>();
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
        if (method == "ANN")
        {
            correspondenceMethod = ANN;
        }
        else if (method == "PROJ")
        {
            correspondenceMethod = PROJ;
        }
        else
        {
            throw std::runtime_error("Unknown correspondence method: " + method);
        }
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
        std::cout << "Bunny Shape ICP" << std::endl;
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
    if (correspondenceMethod == ANN)
    {
        std::cout << "correspondenceMethod: ANN" << std::endl;
    }
    else if (correspondenceMethod == PROJ)
    {
        std::cout << "correspondenceMethod: PROJ" << std::endl;
    }

    std::cout << "====== Other settings ======" << std::endl;
    std::cout << "matchingMaxDistance: " << matchingMaxDistance << std::endl;
    std::cout << "nbOfIterations: " << nbOfIterations << std::endl;

    std::cout << "============================" << std::endl;

    std::cout << std::endl;
}

void ICPConfiguration::_sanityCheck()
{

    // Check if at least one task is defined.
    assert(runShapeICP || runSequenceICP);

    // Check if the number of objective makes sence.
    if (useLinearICP)
    {
        // For linearized we do not combine several methods now.
        assert(usePointToPoint + usePointToPlane + useSymmetric == 1);
        // Weight are defaults (1.) for linearized ICP.
        assert(weightPointToPoint == weightPointToPlane == weightSymmetric == 1);
    }
    else
    {
        assert(usePointToPoint + usePointToPlane + useSymmetric > 0);
        assert(weightPointToPoint + weightPointToPlane + weightSymmetric > 0);
        assert(0 <= weightPointToPoint && weightPointToPoint <= 1);
        assert(0 <= weightPointToPlane && weightPointToPlane <= 1);
        assert(0 <= weightSymmetric && weightSymmetric <= 1);
    }

    if (runShapeICP)
    {
        // Color makes no sense for a bunny example.
        assert(!useColor);
    }
}