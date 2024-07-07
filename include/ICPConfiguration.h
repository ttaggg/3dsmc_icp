#pragma once

#include <cassert>
#include <yaml-cpp/yaml.h>

/**
 * enum for correspondence method: nearest neighbor or projective
 */
enum CorrMethod
{
    ANN,
    PROJ,
};

/**
 * ICP Configuration.
 */
struct ICPConfiguration
{
    // Task
    bool runShapeICP = false;
    bool runSequenceICP = false;
    // ICP type
    bool useLinearICP = false;
    // ICP objective(s)
    bool usePointToPoint = false;
    bool usePointToPlane = false;
    bool useSymmetric = false;
    // Correspondence method (ANN / PROJ)
    CorrMethod correspondenceMethod = ANN;
    // Other settings
    float matchingMaxDistance = 0.0f;
    int nbOfIterations = 0;

    void _loadFromYaml(const std::string &filename)
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
        if (config["usePointToPlane"])
        {
            usePointToPlane = config["usePointToPlane"].as<bool>();
        }
        if (config["useSymmetric"])
        {
            useSymmetric = config["useSymmetric"].as<bool>();
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

    void loadFromYaml(const std::string &filename)
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
    }

    void sanityCheck()
    {

        // Check if at least one task is defined.
        assert(runShapeICP || runSequenceICP);

        // Check if the number of objective makes sence.
        if (useLinearICP)
        {
            assert(usePointToPoint + usePointToPlane + useSymmetric == 1);
        }
        else
        {
            assert(usePointToPoint + usePointToPlane + useSymmetric > 0);
        }
    }

    void show()
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
        std::cout << "usePointToPoint: " << usePointToPoint << std::endl;
        std::cout << "usePointToPlane: " << usePointToPlane << std::endl;
        std::cout << "useSymmetric: " << useSymmetric << std::endl;

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
};