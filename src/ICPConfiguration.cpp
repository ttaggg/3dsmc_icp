#include "ICPConfiguration.h"
#include <stdlib.h>        // for exit
#include <cassert>         // for assert
#include <exception>       // for exception
#include <iostream>        // for basic_ostream, endl, operator<<
#include <stdexcept>       // for runtime_error
#include <variant>         // for variant
#include <unordered_map>   // for unordered_map
#include <yaml-cpp/yaml.h> // for yaml

std::unordered_map<std::string, ConfigItem> createTaskConfigMap(ICPConfiguration &config)
{
    return {
        {"runShapeICP", {&config.runShapeICP}},
        {"runSequenceICP", {&config.runSequenceICP}},
        {"useLinearICP", {&config.useLinearICP}},
        {"usePointToPoint", {&config.usePointToPoint}},
        {"weightPointToPoint", {&config.weightPointToPoint}},
        {"usePointToPlane", {&config.usePointToPlane}},
        {"weightPointToPlane", {&config.weightPointToPlane}},
        {"useSymmetric", {&config.useSymmetric}},
        {"weightSymmetric", {&config.weightSymmetric}},
    };
}
std::unordered_map<std::string, ConfigItem> createTrainingConfigMap(ICPConfiguration &config)
{
    return {
        {"correspondenceMethod", {&config.correspondenceMethod}},
        {"matchingMaxDistance", {&config.matchingMaxDistance}},
        {"nbOfIterations", {&config.nbOfIterations}},
    };
}
std::unordered_map<std::string, ConfigItem> createEvalConfigMap(ICPConfiguration &config)
{
    return {
        {"visualize", {&config.visualize}},
        {"writeMeshes", {&config.writeMeshes}},
        {"evaluateRMSENaive", {&config.evaluateRMSENaive}},
        {"evaluateRMSENearest", {&config.evaluateRMSENearest}},
        {"evaluateRMSENearestPlane", {&config.evaluateRMSENearestPlane}},
        {"evaluateTransforms", {&config.evaluateTransforms}},
        {"evaluateTime", {&config.evaluateTime}},
    };
}
std::unordered_map<std::string, ConfigItem> createExperimentConfigMap(ICPConfiguration &config)
{
    return {
        {"experimentName", {&config.experimentName}},
        {"dataDir", {&config.dataDir}}};
}

void ICPConfiguration::_setConfig(std::unordered_map<std::string, ConfigItem> &configMap, YAML::Node config)
{
    for (const auto &[key, item] : configMap)
    {
        if (config[key])
        {
            if (auto boolPtr = std::get_if<bool *>(&item.value))
                **boolPtr = config[key].as<bool>();
            else if (auto doublePtr = std::get_if<double *>(&item.value))
                **doublePtr = config[key].as<double>();
            else if (auto floatPtr = std::get_if<float *>(&item.value))
                **floatPtr = config[key].as<float>();
            else if (auto intPtr = std::get_if<int *>(&item.value))
                **intPtr = config[key].as<int>();
            else if (auto stringPtr = std::get_if<std::string *>(&item.value))
                **stringPtr = config[key].as<std::string>();
        }
    }
}

void ICPConfiguration::_loadFromYaml(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    auto taskConfigMap = createTaskConfigMap(*this);
    _setConfig(taskConfigMap, config);
    auto trainingConfigMap = createTrainingConfigMap(*this);
    _setConfig(trainingConfigMap, config);
    auto evalConfigMap = createEvalConfigMap(*this);
    _setConfig(evalConfigMap, config);
    auto experimentConfigMap = createExperimentConfigMap(*this);
    _setConfig(experimentConfigMap, config);
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

void ICPConfiguration::_showConfig(std::unordered_map<std::string, ConfigItem> &configMap)
{

    for (const auto &[key, item] : configMap)
    {
        std::cout << key << ": ";
        if (auto boolPtr = std::get_if<bool *>(&item.value))
            std::cout << (**boolPtr ? "true" : "false");
        else if (auto doublePtr = std::get_if<double *>(&item.value))
            std::cout << **doublePtr;
        else if (auto floatPtr = std::get_if<float *>(&item.value))
            std::cout << **floatPtr;
        else if (auto intPtr = std::get_if<int *>(&item.value))
            std::cout << **intPtr;
        else if (auto stringPtr = std::get_if<std::string *>(&item.value))
            std::cout << **stringPtr;
        std::cout << std::endl;
    }
}

void ICPConfiguration::show()
{

    std::cout << "============================" << std::endl;
    std::cout << "Current config values: " << std::endl;

    std::cout << "==========  Task  ==========" << std::endl;
    auto taskConfigMap = createTaskConfigMap(*this);
    _showConfig(taskConfigMap);

    std::cout << "========  Training  ========" << std::endl;
    auto trainingConfigMap = createTrainingConfigMap(*this);
    _showConfig(trainingConfigMap);

    std::cout << "=======  Evaluation  =======" << std::endl;
    auto evalConfigMap = createEvalConfigMap(*this);
    _showConfig(evalConfigMap);

    std::cout << "=======  Experiment  =======" << std::endl;
    auto experimentConfigMap = createExperimentConfigMap(*this);
    _showConfig(experimentConfigMap);

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
}
