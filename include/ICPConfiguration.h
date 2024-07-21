#pragma once

#include <string>          // for basic_string, string
#include <unordered_map>   // for unordered_map
#include <yaml-cpp/yaml.h> // for yaml

struct ConfigItem
{
    std::variant<bool *, double *, float *, int *, std::string *> value;
};

/**
 * ICP Configuration.
 */
class ICPConfiguration
{
public:
    // Task
    bool runShapeICP = false;
    bool runSequenceICP = false;
    // Sampling
    std::string sampling = "FULL";
    double samplingRatio = 1.0;
    // ICP type
    bool useLinearICP = false;
    // ICP objective(s)
    bool usePointToPoint = false;
    double weightPointToPoint = 1.;
    bool usePointToPlane = false;
    double weightPointToPlane = 1.;
    bool useSymmetric = false;
    double weightSymmetric = 1.;
    // Whether to use color information.
    bool useColors = false;
    // Correspondence method (NN / SHOOT)
    std::string correspondenceMethod = "NN";
    // Other settings
    float matchingMaxDistance = 0.0f;
    int nbOfIterations = 0;
    bool visualize = false;
    bool writeMeshes = false;
    bool evaluateRMSENaive = false;
    bool evaluateRMSENearest = false;
    bool evaluateRMSENearestPlane = false;
    bool evaluateTransforms = false;
    bool evaluateTime = false;
    // Experiment
    std::string experimentName = "default";
    std::string dataDir = ".";

    void loadFromYaml(const std::string &filename);
    void show();

private:
    void _loadFromYaml(const std::string &filename);
    void _sanityCheck();
    void _setConfig(std::unordered_map<std::string, ConfigItem> &configMap,
                    YAML::Node config);
    void _showConfig(std::unordered_map<std::string, ConfigItem> &configMap);
};