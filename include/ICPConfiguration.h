#pragma once

#include <iostream>
#include <cassert>
#include <yaml-cpp/yaml.h>

/**
 * enum for correspondence method: nearest neighbor or projective
 */
enum CorrMethod
{
    NN,
    SHOOT,
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
    CorrMethod correspondenceMethod = NN;
    // Other settings
    float matchingMaxDistance = 0.0f;
    int nbOfIterations = 0;
    bool visualize = false;
    bool evaluate_rmse_naive = false;
    bool evaluate_rmse_nn = false;
    bool evaluate_rmse_nn_plane = false;
    bool evaluate_transforms = false;
    bool evaluate_time = false;
    // Experiment
    std::string experiment_name = "default";
    std::string output_dir = ".";

    void loadFromYaml(const std::string &filename);
    void show();

private:
    void _loadFromYaml(const std::string &filename);
    void _sanityCheck();
};