#pragma once

#include <string>

#include "ICPConfiguration.h"
#include "Eigen.h"

class Evaluator
{
public:
    Evaluator() {};
    Evaluator(const ICPConfiguration &config);
    std::pair<double, double> evalTransforms(Matrix4f transform, Matrix4f ground_truth);

    std::vector<double> rmse_vals;
    std::vector<double> rotation_vals;
    std::vector<double> translation_vals;

    bool eval_rmse_naive;
    bool eval_rmse_nn;
    bool eval_transforms;
    std::string output_dir;

    void reset();
    void write();

private:
};
