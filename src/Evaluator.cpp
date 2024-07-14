#include <iostream>

#include "Evaluator.h"

Evaluator::Evaluator(const ICPConfiguration &config)
{
    eval_rmse_naive = config.evaluate_rmse_naive;
    eval_rmse_nn = config.evaluate_rmse_nn;
    eval_transforms = config.evaluate_transforms;

    // TODO(oleg): set it outside
    output_dir = "./";
}

std::pair<double, double> Evaluator::evalTransforms(Matrix4f transform, Matrix4f ground_truth)
{

    Matrix3f rotation_pred = transform.block(0, 0, 3, 3);
    Vector3f translation_pred = transform.block(0, 3, 3, 1);

    Matrix3f rotation_ground = ground_truth.block(0, 0, 3, 3);
    Vector3f translation_ground = ground_truth.block(0, 3, 3, 1);

    // RMSE for translation
    Vector3f translation_error = translation_pred - translation_ground;
    double translation_rmse = std::sqrt(translation_error.squaredNorm() / translation_error.size());

    // Frobenius norm for rotation
    Matrix3f near_id = Matrix3f::Identity() - rotation_pred * rotation_ground.transpose();
    double rotation_frobenius_norm = near_id.norm();

    return std::make_pair(translation_rmse, rotation_frobenius_norm);
}

void Evaluator::reset()
{
    rotation_vals.clear();
    translation_vals.clear();
}

void Evaluator::write()
{
    // Here we write csvs for each method used into the output_dir
}
