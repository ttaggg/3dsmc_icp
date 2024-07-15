#include <iostream>
#include "Evaluator.h"
#include "Utils.h"

Evaluator::Evaluator(const ICPConfiguration &config)
{
    eval_rmse_naive = config.evaluate_rmse_naive;
    eval_rmse_nn = config.evaluate_rmse_nn;
    eval_rmse_nn_plane = config.evaluate_rmse_nn_plane;
    eval_transforms = config.evaluate_transforms;
    eval_time = config.evaluate_time;
}

void Evaluator::reset()
{
    rmse_naive_metric.clear();
    rmse_nn_metric.clear();
    rmse_nn_plane_metric.clear();
    time_metric.clear();
    rotation_metric.clear();
    translation_metric.clear();
}

void Evaluator::addMetrics(double elapsedSecs,
                           const PointCloud &source,
                           const PointCloud &target,
                           std::vector<Match> &matches,
                           Matrix4f &estimatedPose,
                           Matrix4f &groundPose)
{
    if (eval_rmse_naive)
        _NaiveRMSE(
            source,
            target,
            estimatedPose);
    if (eval_rmse_nn)
        _PointToPointComputeRMSE(source,
                                 target,
                                 matches,
                                 estimatedPose);
    if (eval_rmse_nn_plane)
        _PointToPlaneComputeRMSE(source,
                                 target,
                                 matches,
                                 estimatedPose);
    if (eval_transforms)
        _evalTransforms(estimatedPose, groundPose);
    if (eval_time)
        time_metric.push_back(elapsedSecs);
}

void Evaluator::_evalTransforms(Matrix4f &transform, Matrix4f &ground_truth)
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

    translation_metric.push_back(translation_rmse);
    rotation_metric.push_back(rotation_frobenius_norm);
}

void Evaluator::_NaiveRMSE(
    const PointCloud &source,
    const PointCloud &target,
    const Matrix4f &transformation)
{

    // For some meshes we know exact correspondence, skip matches.

    const auto source_p = source.getPoints();
    const auto target_p = target.getPoints();

    double err = 0.0;
    for (int i = 0; i < source_p.size(); i++)
    {
        const auto &sp = source_p[i];
        const auto &tp = target_p[i];

        auto pt_trans = (transformation * Eigen::Vector4f(sp(0), sp(1), sp(2), 1.0)).block<3, 1>(0, 0);
        err += (pt_trans - tp).squaredNorm();
    }
    rmse_naive_metric.push_back(std::sqrt(err / (double)source_p.size()));
}

void Evaluator::_PointToPointComputeRMSE(
    const PointCloud &source,
    const PointCloud &target,
    const std::vector<Match> &match,
    const Matrix4f &transformation)
{

    const auto source_p = source.getPoints();
    const auto target_p = target.getPoints();

    if (match.empty())
        return;

    double err = 0.0;
    int unmatched = 0;
    for (int i = 0; i < match.size(); i++)
    {
        const auto &m = match[i];
        const auto &pt = source_p[i];

        if (m.idx != -1)
        {
            auto pt_trans = (transformation * Eigen::Vector4f(pt(0), pt(1), pt(2), 1.0)).block<3, 1>(0, 0);
            err += (pt_trans - target_p[m.idx]).squaredNorm();
        }
        else
        {
            unmatched++;
        }
        i++;
    }
    rmse_nn_metric.push_back(std::sqrt(err / (double)(match.size() - unmatched)));
}

void Evaluator::_PointToPlaneComputeRMSE(
    const PointCloud &source,
    const PointCloud &target,
    const std::vector<Match> &match,
    const Matrix4f &transformation)
{
    const auto source_p = source.getPoints();
    const auto target_p = target.getPoints();
    const auto target_n = target.getNormals();

    if (match.empty())
        return;

    double err = 0.0, r;
    int i = 0;
    int unmatched = 0;
    for (int i = 0; i < match.size(); i++)
    {
        const auto &m = match[i];
        const auto &pt = source_p[i];
        if (m.idx != -1)
        {
            Eigen::Vector3f pt_trans = (transformation * Eigen::Vector4f(pt(0), pt(1), pt(2), 1.0)).block<3, 1>(0, 0);
            r = (pt_trans - target_p[m.idx]).dot(target_n[m.idx]);
            err += r * r;
        }
        else
        {
            unmatched++;
        }
        i++;
    }
    rmse_nn_plane_metric.push_back(std::sqrt(err / (double)(match.size() - unmatched)));
}

void Evaluator::write(std::string output_dir,
                      std::string experiment_name,
                      std::string mesh_name)
{
    // Here we write csvs for each method used into the output_dir

    if (eval_rmse_naive)
    {
        std::string metric_path = formatString({output_dir, "/",
                                                experiment_name, "_", mesh_name, "_rmse_naive_metric.txt"});
        _write_metric(rmse_naive_metric, metric_path);
    }

    if (eval_rmse_nn)
    {
        std::string metric_path = formatString({output_dir, "/",
                                                experiment_name, "_", mesh_name, "_rmse_nn_metric.txt"});
        _write_metric(rmse_nn_metric, metric_path);
    }

    if (eval_rmse_nn_plane)
    {
        std::string metric_path = formatString({output_dir, "/",
                                                experiment_name, "_", mesh_name, "_rmse_nn_plane_metric.txt"});
        _write_metric(rmse_nn_plane_metric, metric_path);
    }

    if (eval_transforms)
    {
        std::string metric_path_r = formatString({output_dir, "/",
                                                  experiment_name, "_", mesh_name, "_eval_r.txt"});
        _write_metric(rotation_metric, metric_path_r);
        std::string metric_path_t = formatString({output_dir, "/",
                                                  experiment_name, "_", mesh_name, "_eval_t.txt"});
        _write_metric(translation_metric, metric_path_t);
    }

    if (eval_time)
    {
        std::string metric_path = formatString({output_dir, "/",
                                                experiment_name, "_", mesh_name, "_time_metric.txt"});
        _write_metric(time_metric, metric_path);
    }
}

void Evaluator::_write_metric(std::vector<double> &metric, std::string metric_path)
{
    std::ofstream file;
    file.open(metric_path);
    for (int i = 0; i < metric.size(); i++)
    {
        file << i + 1 << "," << metric[i] << std::endl;
    }
    file.close();
}
