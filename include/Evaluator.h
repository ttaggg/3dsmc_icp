#pragma once

#include <string>  // for string
#include <vector>  // for vector
#include "Eigen.h" // for Matrix4f

class ICPConfiguration;
class Match;
class PointCloud;

class Evaluator
{
public:
    Evaluator() {};
    Evaluator(const ICPConfiguration &config);

    std::vector<double> rmse_naive_metric;
    std::vector<double> rmse_nn_metric;
    std::vector<double> rmse_nn_plane_metric;
    std::vector<double> time_metric;
    std::vector<double> rotation_metric;
    std::vector<double> translation_metric;

    bool eval_rmse_naive;
    bool eval_rmse_nn;
    bool eval_rmse_nn_plane;
    bool eval_transforms;
    bool eval_time;

    void reset();
    void write(std::string output_dir,
               std::string experiment_name,
               std::string mesh_name);

    void addMetrics(double elapsedSecs,
                    const PointCloud &source,
                    const PointCloud &target,
                    std::vector<Match> &matches,
                    Matrix4f &estimatedPose,
                    Matrix4f &groundPose);

private:
    void _write_metric(std::vector<double> &metric, std::string metric_path);
    void _evalTransforms(Matrix4f &transform, Matrix4f &ground_truth);
    void _PointToPointComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Match> &match,
        const Matrix4f &transformation);
    void _PointToPlaneComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Match> &match,
        const Matrix4f &transformation);
    void _NaiveRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const Matrix4f &transformation);
};
