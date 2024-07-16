#include "Evaluator.h"
#include <cmath>              // for sqrt
#include <fstream>            // for basic_ofstream
#include <utility>            // for move
#include "Eigen.h"            // for Eigen
#include "ICPConfiguration.h" // for ICPConfiguration
#include "PointCloud.h"       // for PointCloud
#include "Search.h"           // for Match
#include "Utils.h"            // for formatString

namespace fs = std::filesystem;

Evaluator::Evaluator(const ICPConfiguration &config)
{
    evaluateRMSENaive = config.evaluateRMSENaive;
    evaluateRMSENearest = config.evaluateRMSENearest;
    evaluateRMSENearestPlane = config.evaluateRMSENearestPlane;
    evaluateTransforms = config.evaluateTransforms;
    evaluateTime = config.evaluateTime;
}

void Evaluator::reset()
{
    rmseNaiveMetric.clear();
    rmseNearestMetric.clear();
    rmseNearestPlaneMetric.clear();
    timeMetric.clear();
    rotationMetric.clear();
    translationMetric.clear();
}

void Evaluator::addMetrics(double elapsedSecs,
                           const PointCloud &source,
                           const PointCloud &target,
                           std::vector<Match> &matches,
                           Matrix4f &estimatedPose,
                           Matrix4f &groundPose)
{
    if (evaluateRMSENaive)
        _NaiveRMSE(
            source,
            target,
            estimatedPose);
    if (evaluateRMSENearest)
        _PointToPointComputeRMSE(source,
                                 target,
                                 matches,
                                 estimatedPose);
    if (evaluateRMSENearestPlane)
        _PointToPlaneComputeRMSE(source,
                                 target,
                                 matches,
                                 estimatedPose);
    if (evaluateTransforms)
        _evalTransforms(estimatedPose, groundPose);
    if (evaluateTime)
        timeMetric.push_back(elapsedSecs);
}

void Evaluator::_evalTransforms(Matrix4f &transform, Matrix4f &groundTruth)
{
    Matrix3f rotation_pred = transform.block(0, 0, 3, 3);
    Vector3f translation_pred = transform.block(0, 3, 3, 1);

    Matrix3f rotation_ground = groundTruth.block(0, 0, 3, 3);
    Vector3f translation_ground = groundTruth.block(0, 3, 3, 1);

    // RMSE for translation
    Vector3f translation_error = translation_pred - translation_ground;
    double translation_rmse = std::sqrt(translation_error.squaredNorm() / translation_error.size());

    // Frobenius norm for rotation
    Matrix3f near_id = Matrix3f::Identity() - rotation_pred * rotation_ground.transpose();
    double rotation_frobenius_norm = near_id.norm();

    translationMetric.push_back(translation_rmse);
    rotationMetric.push_back(rotation_frobenius_norm);
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

        auto pt_trans = (transformation * Vector4f(sp(0), sp(1), sp(2), 1.0)).block<3, 1>(0, 0);
        err += (pt_trans - tp).squaredNorm();
    }
    rmseNaiveMetric.push_back(std::sqrt(err / (double)source_p.size()));
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
            auto pt_trans = (transformation * Vector4f(pt(0), pt(1), pt(2), 1.0)).block<3, 1>(0, 0);
            err += (pt_trans - target_p[m.idx]).squaredNorm();
        }
        else
        {
            unmatched++;
        }
        i++;
    }
    rmseNearestMetric.push_back(std::sqrt(err / (double)(match.size() - unmatched)));
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
            Vector3f pt_trans = (transformation * Vector4f(pt(0), pt(1), pt(2), 1.0)).block<3, 1>(0, 0);
            r = (pt_trans - target_p[m.idx]).dot(target_n[m.idx]);
            err += r * r;
        }
        else
        {
            unmatched++;
        }
        i++;
    }
    rmseNearestPlaneMetric.push_back(std::sqrt(err / (double)(match.size() - unmatched)));
}

void Evaluator::write(fs::path outputDir)
{
    // Here we write csvs for each method used into the outputDir.
    fs::create_directories(outputDir);

    if (evaluateRMSENaive)
    {
        std::string metricPath = outputDir / fs::path{"rmse_naive_metric.txt"};
        _write_metric(rmseNaiveMetric, metricPath);
    }

    if (evaluateRMSENearest)
    {
        std::string metricPath = outputDir / fs::path{"rmse_nn_metric.txt"};
        _write_metric(rmseNearestMetric, metricPath);
    }

    if (evaluateRMSENearestPlane)
    {
        std::string metricPath = outputDir / fs::path{"rmse_nn_plane_metric.txt"};
        _write_metric(rmseNearestPlaneMetric, metricPath);
    }

    if (evaluateTransforms)
    {
        std::string metricPathR = outputDir / fs::path{"eval_r.txt"};
        _write_metric(rotationMetric, metricPathR);

        std::string metricPathT = outputDir / fs::path{"eval_t.txt"};
        _write_metric(translationMetric, metricPathT);
    }

    if (evaluateTime)
    {
        std::string metricPath = outputDir / fs::path{"time_metric.txt"};
        _write_metric(timeMetric, metricPath);
    }
}

void Evaluator::_write_metric(std::vector<double> &metric, std::string metricPath)
{
    std::ofstream file;
    file.open(metricPath);
    for (int i = 0; i < metric.size(); i++)
    {
        file << i + 1 << "," << metric[i] << std::endl;
    }
    file.close();
}
