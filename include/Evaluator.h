#pragma once

#include <filesystem> // for fs
#include <string>     // for string
#include <vector>     // for vector
#include "Eigen.h"    // for Matrix4f

namespace fs = std::filesystem;

class ICPConfiguration;
class Match;
class PointCloud;

class Evaluator
{
public:
    Evaluator() {};
    Evaluator(const ICPConfiguration &config);

    std::vector<double> rmseNaiveMetric;
    std::vector<double> rmseNearestMetric;
    std::vector<double> rmseNearestPlaneMetric;
    std::vector<double> timeMatchMetric;
    std::vector<double> timeOptiMetric;
    std::vector<double> rotationMetric;
    std::vector<double> translationMetric;

    bool evaluateRMSENaive;
    bool evaluateRMSENearest;
    bool evaluateRMSENearestPlane;
    bool evaluateTransforms;
    bool evaluateTime;

    void reset();
    void write(fs::path outputDir);

    void addMetrics(double elapsedSecsMatch,
                    double elapsedSecsOpti,
                    const PointCloud &source,
                    const PointCloud &target,
                    std::vector<Match> &matches,
                    Matrix4f &estimatedPose,
                    Matrix4f &groundPose);

private:
    void _write_metric(std::vector<double> &metric, std::string metricPath);
    void _evalTransforms(Matrix4f &transform, Matrix4f &groundTruth);
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
