#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>      // for Solver
#include <memory>             // for unique_ptr
#include <vector>             // for vector
#include "Eigen.h"            // for vector
#include "ICPConfiguration.h" // for CorrMethod
#include "PointCloud.h"       // for PointCloud
#include "Search.h"           // for Match (ptr only), Search

class Evaluator;
template <typename T>
class PoseIncrement;

/**
 * ICP optimizer - Abstract Base Class
 */
class ICPOptimizer
{
public:
    ICPOptimizer();
    void setMatchingMaxDistance(float maxDistance);
    void setColorGamma(float colorGamma);
    void setCorrespondenceMethod(CorrMethod method, bool useColor);
    void usePointToPointConstraints(bool bUsePointToPointConstraints, double weightPointToPointConstraints);
    void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints, double weightPointToPlaneConstraints);
    void useSymmetricConstraints(bool bUseSymmetricConstraints, double weightSymmetricConstraints);
    void setNbOfIterations(unsigned nIterations);

    virtual ~ICPOptimizer() = default;
    virtual void estimatePose(const PointCloud &source,
                              const PointCloud &target,
                              Matrix4f &initialPose,
                              Matrix4f &groundPose) = 0;

    void setEvaluator(Evaluator *evaluator);
    Evaluator *evaluator;

protected:
    bool m_bUsePointToPointConstraints;
    bool m_weightPointToPointConstraints;
    bool m_bUsePointToPlaneConstraints;
    bool m_weightPointToPlaneConstraints;
    bool m_bUseSymmetricConstraints;
    bool m_weightSymmetricConstraints;
    bool m_evaluate = false;
    double m_colorGamma = 0;
    unsigned m_nIterations;
    std::unique_ptr<Search> m_corrAlgo;
    std::vector<Vector3f> transformPoints(const std::vector<Vector3f> &sourcePoints, const Matrix4f &pose);
    std::vector<Vector3f> transformNormals(const std::vector<Vector3f> &sourceNormals, const Matrix4f &pose);
    void pruneCorrespondences(const std::vector<Vector3f> &sourceNormals, const std::vector<Vector3f> &targetNormals, std::vector<Match> &matches);
};

/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer
{
public:
    CeresICPOptimizer();
    virtual void estimatePose(const PointCloud &source,
                              const PointCloud &target,
                              Matrix4f &initialPose,
                              Matrix4f &groundPose) override;

private:
    void configureSolver(ceres::Solver::Options &options);
    void prepareConstraints(const std::vector<Vector3f> &sourcePoints,
                            const std::vector<Vector3f> &targetPoints,
                            const std::vector<Vector3f> &sourceColors,
                            const std::vector<Vector3f> &targetColors,
                            const std::vector<Vector3f> &sourceNormals,
                            const std::vector<Vector3f> &targetNormals,
                            const std::vector<Match> matches,
                            const PoseIncrement<double> &poseIncrement,
                            ceres::Problem &problem) const;
};

/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer
{
public:
    LinearICPOptimizer();
    virtual void estimatePose(const PointCloud &source,
                              const PointCloud &target,
                              Matrix4f &initialPose,
                              Matrix4f &groundPose) override;

private:
    Matrix4f estimatePosePointToPoint(const std::vector<Vector3f> &sourcePoints,
                                      const std::vector<Vector3f> &targetPoints);
    Matrix4f estimatePosePointToPlane(const std::vector<Vector3f> &sourcePoints,
                                      const std::vector<Vector3f> &targetPoints,
                                      const std::vector<Vector3f> &targetNormals);
    Matrix4f estimatePoseSymmetric(const std::vector<Vector3f> &sourcePoints,
                                   const std::vector<Vector3f> &targetPoints,
                                   const std::vector<Vector3f> &sourceNormals,
                                   const std::vector<Vector3f> &targetNormals);
};
