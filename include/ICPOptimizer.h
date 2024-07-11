#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <flann/flann.hpp>

#include "SimpleMesh.h"
#include "Search.h"
#include "PointCloud.h"
#include "Constraints.h"
#include "ProcrustesAligner.h"

/**
 * enum for correspondence method: nearest neighbor or projective
 */
enum CorrMethod
{
    ANN,
    PROJ,
};

/**
 * ICP optimizer - Abstract Base Class
 */
class ICPOptimizer
{
public:
    ICPOptimizer();
    void setMatchingMaxDistance(float maxDistance);
    void setCorrespondenceMethod(CorrMethod method);
    void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints);
    void setNbOfIterations(unsigned nIterations);

    virtual ~ICPOptimizer() = default;
    virtual void estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose) = 0;
    std::vector<Eigen::Vector2i> FindCorrespondence(const PointCloud &source, const PointCloud &target);
    double PointToPointComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);
    double PointToPlaneComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);
protected:
    bool m_bUsePointToPlaneConstraints;
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
    virtual void estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose) override;
    std::vector<Eigen::Vector2i> FindCorrespondence(const PointCloud &source, const PointCloud &target);
    double PointToPointComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);
    double PointToPlaneComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);

private:
    void configureSolver(ceres::Solver::Options &options);
    void prepareConstraints(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints, const std::vector<Vector3f> &targetNormals, const std::vector<Match> matches, const PoseIncrement<double> &poseIncrement, ceres::Problem &problem) const;
};

/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer
{
public:
    LinearICPOptimizer();
    virtual void estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose) override;
    std::vector<Eigen::Vector2i> FindCorrespondence(const PointCloud &source, const PointCloud &target);
    double PointToPointComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);
    double PointToPlaneComputeRMSE(
        const PointCloud &source,
        const PointCloud &target,
        const std::vector<Eigen::Vector2i> &corres,
        const Eigen::Matrix4f &transformation);

private:
    Matrix4f estimatePosePointToPoint(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints);
    Matrix4f estimatePosePointToPlane(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints, const std::vector<Vector3f> &targetNormals);
};
