#include "ICPOptimizer.h"
#include <iostream>            // for basic_o...
#include <math.h>              // for M_PI
#include <stdlib.h>            // for exit
#include <time.h>              // for clock
#include <algorithm>           // for fill_n
#include <utility>             // for move, swap
#include "Constraints.h"       // for PoseInc...
#include "Eigen.h"             // for Eigen
#include "Evaluator.h"         // for Evaluator
#include "ProcrustesAligner.h" // for Procrus...

/**
 * ICP optimizer - Abstract Base Class
 */

ICPOptimizer::ICPOptimizer() : m_bUsePointToPlaneConstraints{false},
                               m_nIterations{20}
{
}

void ICPOptimizer::setMatchingMaxDistance(float maxDistance)
{
    m_corrAlgo->setMatchingMaxDistance(maxDistance);
}

void ICPOptimizer::setEvaluator(Evaluator *evaluator_)
{
    evaluator = evaluator_;
    m_evaluate = true;
}

void ICPOptimizer::setCorrespondenceMethod(std::string method)
{
    if (method == "NN")
    {
        m_corrAlgo = std::make_unique<NearestNeighborSearch>();
    }
    else if (method == "NN_COLORS")
    {
        m_corrAlgo = std::make_unique<NearestNeighborSearchWithColors>();
    }
    else if (method == "SHOOT")
    {
        m_corrAlgo = std::make_unique<NormalShootCorrespondence>();
    }
    else if (method == "SHOOT_COLORS")
    {
        m_corrAlgo = std::make_unique<NormalShootCorrespondenceWithColors>();
    }
    else
    {
        throw std::runtime_error("Unknown correspondence method: " + method);
    }
}

void ICPOptimizer::usePointToPointConstraints(bool bUsePointToPointConstraints,
                                              double weightPointToPointConstraints)
{
    m_bUsePointToPointConstraints = bUsePointToPointConstraints;
    m_weightPointToPointConstraints = weightPointToPointConstraints;
}

void ICPOptimizer::usePointToPlaneConstraints(bool bUsePointToPlaneConstraints,
                                              double weightPointToPlaneConstraints)
{
    m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
    m_weightPointToPlaneConstraints = weightPointToPlaneConstraints;
}

void ICPOptimizer::useSymmetricConstraints(bool bUseSymmetricConstraints,
                                           double weightSymmetricConstraints)
{
    m_bUseSymmetricConstraints = bUseSymmetricConstraints;
    m_weightSymmetricConstraints = weightSymmetricConstraints;
}

void ICPOptimizer::setNbOfIterations(unsigned nIterations)
{
    m_nIterations = nIterations;
}

std::vector<Vector3f> ICPOptimizer::transformPoints(const std::vector<Vector3f> &sourcePoints, const Matrix4f &pose)
{
    std::vector<Vector3f> transformedPoints;
    transformedPoints.reserve(sourcePoints.size());

    const auto rotation = pose.block(0, 0, 3, 3);
    const auto translation = pose.block(0, 3, 3, 1);

    for (const auto &point : sourcePoints)
    {
        transformedPoints.push_back(rotation * point + translation);
    }

    return transformedPoints;
}

std::vector<Vector3f> ICPOptimizer::transformNormals(const std::vector<Vector3f> &sourceNormals, const Matrix4f &pose)
{
    std::vector<Vector3f> transformedNormals;
    transformedNormals.reserve(sourceNormals.size());

    const auto rotation = pose.block(0, 0, 3, 3);

    for (const auto &normal : sourceNormals)
    {
        transformedNormals.push_back(rotation.inverse().transpose() * normal);
    }

    return transformedNormals;
}

void ICPOptimizer::pruneCorrespondences(const std::vector<Vector3f> &sourceNormals, const std::vector<Vector3f> &targetNormals, std::vector<Match> &matches)
{
    const unsigned nPoints = sourceNormals.size();

    for (unsigned i = 0; i < nPoints; i++)
    {
        Match &match = matches.at(i);
        if (match.idx >= 0)
        {
            const auto &sourceNormal = sourceNormals[i];
            const auto &targetNormal = targetNormals[match.idx];
            auto angle_rad = (sourceNormal.transpose() * targetNormal).array().acos().value();
            auto angle = 180.0 * angle_rad / M_PI;
            if (angle > 60 || angle < -60)
            {
                matches[i].idx = -1;
            }
        }
    }
}

/**
 * ICP optimizer - using Ceres for optimization.
 */

CeresICPOptimizer::CeresICPOptimizer() {}

void CeresICPOptimizer::estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose, Matrix4f &groundPose)
{
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    m_corrAlgo->buildIndex(target.getPoints(), &target.getColors(), &target.getNormals());

    // The initial estimate can be given as an argument.
    Matrix4f estimatedPose = initialPose;
    std::vector<std::vector<double>> tmp;

    // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
    // the rotation angle) and 3 parameters for the translation vector.
    double incrementArray[6];
    auto poseIncrement = PoseIncrement<double>(incrementArray);
    poseIncrement.setZero();

    for (int i = 0; i < m_nIterations; ++i)
    {
        std::cout << "======================" << std::endl;
        std::cout << "Starting iteration: " << i + 1 << std::endl;
        std::cout << std::endl;

        // Compute the matches.
        clock_t beginMatch = clock();
        auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
        auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

        std::vector<Match> matches = m_corrAlgo->queryMatches(transformedPoints,
                                                              &source.getColors(),
                                                              &transformedNormals);
        pruneCorrespondences(transformedNormals, target.getNormals(), matches);

        clock_t endMatch = clock();
        double elapsedSecsMatch = double(endMatch - beginMatch) / CLOCKS_PER_SEC;

        // Prepare point-to-point and point-to-plane constraints.
        ceres::Problem problem;
        prepareConstraints(transformedPoints,
                           target.getPoints(),
                           transformedNormals,
                           target.getNormals(),
                           matches,
                           poseIncrement,
                           problem);

        // Configure options for the solver.
        ceres::Solver::Options options;
        configureSolver(options);

        clock_t beginOpti = clock();
        // Run the solver (for one iteration).
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;
        std::cout << std::endl;

        clock_t endOpti = clock();
        double elapsedSecsOpti = double(endOpti - beginOpti) / CLOCKS_PER_SEC;

        // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
        Matrix4f matrix;
        if (m_bUseSymmetricConstraints)
        {
            matrix = PoseIncrement<double>::convertToRTRMatrix(poseIncrement);
        }
        else
        {
            matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
        }
        estimatedPose = matrix * estimatedPose;
        poseIncrement.setZero();

        // Calculate Error metric
        if (m_evaluate)
        {
            evaluator->addMetrics(elapsedSecsMatch,
                                  elapsedSecsOpti,
                                  source,
                                  target,
                                  matches,
                                  estimatedPose,
                                  groundPose);
        }
        std::cout << std::endl;
        std::cout << "Optimization iteration " << i + 1 << " / " << m_nIterations << " is done." << std::endl;
        std::cout << std::endl;
    }

    // Store result
    initialPose = estimatedPose;
}

void CeresICPOptimizer::configureSolver(ceres::Solver::Options &options)
{
    // Ceres options.
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.use_nonmonotonic_steps = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = 1;
    options.max_num_iterations = 1;
    options.num_threads = 8;
}

void CeresICPOptimizer::
    CeresICPOptimizer::prepareConstraints(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints,
                                          const std::vector<Vector3f> &sourceNormals, const std::vector<Vector3f> &targetNormals,
                                          const std::vector<Match> matches, const PoseIncrement<double> &poseIncrement, ceres::Problem &problem) const
{
    const unsigned nPoints = sourcePoints.size();

    for (unsigned i = 0; i < nPoints; ++i)
    {
        const auto match = matches[i];
        if (match.idx >= 0)
        {
            const auto &sourcePoint = sourcePoints[i];
            const auto &targetPoint = targetPoints[match.idx];

            if (m_bUsePointToPointConstraints)
            {
                if (!sourcePoint.allFinite() || !targetPoint.allFinite())
                    continue;

                ceres::CostFunction *cost_function = PointToPointConstraint::create(sourcePoint,
                                                                                    targetPoint,
                                                                                    m_weightPointToPointConstraints);
                problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());
            }

            if (m_bUsePointToPlaneConstraints)
            {
                const auto &targetNormal = targetNormals[match.idx];

                if (!targetNormal.allFinite())
                    continue;

                ceres::CostFunction *cost_function = PointToPlaneConstraint::create(sourcePoint,
                                                                                    targetPoint,
                                                                                    targetNormal,
                                                                                    m_weightPointToPlaneConstraints);
                problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());
            }

            if (m_bUseSymmetricConstraints)
            {
                const auto &targetNormal = targetNormals.at(match.idx);
                const auto &sourceNormal = sourceNormals[i];

                if (!targetNormal.allFinite() || !sourceNormal.allFinite())
                    continue;

                ceres::CostFunction *cost_function = SymmetricConstraint::create(sourcePoint,
                                                                                 targetPoint,
                                                                                 sourceNormal,
                                                                                 targetNormal,
                                                                                 m_weightSymmetricConstraints);
                problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());
            }
        }
    }
}

/**
 * ICP optimizer - using linear least-squares for optimization.
 */

LinearICPOptimizer::LinearICPOptimizer() {}

void LinearICPOptimizer::estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose, Matrix4f &groundPose)
{
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    m_corrAlgo->buildIndex(target.getPoints(), &target.getColors(), &target.getNormals());

    // The initial estimate can be given as an argument.
    Matrix4f estimatedPose = initialPose;
    std::vector<std::vector<double>> tmp;

    for (int i = 0; i < m_nIterations; ++i)
    {
        std::cout << "======================" << std::endl;
        std::cout << "Starting iteration: " << i + 1 << std::endl;
        std::cout << std::endl;

        // Compute the matches.
        clock_t beginMatch = clock();

        auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
        auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

        std::vector<Match> matches = m_corrAlgo->queryMatches(transformedPoints,
                                                              &source.getColors(),
                                                              &transformedNormals);
        pruneCorrespondences(transformedNormals, target.getNormals(), matches);

        clock_t endMatch = clock();
        double elapsedSecsMatch = double(endMatch - beginMatch) / CLOCKS_PER_SEC;

        std::vector<Vector3f> sourcePoints;
        std::vector<Vector3f> targetPoints;
        std::vector<Vector3f> targetNormals;
        std::vector<Vector3f> sourceNormals;

        // Add all matches to the sourcePoints and targetPoints vector,
        // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
        // the matches vector holds the index of the matching target point.
        for (int j = 0; j < transformedPoints.size(); j++)
        {
            const auto &match = matches[j];
            if (match.idx >= 0)
            {
                sourcePoints.push_back(transformedPoints[j]);
                sourceNormals.push_back(transformedNormals[j]);
                targetPoints.push_back(target.getPoints()[match.idx]);
                targetNormals.push_back(target.getNormals()[match.idx]);
            }
        }
        clock_t beginOpti = clock();
        // Estimate the new pose
        if (m_bUsePointToPointConstraints)
        {
            estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
        }
        else if (m_bUsePointToPlaneConstraints)
        {
            estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, targetNormals) * estimatedPose;
        }
        else if (m_bUseSymmetricConstraints)
        {
            estimatedPose = estimatePoseSymmetric(sourcePoints, targetPoints, sourceNormals, targetNormals) * estimatedPose;
        }
        else
        {
            std::cout << "Flags for all methods are set to false." << std::endl;
            exit(1);
        }
        clock_t endOpti = clock();
        double elapsedSecsOpti = double(endOpti - beginOpti) / CLOCKS_PER_SEC;

        // Calculate Error metric
        if (m_evaluate)
        {
            evaluator->addMetrics(
                elapsedSecsMatch,
                elapsedSecsOpti,
                source,
                target,
                matches,
                estimatedPose,
                groundPose);
        }

        std::cout << std::endl;
        std::cout << "Optimization iteration " << i + 1 << " / " << m_nIterations << " is done." << std::endl;
        std::cout << std::endl;
    }

    // Store result
    initialPose = estimatedPose;
}

Matrix4f LinearICPOptimizer::estimatePosePointToPoint(const std::vector<Vector3f> &sourcePoints,
                                                      const std::vector<Vector3f> &targetPoints)
{
    ProcrustesAligner procrustAligner;
    Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

    return estimatedPose;
}

Matrix4f LinearICPOptimizer::
    LinearICPOptimizer::estimatePosePointToPlane(const std::vector<Vector3f> &sourcePoints,
                                                 const std::vector<Vector3f> &targetPoints,
                                                 const std::vector<Vector3f> &targetNormals)
{
    const unsigned nPoints = sourcePoints.size();

    // Build the system
    MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
    VectorXf b = VectorXf::Zero(4 * nPoints);

    for (unsigned i = 0; i < nPoints; i++)
    {
        const auto &s = sourcePoints[i];
        const auto &d = targetPoints[i];
        const auto &n = targetNormals[i];

        Matrix3f cross_s;
        cross_s << 0., -s.z(), s.y(),
            s.z(), 0, -s.x(),
            -s.y(), s.x(), 0;

        auto diff = d - s;

        A.block(4 * i, 0, 3, 3) = -cross_s;
        A.block(4 * i, 3, 3, 3) = Matrix3f::Identity();
        b.segment(4 * i, 3) = diff;

        A.block(4 * i + 3, 0, 1, 3) = n.transpose() * -cross_s;
        A.block(4 * i + 3, 3, 1, 3) = n.transpose();
        b(4 * i + 3) = n.transpose() * diff;
    }

    CompleteOrthogonalDecomposition<MatrixXf> cod(A);
    VectorXf x = cod.solve(b);

    float alpha = x(0), beta = x(1), gamma = x(2);

    // Build the pose matrix
    Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                        AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

    Vector3f translation = x.tail(3);

    Matrix4f estimatedPose = Matrix4f::Identity();
    estimatedPose.block(0, 0, 3, 3) = rotation;
    estimatedPose.block(0, 3, 3, 1) = translation;

    return estimatedPose;
}

Matrix4f LinearICPOptimizer::estimatePoseSymmetric(const std::vector<Vector3f> &sourcePoints,
                                                   const std::vector<Vector3f> &targetPoints,
                                                   const std::vector<Vector3f> &sourceNormals,
                                                   const std::vector<Vector3f> &targetNormals)
{
    const unsigned nPoints = sourcePoints.size();

    // Build the system
    MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
    VectorXf b = VectorXf::Zero(4 * nPoints);

    for (unsigned i = 0; i < nPoints; i++)
    {
        const auto &s = sourcePoints[i];
        const auto &d = targetPoints[i];
        const auto &ns = sourceNormals[i];
        const auto &nt = targetNormals[i];

        Matrix3f cross_s;
        cross_s << 0., -s.z(), s.y(),
            s.z(), 0, -s.x(),
            -s.y(), s.x(), 0;

        Matrix3f cross_d;
        cross_d << 0., -d.z(), d.y(),
            d.z(), 0, -d.x(),
            -d.y(), d.x(), 0;

        auto diff = d - s;
        Vector3f normalSum = ns + nt;

        A.block(4 * i, 0, 3, 3) = -cross_s;
        A.block(4 * i, 3, 3, 3) = Matrix3f::Identity();
        b.segment(4 * i, 3) = diff;

        A.block(4 * i + 3, 0, 1, 3) = normalSum.transpose() * (-cross_s + cross_d);
        A.block(4 * i + 3, 3, 1, 3) = normalSum.transpose();
        b(4 * i + 3) = normalSum.transpose() * diff;
    }

    CompleteOrthogonalDecomposition<MatrixXf> cod(A);
    VectorXf x = cod.solve(b);

    float alpha = x(0), beta = x(1), gamma = x(2);

    // Build the pose matrix
    Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                        AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

    Vector3f translation = x.tail(3);

    Matrix4f estimatedPose = Matrix4f::Identity();
    estimatedPose.block(0, 0, 3, 3) = rotation;
    estimatedPose.block(0, 3, 3, 1) = translation;

    return estimatedPose;
}
