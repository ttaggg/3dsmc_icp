#include "ICPOptimizer.h"

/**
 * ICP optimizer - Abstract Base Class
 */

ICPOptimizer::ICPOptimizer() : m_bUsePointToPlaneConstraints{false},
                               m_nIterations{20},
                               m_nearestNeighborSearch{std::make_unique<NearestNeighborSearchFlann>()}
{
}

void ICPOptimizer::setMatchingMaxDistance(float maxDistance)
{
    m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
}

void ICPOptimizer::usePointToPlaneConstraints(bool bUsePointToPlaneConstraints)
{
    m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
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
        Match &match = matches[i];
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

void CeresICPOptimizer::estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose)
{
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    m_nearestNeighborSearch->buildIndex(target.getPoints());

    // The initial estimate can be given as an argument.
    Matrix4f estimatedPose = initialPose;

    // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
    // the rotation angle) and 3 parameters for the translation vector.
    double incrementArray[6];
    auto poseIncrement = PoseIncrement<double>(incrementArray);
    poseIncrement.setZero();

    for (int i = 0; i < m_nIterations; ++i)
    {
        // Compute the matches.
        std::cout << "Matching points ..." << std::endl;
        clock_t begin = clock();

        auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
        auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

        auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
        pruneCorrespondences(transformedNormals, target.getNormals(), matches);

        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

        // Prepare point-to-point and point-to-plane constraints.
        ceres::Problem problem;
        prepareConstraints(transformedPoints, target.getPoints(), target.getNormals(), matches, poseIncrement, problem);

        // Configure options for the solver.
        ceres::Solver::Options options;
        configureSolver(options);

        // Run the solver (for one iteration).
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        // std::cout << summary.FullReport() << std::endl;

        // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
        Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
        estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
        poseIncrement.setZero();

        std::cout << "Optimization iteration done." << std::endl;
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
    CeresICPOptimizer::prepareConstraints(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints, const std::vector<Vector3f> &targetNormals, const std::vector<Match> matches, const PoseIncrement<double> &poseIncrement, ceres::Problem &problem) const
{
    const unsigned nPoints = sourcePoints.size();

    for (unsigned i = 0; i < nPoints; ++i)
    {
        const auto match = matches[i];
        if (match.idx >= 0)
        {
            const auto &sourcePoint = sourcePoints[i];
            const auto &targetPoint = targetPoints[match.idx];

            if (!sourcePoint.allFinite() || !targetPoint.allFinite())
                continue;

            const double &weight = 1.0;
            ceres::CostFunction *cost_function = PointToPointConstraint::create(sourcePoint, targetPoint, weight);
            problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());

            if (m_bUsePointToPlaneConstraints)
            {
                const auto &targetNormal = targetNormals[match.idx];

                if (!targetNormal.allFinite())
                    continue;

                const double &weight = 1.0;
                ceres::CostFunction *cost_function = PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, weight);
                problem.AddResidualBlock(cost_function, nullptr, poseIncrement.getData());
            }
        }
    }
}

/**
 * ICP optimizer - using linear least-squares for optimization.
 */

LinearICPOptimizer::LinearICPOptimizer() {}

void LinearICPOptimizer::estimatePose(const PointCloud &source, const PointCloud &target, Matrix4f &initialPose)
{
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    m_nearestNeighborSearch->buildIndex(target.getPoints());

    // The initial estimate can be given as an argument.
    Matrix4f estimatedPose = initialPose;

    for (int i = 0; i < m_nIterations; ++i)
    {
        // Compute the matches.
        std::cout << "Matching points ..." << std::endl;
        clock_t begin = clock();

        auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
        auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

        auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
        pruneCorrespondences(transformedNormals, target.getNormals(), matches);

        clock_t end = clock();
        double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

        std::vector<Vector3f> sourcePoints;
        std::vector<Vector3f> targetPoints;

        // Add all matches to the sourcePoints and targetPoints vector,
        // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
        // the matches vector holds the index of the matching target point.
        for (int j = 0; j < transformedPoints.size(); j++)
        {
            const auto &match = matches[j];
            if (match.idx >= 0)
            {
                sourcePoints.push_back(transformedPoints[j]);
                targetPoints.push_back(target.getPoints()[match.idx]);
            }
        }

        // Estimate the new pose
        if (m_bUsePointToPlaneConstraints)
        {
            estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals()) * estimatedPose;
        }
        else
        {
            estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
        }

        std::cout << "Optimization iteration done." << std::endl;
    }

    // Store result
    initialPose = estimatedPose;
}

Matrix4f LinearICPOptimizer::estimatePosePointToPoint(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints)
{
    ProcrustesAligner procrustAligner;
    Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

    return estimatedPose;
}

Matrix4f LinearICPOptimizer::
    LinearICPOptimizer::estimatePosePointToPlane(const std::vector<Vector3f> &sourcePoints, const std::vector<Vector3f> &targetPoints, const std::vector<Vector3f> &targetNormals)
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

    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXf> cod(A);
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
