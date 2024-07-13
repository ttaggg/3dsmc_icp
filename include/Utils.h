#include <Eigen/Dense>
#include <random>
#include <cmath>

void visualize(std::string filenameOutput)
{

    // Visualize the mesh with Open3D.
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    if (!open3d::io::ReadTriangleMesh(filenameOutput, *mesh))
    {
        std::cerr << "Failed to read mesh from " << filenameOutput << std::endl;
        return;
    }

    if (!mesh->HasVertexNormals())
    {
        mesh->ComputeVertexNormals();
    }

    open3d::visualization::DrawGeometries({mesh}, "Mesh Visualization");
}

Matrix4f alignShapes(SimpleMesh &sourceMesh,
                     SimpleMesh &targetMesh,
                     ICPOptimizer *optimizer,
                     std::string filenameOutput)
{

    PointCloud source{sourceMesh};
    PointCloud target{targetMesh};

    Matrix4f estimatedPose = Matrix4f::Identity();
    optimizer->estimatePose(source, target, estimatedPose);

    // Visualize the resulting joined mesh. We add triangulated spheres for point matches.
    SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
    resultingMesh.writeMesh(filenameOutput);
    std::cout << "Resulting mesh written." << std::endl;

    return estimatedPose;
}

Matrix4f getRandomTransformation(std::mt19937 &rng, float lim_angle, float lim_trans)
{
    // Random angle distribution
    std::uniform_real_distribution<float> angle_dist(-lim_angle, lim_angle);
    // Random translation distribution
    std::uniform_real_distribution<float> trans_dist(-lim_trans, lim_trans);

    // Generate random rotations in degrees
    float angle_x = angle_dist(rng);
    float angle_y = angle_dist(rng);
    float angle_z = angle_dist(rng);

    // Convert angles to radians
    float angle_x_rad = angle_x * M_PI / 180.0f;
    float angle_y_rad = angle_y * M_PI / 180.0f;
    float angle_z_rad = angle_z * M_PI / 180.0f;

    // Generate rotation matrices
    Eigen::Matrix3f rotation_x;
    rotation_x = Eigen::AngleAxisf(angle_x_rad, Vector3f::UnitX());

    Eigen::Matrix3f rotation_y;
    rotation_y = Eigen::AngleAxisf(angle_y_rad, Vector3f::UnitY());

    Eigen::Matrix3f rotation_z;
    rotation_z = Eigen::AngleAxisf(angle_z_rad, Vector3f::UnitZ());

    // Combined rotation matrix
    Eigen::Matrix3f rotation = rotation_z * rotation_y * rotation_x;

    // Generate random translation
    Vector3f translation(trans_dist(rng), trans_dist(rng), trans_dist(rng));

    // Create the transformation matrix
    Matrix4f transformation = Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation.block<3, 1>(0, 3) = translation;

    return transformation;
}

ICPOptimizer *createOptimizer(const ICPConfiguration &config)
{
    ICPOptimizer *optimizer = nullptr;

    if (config.useLinearICP)
    {
        optimizer = new LinearICPOptimizer();
    }
    else
    {
        optimizer = new CeresICPOptimizer();
    }

    optimizer->setCorrespondenceMethod(config.correspondenceMethod, config.useColors);
    optimizer->setMatchingMaxDistance(config.matchingMaxDistance);
    optimizer->usePointToPointConstraints(config.usePointToPoint, config.weightPointToPoint);
    optimizer->usePointToPlaneConstraints(config.usePointToPlane, config.weightPointToPlane);
    optimizer->useSymmetricConstraints(config.useSymmetric, config.weightSymmetric);
    optimizer->setNbOfIterations(config.nbOfIterations);

    return optimizer;
}