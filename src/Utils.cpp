#include "Utils.h"
#include <cmath>              // for M_PI
#include <iostream>           // for basic_ostream
#include <sstream>            // for basic_ostringst...
#include <sys/stat.h>         //
#include <sys/types.h>        //
#include <utility>            // for move
#include "DataLoader.h"       // for MeshDataLoader
#include "ICPConfiguration.h" // for ICPConfiguration
#include "ICPOptimizer.h"     // for ICPOptimizer
#include "PointCloud.h"       // for PointCloud
#include "SimpleMesh.h"       // for SimpleMesh
#include "Eigen.h"

#ifdef OPEN3D_ENABLED
#include <Open3D/Open3D.h>
#endif

bool containsSubstring(const std::string &str, const std::string &substring)
{
    return str.find(substring) != std::string::npos;
}

std::string formatString(std::initializer_list<std::string> elements)
// formatString without installing fmt or C++20
{
    std::ostringstream oss;
    for (auto &el : elements)
    {
        oss << el;
    }
    return oss.str();
}

#define OPEN3D_ENABLED
#ifdef OPEN3D_ENABLED
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
#endif

Matrix4f alignShapes(SimpleMesh &sourceMesh,
                     SimpleMesh &targetMesh,
                     Matrix4f &gtTransform,
                     ICPOptimizer *optimizer,
                     std::string filenameOutput)
{

    PointCloud source{sourceMesh};
    PointCloud target{targetMesh};

    Matrix4f estimatedPose = Matrix4f::Identity();
    optimizer->estimatePose(source, target, estimatedPose, gtTransform);

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
    Matrix3f rotation_x;
    rotation_x = AngleAxisf(angle_x_rad, Vector3f::UnitX());

    Matrix3f rotation_y;
    rotation_y = AngleAxisf(angle_y_rad, Vector3f::UnitY());

    Matrix3f rotation_z;
    rotation_z = AngleAxisf(angle_z_rad, Vector3f::UnitZ());

    // Combined rotation matrix
    Matrix3f rotation = rotation_z * rotation_y * rotation_x;

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

std::unique_ptr<DataLoader> createDataloader(const std::string &directoryPath)
{
    if (containsSubstring(directoryPath, "partial"))
    {
        return std::make_unique<PartialMeshDataLoader>();
    }
    else
    {
        return std::make_unique<MeshDataLoader>();
    }
}

bool _directoryExists(const std::string &path)
{
    struct stat info;

    if (stat(path.c_str(), &info) != 0)
    {
        return false;
    }
    else if (info.st_mode & S_IFDIR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool _createDirectory(const std::string &path)
{
#if defined(_WIN32)
    int ret = _mkdir(path.c_str());
#else
    mode_t mode = 0755; // Default mode
    int ret = mkdir(path.c_str(), mode);
#endif
    return (ret == 0);
}

bool ensureDirectoryExists(const std::string &path)
{
    if (_directoryExists(path))
    {
        return true;
    }
    else
    {
        return _createDirectory(path);
    }
}