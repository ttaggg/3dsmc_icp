#include "Utils.h"
#include <cmath>              // for M_PI
#include <iostream>           // for basic_ostream
#include <Open3D/Open3D.h>    //
#include <sstream>            // for basic_ostringst...
#include <sys/stat.h>         //
#include <sys/types.h>        //
#include <utility>            // for move
#include "DataLoader.h"       // for MeshDataLoader
#include "ICPConfiguration.h" // for ICPConfiguration
#include "ICPOptimizer.h"     // for ICPOptimizer
#include "PointCloud.h"       // for PointCloud
#include "SimpleMesh.h"       // for SimpleMesh
#include "VirtualSensor.h"    // for VirtualSensor
#include "Eigen.h"

namespace fs = std::filesystem;

bool containsSubstring(const std::string &str, const std::string &substring)
{
    return str.find(substring) != std::string::npos;
}

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
                     Matrix4f &gtTransform,
                     ICPOptimizer *optimizer)
{

    PointCloud source{sourceMesh};
    PointCloud target{targetMesh};

    Matrix4f estimatedPose = Matrix4f::Identity();
    optimizer->estimatePose(source, target, estimatedPose, gtTransform);

    return estimatedPose;
}

void writeShapeMesh(SimpleMesh &sourceMesh,
                    SimpleMesh &targetMesh,
                    Matrix4f &estimatedPose,
                    std::string filenameOutputColor,
                    std::string filenameOutputRG)
{

    PointCloud source{sourceMesh};
    PointCloud target{targetMesh};

    SimpleMesh resultingMeshRG = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
    resultingMeshRG.writeMesh(filenameOutputRG);

    SimpleMesh resultingMeshColor = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose, true);
    resultingMeshColor.writeMesh(filenameOutputColor);
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

void writeRoomMesh(VirtualSensor &sensor, Matrix4f &currentCameraPose, fs::path outputDir)
{
    fs::create_directories(outputDir);

    SimpleMesh currentDepthMesh{sensor, currentCameraPose, 0.1f};
    SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
    SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh,
                                                      currentCameraMesh,
                                                      Matrix4f::Identity(),
                                                      true);

    fs::path outputMeshPath = outputDir / fs::path{std::to_string(sensor.getCurrentFrameCnt()) + ".off"};

    if (!resultingMesh.writeMesh(outputMeshPath))
    {
        std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
        return;
    }
}