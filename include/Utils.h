#pragma once

#include <filesystem>
#include <random>           // for mt19937
#include <initializer_list> // for initializer_list
#include <memory>           // for unique_ptr
#include <string>           // for string
#include "Eigen.h"

class DataLoader;
class ICPOptimizer;
class ICPConfiguration;
class SimpleMesh;
class VirtualSensor;

namespace fs = std::filesystem;

bool containsSubstring(const std::string &str, const std::string &substring);

void visualize(std::string filenameOutput);

Matrix4f alignShapes(SimpleMesh &sourceMesh,
                     SimpleMesh &targetMesh,
                     Matrix4f &gtTransform,
                     ICPOptimizer *optimizer);

void writeShapeMesh(SimpleMesh &sourceMesh,
                    SimpleMesh &targetMesh,
                    Matrix4f &estimatedPose,
                    std::string filenameOutputColor,
                    std::string filenameOutputRG);

Matrix4f getRandomTransformation(std::mt19937 &rng, float lim_angle, float lim_trans);

ICPOptimizer *createOptimizer(const ICPConfiguration &config);

std::unique_ptr<DataLoader> createDataloader(const std::string &directoryPath);

void writeRoomMesh(VirtualSensor &sensor, Matrix4f &currentCameraPose, fs::path outputDir);
