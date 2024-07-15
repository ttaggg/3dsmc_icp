#pragma once

#include <random>           // for mt19937
#include <initializer_list> // for initializer_list
#include <memory>           // for unique_ptr
#include <string>           // for string
#include "Eigen.h"

class DataLoader;
class ICPOptimizer;
class ICPConfiguration;
class SimpleMesh;

bool containsSubstring(const std::string &str, const std::string &substring);

std::string formatString(std::initializer_list<std::string> elements);

#define OPEN3D_ENABLED
#ifdef OPEN3D_ENABLED
void visualize(std::string filenameOutput);
#endif

Matrix4f alignShapes(SimpleMesh &sourceMesh,
                     SimpleMesh &targetMesh,
                     Matrix4f &gtTransform,
                     ICPOptimizer *optimizer,
                     std::string filenameOutput);

Matrix4f getRandomTransformation(std::mt19937 &rng, float lim_angle, float lim_trans);

ICPOptimizer *createOptimizer(const ICPConfiguration &config);

std::unique_ptr<DataLoader> createDataloader(const std::string &directoryPath);

bool _directoryExists(const std::string &path);
bool _createDirectory(const std::string &path);
bool ensureDirectoryExists(const std::string &path);
