#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>
#include "Eigen.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include "DataLoader.h"

#define OPEN3D_ENABLED 1
#ifndef OPEN3D_ENABLED 1
#include <Open3D/Open3D.h>
#endif

bool containsSubstring(const std::string &str, const std::string &substring);

std::string formatString(std::initializer_list<std::string> elements);

void visualize(std::string filenameOutput);

Matrix4f alignShapes(SimpleMesh &sourceMesh,
                     SimpleMesh &targetMesh,
                     ICPOptimizer *optimizer,
                     std::string filenameOutput,
                     std::vector<std::vector<double>> &metric);
Matrix4f getRandomTransformation(std::mt19937 &rng, float lim_angle, float lim_trans);
ICPOptimizer *createOptimizer(const ICPConfiguration &config);

std::unique_ptr<DataLoader> createDataloader(const std::string &directoryPath);