#include "DataLoader.h"
#include <filesystem>   // for directory_iterator, path, directory_entry
#include <iostream>     // for basic_ostream, operator<<, endl, cerr, cout
#include "SimpleMesh.h" // for SimpleMesh
#include "Utils.h"      // for formatString

namespace fs = std::filesystem;

size_t DataLoader::size()
{
    return meshPaths.size();
}

std::string DataLoader::getName(size_t index)
{
    if (index >= meshPaths.size())
    {
        std::cerr << "Index out of bounds: " << index << std::endl;
        return "";
    }

    // Get the path
    std::filesystem::path meshPath = meshPaths[index];

    // Extract the filename without extension
    std::string filename = meshPath.stem().string();

    return filename;
}

bool MeshDataLoader::createMeshes(size_t index,
                                  SimpleMesh &sourceMesh,
                                  SimpleMesh &targetMesh,
                                  Matrix4f &gt_trans)
{
    if (index >= meshPaths.size())
    {
        std::cerr << "Index out of bounds: " << index << std::endl;
        return false;
    }

    sourceMesh.loadMesh(meshPaths[index]);
    targetMesh = sourceMesh.transformMesh(gt_trans);

    return true;
}

void MeshDataLoader::loadMeshPaths(const std::string &directoryPath)
{
    for (const auto &entry : std::filesystem::directory_iterator(directoryPath))
    {
        if (entry.is_regular_file())
        {
            std::string path = entry.path().string();
            if (path.substr(path.find_last_of(".") + 1) == "off")
            {
                meshPaths.push_back(path);
                std::cout << "Found mesh: " << path << std::endl;
            }
        }
    }
}

bool PartialMeshDataLoader::createMeshes(size_t index,
                                         SimpleMesh &sourceMesh,
                                         SimpleMesh &targetMesh,
                                         Matrix4f &gt_trans)
{

    if (index >= meshPaths.size())
    {
        std::cerr << "Index out of bounds: " << index << std::endl;
        return false;
    }

    std::string sourceMeshPath = fs::path{meshPaths[index]} / fs::path{"1.off"};
    std::string targetMeshPath = fs::path{meshPaths[index]} / fs::path{"2.off"};

    std::cout << sourceMeshPath << std::endl;
    std::cout << targetMeshPath << std::endl;

    sourceMesh.loadMesh(sourceMeshPath);

    SimpleMesh tmpMesh;
    tmpMesh.loadMesh(targetMeshPath);

    targetMesh = tmpMesh.transformMesh(gt_trans);

    return true;
}

void PartialMeshDataLoader::loadMeshPaths(const std::string &directoryPath)
{
    for (const auto &entry : std::filesystem::directory_iterator(directoryPath))
    {
        if (entry.is_directory())
        {
            // Add directory name.
            std::string path = entry.path().string();
            meshPaths.push_back(path);
        }
    }
}
