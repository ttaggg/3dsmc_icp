#include <filesystem>
#include <string>
#include <vector>
#include <iostream>
#include "SimpleMesh.h"
#include "MeshDataLoader.h"

MeshDataLoader::MeshDataLoader(const std::string &directoryPath)
{
    loadMeshPaths(directoryPath);
}

bool MeshDataLoader::getMesh(size_t index, SimpleMesh &mesh)
{
    if (index >= meshPaths.size())
    {
        std::cerr << "Index out of bounds: " << index << std::endl;
        return false;
    }

    return mesh.loadMesh(meshPaths[index]);
}

std::string MeshDataLoader::getName(size_t index)
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

size_t MeshDataLoader::size()
{
    return meshPaths.size();
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
