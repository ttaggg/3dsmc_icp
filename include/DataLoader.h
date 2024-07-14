#pragma once

class DataLoader
{
public:
    size_t size();
    std::string getName(size_t index);

    virtual bool createMeshes(size_t index,
                              SimpleMesh &sourceMesh,
                              SimpleMesh &targetMesh,
                              Matrix4f &gt_trans) = 0;
    virtual void loadMeshPaths(const std::string &directoryPath) = 0;
    virtual ~DataLoader() = default;

protected:
    std::vector<std::string> meshPaths;
};

class MeshDataLoader : public DataLoader
{
public:
    bool createMeshes(size_t index,
                      SimpleMesh &sourceMesh,
                      SimpleMesh &targetMesh,
                      Matrix4f &gt_trans);
    void loadMeshPaths(const std::string &directoryPath);
};

class PartialMeshDataLoader : public DataLoader
{
public:
    bool createMeshes(size_t index,
                      SimpleMesh &sourceMesh,
                      SimpleMesh &targetMesh,
                      Matrix4f &gt_trans);
    void loadMeshPaths(const std::string &directoryPath);
};