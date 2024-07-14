

class MeshDataLoader
{
public:
    MeshDataLoader(const std::string &directoryPath);
    bool getMesh(size_t index, SimpleMesh &mesh);
    std::string getName(size_t index);
    size_t size();

private:
    std::vector<std::string> meshPaths;
    void loadMeshPaths(const std::string &directoryPath);
};