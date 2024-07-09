#include "Search.h"

Search::Search() : m_maxDistance{0.005f} {}

void Search::setMatchingMaxDistance(float maxDistance)
{
    m_maxDistance = maxDistance;
}

/**
 * Nearest neighbor search using FLANN.
 */

NearestNeighborSearch::NearestNeighborSearch() : Search(),
                                                 m_nTrees{1},
                                                 m_index{nullptr},
                                                 m_flatPoints{nullptr}
{
}

NearestNeighborSearch::~NearestNeighborSearch()
{
    if (m_index)
    {
        delete m_flatPoints;
        delete m_index;
        m_flatPoints = nullptr;
        m_index = nullptr;
    }
}

void NearestNeighborSearch::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
                                       const std::vector<Eigen::Vector3f> *targetColors,
                                       const std::vector<Eigen::Vector3f> *targetNormals)
{
    std::cout << "Initializing FLANN index with " << targetPoints.size() << " points." << std::endl;

    // FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
    m_flatPoints = new float[targetPoints.size() * 3];
    for (size_t pointIndex = 0; pointIndex < targetPoints.size(); pointIndex++)
    {
        for (size_t dim = 0; dim < 3; dim++)
        {
            m_flatPoints[pointIndex * 3 + dim] = targetPoints[pointIndex][dim];
        }
    }

    flann::Matrix<float> dataset(m_flatPoints, targetPoints.size(), 3);

    // Building the index takes some time.
    m_index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(m_nTrees));
    m_index->buildIndex();

    std::cout << "FLANN index created." << std::endl;
}

std::vector<Match> NearestNeighborSearch::queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints,
                                                       const std::vector<Eigen::Vector3f> *transformedColors,
                                                       const std::vector<Eigen::Vector3f> *transformedNormals)
{
    if (!m_index)
    {
        std::cout << "FLANN index needs to be build before querying any matches." << std::endl;
        return {};
    }

    // FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
    float *queryPoints = new float[transformedPoints.size() * 3];
    for (size_t pointIndex = 0; pointIndex < transformedPoints.size(); pointIndex++)
    {
        for (size_t dim = 0; dim < 3; dim++)
        {
            queryPoints[pointIndex * 3 + dim] = transformedPoints[pointIndex][dim];
        }
    }

    flann::Matrix<float> query(queryPoints, transformedPoints.size(), 3);
    flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
    flann::Matrix<float> distances(new float[query.rows * 1], query.rows, 1);

    // Do a knn search, searching for 1 nearest point and using 16 checks.
    flann::SearchParams searchParams{16};
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    // Filter the matches.
    const unsigned nMatches = transformedPoints.size();
    std::vector<Match> matches;
    matches.reserve(nMatches);

    for (int i = 0; i < nMatches; ++i)
    {
        if (*distances[i] <= m_maxDistance)
            matches.push_back(Match{*indices[i], 1.f});
        else
            matches.push_back(Match{-1, 0.f});
    }

    // Release the memory.
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] distances.ptr();

    return matches;
}

/**
 * Nearest neighbor search using FLANN with color information.
 */
void NearestNeighborSearchWithColors::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
                                                 const std::vector<Eigen::Vector3f> &targetColors,
                                                 const std::vector<Eigen::Vector3f> &targetNormals)
{
    // Assuming targetColors.size() == targetPoints.size()
    m_flatPoints = new float[targetPoints.size() * 6];
    for (size_t pointIndex = 0; pointIndex < targetPoints.size(); ++pointIndex)
    {
        for (size_t dim = 0; dim < 3; ++dim)
        {
            m_flatPoints[pointIndex * 6 + dim] = targetPoints[pointIndex][dim];
            m_flatPoints[pointIndex * 6 + 3 + dim] = targetColors[pointIndex][dim];
        }
    }

    flann::Matrix<float> dataset(m_flatPoints, targetPoints.size(), 6);
    m_index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(m_nTrees));
    m_index->buildIndex();
    std::cout << "FLANN index with color created." << std::endl;
}

std::vector<Match> NearestNeighborSearchWithColors::queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints,
                                                                 const std::vector<Eigen::Vector3f> &transformedColors,
                                                                 const std::vector<Eigen::Vector3f> &transformedNormals)
{
    // Assuming transformedColors.size() == transformedPoints.size()
    float *queryPoints = new float[transformedPoints.size() * 6];
    for (size_t pointIndex = 0; pointIndex < transformedPoints.size(); ++pointIndex)
    {
        for (size_t dim = 0; dim < 3; ++dim)
        {
            queryPoints[pointIndex * 6 + dim] = transformedPoints[pointIndex][dim];
            queryPoints[pointIndex * 6 + 3 + dim] = transformedColors[pointIndex][dim];
        }
    }

    flann::Matrix<float> query(queryPoints, transformedPoints.size(), 6);
    flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
    flann::Matrix<float> distances(new float[query.rows * 1], query.rows, 1);

    flann::SearchParams searchParams{16};
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    const unsigned nMatches = transformedPoints.size();
    std::vector<Match> matches;
    matches.reserve(nMatches);

    for (int i = 0; i < nMatches; ++i)
    {
        if (*distances[i] <= m_maxDistance)
            matches.push_back(Match{*indices[i], 1.f});
        else
            matches.push_back(Match{-1, 0.f});
    }

    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] distances.ptr();

    return matches;
}

/**
 * Normal shoot correspondence.
 */

NormalShootCorrespondence::NormalShootCorrespondence()
    : Search(),
      m_nTrees{1},
      m_index{nullptr},
      m_flatPoints{nullptr} {}

NormalShootCorrespondence::~NormalShootCorrespondence()
{
    if (m_flatPoints)
    {
        delete[] m_flatPoints;
    }
    if (m_index)
    {
        delete m_index;
    }
}

void NormalShootCorrespondence::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
                                           const std::vector<Eigen::Vector3f> *targetColors,
                                           const std::vector<Eigen::Vector3f> *targetNormals)
{
    return _buildIndex(targetPoints, *targetColors, *targetNormals);
}

void NormalShootCorrespondence::_buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
                                            const std::vector<Eigen::Vector3f> &targetColors,
                                            const std::vector<Eigen::Vector3f> &targetNormals)
{
    std::cout << "Initializing FLANN index with " << targetPoints.size() << " points." << std::endl;

    m_flatPoints = new float[targetPoints.size() * 3];
    for (size_t pointIndex = 0; pointIndex < targetPoints.size(); pointIndex++)
    {
        for (size_t dim = 0; dim < 3; dim++)
        {
            m_flatPoints[pointIndex * 3 + dim] = targetPoints[pointIndex][dim];
        }
    }

    flann::Matrix<float> dataset(m_flatPoints, targetPoints.size(), 3);

    m_index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(m_nTrees));
    m_index->buildIndex();

    std::cout << "FLANN index created." << std::endl;
}

std::vector<Match> NormalShootCorrespondence::queryMatches(const std::vector<Vector3f> &transformedPoints,
                                                           const std::vector<Vector3f> *transformedColors,
                                                           const std::vector<Vector3f> *transformedNormals)
{
    return _queryMatches(transformedPoints, *transformedColors, *transformedNormals);
}

std::vector<Match> NormalShootCorrespondence::_queryMatches(const std::vector<Vector3f> &transformedPoints,
                                                            const std::vector<Vector3f> &transformedColors,
                                                            const std::vector<Vector3f> &transformedNormals)
{
    if (!m_index)
    {
        std::cout << "FLANN index needs to be built before querying any matches." << std::endl;
        return {};
    }

    std::vector<Match> matches;
    for (int idx = 0; idx < transformedPoints.size(); ++idx)
    {
        auto point = transformedPoints.at(idx);
        auto normal = transformedNormals.at(idx);
        Eigen::Vector3f intersection;
        if (findRayIntersection(point, normal, intersection))
        {
            int matched_index = findNearestNeighbor(intersection);
            matches.push_back(Match{matched_index, 1.});
        }
        else
        {
            matches.push_back(Match{-1, 0.});
        }
    }

    return matches;
}

bool NormalShootCorrespondence::findRayIntersection(const Eigen::Vector3f &origin, const Eigen::Vector3f &direction, Eigen::Vector3f &intersection)
{
    float queryPoint[3] = {origin.x(), origin.y(), origin.z()};
    flann::Matrix<float> query(queryPoint, 1, 3);
    flann::Matrix<int> indices(new int[1], 1, 1);
    flann::Matrix<float> distances(new float[1], 1, 1);

    flann::SearchParams searchParams{16};
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    if (distances[0][0] <= m_maxDistance)
    {
        Eigen::Vector3f nearestPoint(
            m_flatPoints[indices[0][0] * 3],
            m_flatPoints[indices[0][0] * 3 + 1],
            m_flatPoints[indices[0][0] * 3 + 2]);

        Eigen::Vector3f vec = nearestPoint - origin;
        float t = vec.dot(direction) / direction.dot(direction);
        intersection = origin + t * direction;

        delete[] indices.ptr();
        delete[] distances.ptr();
        return true;
    }

    delete[] indices.ptr();
    delete[] distances.ptr();
    return false;
}

int NormalShootCorrespondence::findNearestNeighbor(const Eigen::Vector3f &point)
{
    float queryPoint[3] = {point.x(), point.y(), point.z()};
    flann::Matrix<float> query(queryPoint, 1, 3);
    flann::Matrix<int> indices(new int[1], 1, 1);
    flann::Matrix<float> distances(new float[1], 1, 1);

    flann::SearchParams searchParams{16};
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    int nearestIndex = indices[0][0];

    delete[] indices.ptr();
    delete[] distances.ptr();

    return nearestIndex;
}