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
                                       const std::vector<Eigen::Vector3f> *targetColors)
{
    std::cout << "Initializing FLANN index with " << targetPoints.size() << " points." << std::endl;

    if (targetColors != nullptr)
    { // TODO(oleg): this logic is studip, maybe make a separate search methods for color.
        return _buildIndexWithColor(targetPoints, *targetColors);
    }

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

std::vector<Match> NearestNeighborSearch::queryMatches(const std::vector<Vector3f> &transformedPoints,
                                                       const std::vector<Eigen::Vector3f> *transformedColors)
{
    if (!m_index)
    {
        std::cout << "FLANN index needs to be build before querying any matches." << std::endl;
        return {};
    }

    if (transformedColors != nullptr)
    {
        return _queryMatchesWithColor(transformedPoints, *transformedColors);
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
void NearestNeighborSearch::_buildIndexWithColor(const std::vector<Eigen::Vector3f> &targetPoints,
                                                 const std::vector<Eigen::Vector3f> &targetColors)
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

std::vector<Match> NearestNeighborSearch::_queryMatchesWithColor(const std::vector<Eigen::Vector3f> &transformedPoints,
                                                                 const std::vector<Eigen::Vector3f> &transformedColors)
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
 * Projective correspondence.
 */

ProjectiveCorrespondence::ProjectiveCorrespondence() : Search(), m_maxDistance{0.005f} {}

ProjectiveCorrespondence::~ProjectiveCorrespondence() {}

void ProjectiveCorrespondence::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
                                          const std::vector<Eigen::Vector3f> *targetColors)
{
    m_targetPoints = targetPoints;
    if (targetColors)
    {
        m_targetColors = *targetColors;
    }
}

std::vector<Match> ProjectiveCorrespondence::queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints,
                                                          const std::vector<Eigen::Vector3f> *transformedColors)
{
    std::vector<Match> matches;
    matches.reserve(transformedPoints.size());

#pragma omp parallel for
    for (size_t i = 0; i < transformedPoints.size(); ++i)
    {
        const auto &transformedPoint = transformedPoints[i];
        const Eigen::Vector3f *transformedColor = transformedColors ? &(*transformedColors)[i] : nullptr;

        float minDistance = std::numeric_limits<float>::max();
        int bestMatch = -1;

        for (size_t j = 0; j < m_targetPoints.size(); ++j)
        {
            float distance = (transformedPoint - m_targetPoints[j]).norm();

            if (transformedColor && !m_targetColors.empty())
            {
                float colorDistance = ((*transformedColor) - m_targetColors[j]).norm();
                distance = 0.5 * distance + 0.5 * colorDistance; // Combine distances with dummy weights for now.
            }

            if (distance < minDistance && distance <= m_maxDistance)
            {
                minDistance = distance;
                bestMatch = static_cast<int>(j);
            }
        }

        if (bestMatch != -1)
        {
            matches.push_back(Match{bestMatch, 1.f});
        }
        else
        {
            matches.push_back(Match{-1, 0.f});
        }
    }

    return matches;
}
