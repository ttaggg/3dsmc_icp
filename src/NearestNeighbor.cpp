#include "NearestNeighbor.h"

Search::Search() : m_maxDistance{0.005f} {}

void Search::setMatchingMaxDistance(float maxDistance)
{
    m_maxDistance = maxDistance;
}

NearestNeighborSearchBruteForce::NearestNeighborSearchBruteForce() : Search() {}

void NearestNeighborSearchBruteForce::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints)
{
    m_points = targetPoints;
}

std::vector<Match> NearestNeighborSearchBruteForce::queryMatches(const std::vector<Vector3f> &transformedPoints)
{
    const unsigned nMatches = transformedPoints.size();
    std::vector<Match> matches(nMatches);
    const unsigned nTargetPoints = m_points.size();
    std::cout << "nMatches: " << nMatches << std::endl;
    std::cout << "nTargetPoints: " << nTargetPoints << std::endl;

#pragma omp parallel for
    for (int i = 0; i < nMatches; i++)
    {
        matches[i] = getClosestPoint(transformedPoints[i]);
    }

    return matches;
}

Match NearestNeighborSearchBruteForce::getClosestPoint(const Vector3f &p)
{
    int idx = -1;

    float minDist = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < m_points.size(); ++i)
    {
        float dist = (p - m_points[i]).norm();
        if (minDist > dist)
        {
            idx = i;
            minDist = dist;
        }
    }

    if (minDist <= m_maxDistance)
        return Match{idx, 1.f};
    else
        return Match{-1, 0.f};
}

NearestNeighborSearchFlann::NearestNeighborSearchFlann() : Search(),
                                                           m_nTrees{1},
                                                           m_index{nullptr},
                                                           m_flatPoints{nullptr}
{
}

NearestNeighborSearchFlann::~NearestNeighborSearchFlann()
{
    if (m_index)
    {
        delete m_flatPoints;
        delete m_index;
        m_flatPoints = nullptr;
        m_index = nullptr;
    }
}

void NearestNeighborSearchFlann::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints)
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

std::vector<Match> NearestNeighborSearchFlann::queryMatches(const std::vector<Vector3f> &transformedPoints)
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
