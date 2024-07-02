#include "Search.h"

Search::Search() : m_maxDistance{0.005f} {}

void Search::setMatchingMaxDistance(float maxDistance)
{
    m_maxDistance = maxDistance;
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

ProjectiveCorrespondence::ProjectiveCorrespondence() : Search(), m_maxDistance{0.005f} {}

ProjectiveCorrespondence::~ProjectiveCorrespondence() {}

void ProjectiveCorrespondence::buildIndex(const std::vector<Eigen::Vector3f> &targetPoints)
{
    m_targetPoints = targetPoints;
}

std::vector<Match> ProjectiveCorrespondence::queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints)
{
    std::vector<Match> matches;
    matches.reserve(transformedPoints.size());

    for (const auto &transformedPoint : transformedPoints)
    {
        float minDistance = std::numeric_limits<float>::max();
        int bestMatch = -1;

        for (size_t i = 0; i < m_targetPoints.size(); ++i)
        {
            float distance = (transformedPoint - m_targetPoints[i]).norm();
            if (distance < minDistance && distance <= m_maxDistance)
            {
                minDistance = distance;
                bestMatch = static_cast<int>(i);
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