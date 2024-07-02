#pragma once

#include <flann/flann.hpp>
#include "Eigen.h"

struct Match
{
	int idx;
	float weight;
};

class Search
{
public:
	virtual ~Search() {}
	virtual void setMatchingMaxDistance(float maxDistance);
	virtual void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints) = 0;
	virtual std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints) = 0;

protected:
	float m_maxDistance;
	Search();
};

/**
 * Nearest neighbor search using FLANN.
 */
class NearestNeighborSearchFlann : public Search
{
public:
	NearestNeighborSearchFlann();
	~NearestNeighborSearchFlann();
	void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints);
	std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints);

private:
	int m_nTrees;
	flann::Index<flann::L2<float>> *m_index;
	float *m_flatPoints;
};
