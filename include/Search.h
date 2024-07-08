#pragma once

#include <flann/flann.hpp>
#include "Eigen.h"

struct Match
{
	int idx;
	float weight;
	Eigen::Vector3f color = {MINF, MINF, MINF};
};

class Search
{
public:
	virtual ~Search() {}
	virtual void setMatchingMaxDistance(float maxDistance);
	virtual void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
							const std::vector<Eigen::Vector3f> *targetColors = nullptr) = 0;
	virtual std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints,
											const std::vector<Eigen::Vector3f> *transformedColors = nullptr) = 0;

protected:
	float m_maxDistance;
	Search();
};

/**
 * Nearest neighbor search using FLANN.
 */
class NearestNeighborSearch : public Search
{
public:
	NearestNeighborSearch();
	~NearestNeighborSearch();
	void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
					const std::vector<Eigen::Vector3f> *targetColors = nullptr);
	std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints,
									const std::vector<Eigen::Vector3f> *transformedColors = nullptr);

private:
	int m_nTrees;
	flann::Index<flann::L2<float>> *m_index;
	float *m_flatPoints;

	void _buildIndexWithColor(const std::vector<Eigen::Vector3f> &targetPoints,
							  const std::vector<Eigen::Vector3f> &targetColors);
	std::vector<Match> _queryMatchesWithColor(const std::vector<Vector3f> &transformedPoints,
											  const std::vector<Eigen::Vector3f> &transformedColors);
};

/**
 * Projective correspondence.
 */

class ProjectiveCorrespondence : public Search
{
public:
	ProjectiveCorrespondence();
	~ProjectiveCorrespondence();

	void buildIndex(const std::vector<Eigen::Vector3f> &targetPoint,
					const std::vector<Eigen::Vector3f> *targetColors = nullptr);
	std::vector<Match> queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints,
									const std::vector<Eigen::Vector3f> *transformedColors = nullptr);

private:
	std::vector<Eigen::Vector3f> m_targetPoints;
	std::vector<Eigen::Vector3f> m_targetColors;
	float m_maxDistance;
};