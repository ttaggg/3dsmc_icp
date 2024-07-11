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
							const std::vector<Eigen::Vector3f> *targetColors = nullptr,
							const std::vector<Vector3f> *targetNormals = nullptr) = 0;
	virtual std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints,
											const std::vector<Eigen::Vector3f> *transformedColors = nullptr,
											const std::vector<Eigen::Vector3f> *transformedNormals = nullptr) = 0;

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
					const std::vector<Eigen::Vector3f> *targetColors = nullptr,
					const std::vector<Eigen::Vector3f> *targetNormals = nullptr);
	std::vector<Match> queryMatches(const std::vector<Eigen::Vector3f> &transformedPoints,
									const std::vector<Eigen::Vector3f> *transformedColors = nullptr,
									const std::vector<Eigen::Vector3f> *transformedNormals = nullptr);

protected:
	int m_nTrees;
	flann::Index<flann::L2<float>> *m_index;
	float *m_flatPoints;
};

class NearestNeighborSearchWithColors : public NearestNeighborSearch
{
public:
	void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
					const std::vector<Eigen::Vector3f> &targetColors,
					const std::vector<Eigen::Vector3f> &targetNormals);
	std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints,
									const std::vector<Eigen::Vector3f> &transformedColors,
									const std::vector<Eigen::Vector3f> &transformedNormals);
};

/**
 * Normal shooting correspondence.
 */

class NormalShootCorrespondence : public Search
{
public:
	NormalShootCorrespondence();
	~NormalShootCorrespondence();

	void buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
					const std::vector<Eigen::Vector3f> *targetColors = nullptr,
					const std::vector<Vector3f> *targetNormals = nullptr);
	std::vector<Match> queryMatches(const std::vector<Vector3f> &transformedPoints,
									const std::vector<Eigen::Vector3f> *transformedColors = nullptr,
									const std::vector<Vector3f> *transformedNormals = nullptr);

private:
	std::vector<Eigen::Vector3f> m_targetPoints;
	int m_nTrees;
	float *m_flatPoints;
	flann::Index<flann::L2<float>> *m_index;

	void _buildIndex(const std::vector<Eigen::Vector3f> &targetPoints,
					 const std::vector<Eigen::Vector3f> &targetColors,
					 const std::vector<Vector3f> &targetNormals);
	std::vector<Match> _queryMatches(const std::vector<Vector3f> &transformedPoints,
									 const std::vector<Eigen::Vector3f> &transformedColors,
									 const std::vector<Vector3f> &transformedNormals);

	bool findRayIntersection(const Eigen::Vector3f &origin,
							 const Eigen::Vector3f &direction,
							 Eigen::Vector3f &intersection);
	int findNearestNeighbor(const Eigen::Vector3f &point);
};