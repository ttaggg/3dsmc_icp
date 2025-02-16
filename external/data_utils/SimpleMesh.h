#pragma once

#include <iostream>
#include <fstream>
#include <random>
#include <unordered_set>
#include "Eigen.h"
#include "VirtualSensor.h"
#include <Utils.h>

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// Color stored as 4 unsigned char
	Vector4uc color;
	Vector3f normal;
};

struct Triangle
{
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;

	Triangle() : idx0{0}, idx1{0}, idx2{0} {}

	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) : idx0(_idx0), idx1(_idx1), idx2(_idx2) {}
};

class SimpleMesh
{
public:
	SimpleMesh() {}

	SimpleMesh transformMesh(const Matrix4f &transformation)
	{
		SimpleMesh transformedMesh;

		Matrix3f rotation = transformation.block<3, 3>(0, 0);

		// Transform vertices and normals
		for (const Vertex &v : m_vertices)
		{
			Vertex transformedVertex;

			// Transform the position
			transformedVertex.position = transformation * v.position;

			// Transform the normal
			transformedVertex.normal = rotation * v.normal;

			// Return the color
			transformedVertex.color = v.color;

			transformedMesh.addVertex(transformedVertex);
		}

		// Same triangles
		for (const Triangle &t : m_triangles)
		{
			transformedMesh.addTriangle(t);
		}

		return transformedMesh;
	}

	/**
	 * Constructs a mesh from the current color and depth image.
	 */
	SimpleMesh(VirtualSensor &sensor, const Matrix4f &cameraPose, float edgeThreshold = 0.01f)
	{
		// Get ptr to the current depth frame.
		// Depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight()).
		float *depthMap = sensor.getDepth();
		// Get ptr to the current color frame.
		// Color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight()).
		BYTE *colorMap = sensor.getColorRGBX();

		// Get depth intrinsics.
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// Compute inverse depth extrinsics.
		Matrix4f depthExtrinsicsInv = sensor.getDepthExtrinsics().inverse();

		// Compute inverse camera pose (mapping from camera CS to world CS).
		Matrix4f cameraPoseInverse = cameraPose.inverse();

		// Compute vertices with back-projection.
		m_vertices.resize(sensor.getDepthImageWidth() * sensor.getDepthImageHeight());
		// For every pixel row.
		for (unsigned int v = 0; v < sensor.getDepthImageHeight(); ++v)
		{
			// For every pixel in a row.
			for (unsigned int u = 0; u < sensor.getDepthImageWidth(); ++u)
			{
				unsigned int idx = v * sensor.getDepthImageWidth() + u; // linearized index
				float depth = depthMap[idx];
				if (depth == MINF)
				{
					m_vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					m_vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
				else
				{
					// Back-projection and tranformation to world space.
					m_vertices[idx].position = cameraPoseInverse * depthExtrinsicsInv * Vector4f((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth, 1.0f);

					// Project position to color map.
					Vector3f proj = sensor.getColorIntrinsics() * (sensor.getColorExtrinsics() * cameraPose * m_vertices[idx].position).block<3, 1>(0, 0);
					proj /= proj.z(); // dehomogenization
					unsigned int uCol = (unsigned int)std::floor(proj.x());
					unsigned int vCol = (unsigned int)std::floor(proj.y());
					if (uCol >= sensor.getColorImageWidth())
						uCol = sensor.getColorImageWidth() - 1;
					if (vCol >= sensor.getColorImageHeight())
						vCol = sensor.getColorImageHeight() - 1;
					unsigned int idxCol = vCol * sensor.getColorImageWidth() + uCol; // linearized index color
																					 // unsigned int idxCol = idx; // linearized index color

					// Write color to vertex.
					m_vertices[idx].color = Vector4uc(colorMap[4 * idxCol + 0], colorMap[4 * idxCol + 1], colorMap[4 * idxCol + 2], colorMap[4 * idxCol + 3]);
				}
			}
		}

		// Compute triangles (faces).
		m_triangles.reserve((sensor.getDepthImageHeight() - 1) * (sensor.getDepthImageWidth() - 1) * 2);
		for (unsigned int i = 0; i < sensor.getDepthImageHeight() - 1; i++)
		{
			for (unsigned int j = 0; j < sensor.getDepthImageWidth() - 1; j++)
			{
				unsigned int i0 = i * sensor.getDepthImageWidth() + j;
				unsigned int i1 = (i + 1) * sensor.getDepthImageWidth() + j;
				unsigned int i2 = i * sensor.getDepthImageWidth() + j + 1;
				unsigned int i3 = (i + 1) * sensor.getDepthImageWidth() + j + 1;

				bool valid0 = m_vertices[i0].position.allFinite();
				bool valid1 = m_vertices[i1].position.allFinite();
				bool valid2 = m_vertices[i2].position.allFinite();
				bool valid3 = m_vertices[i3].position.allFinite();

				if (valid0 && valid1 && valid2)
				{
					float d0 = (m_vertices[i0].position - m_vertices[i1].position).norm();
					float d1 = (m_vertices[i0].position - m_vertices[i2].position).norm();
					float d2 = (m_vertices[i1].position - m_vertices[i2].position).norm();
					if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
						addFace(i0, i1, i2);
				}
				if (valid1 && valid2 && valid3)
				{
					float d0 = (m_vertices[i3].position - m_vertices[i1].position).norm();
					float d1 = (m_vertices[i3].position - m_vertices[i2].position).norm();
					float d2 = (m_vertices[i1].position - m_vertices[i2].position).norm();
					if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2)
						addFace(i1, i3, i2);
				}
			}
		}
	}

	void clear()
	{
		m_vertices.clear();
		m_triangles.clear();
	}

	unsigned int addVertex(Vertex &vertex)
	{
		unsigned int vId = (unsigned int)m_vertices.size();
		m_vertices.push_back(vertex);
		return vId;
	}

	unsigned int addFace(unsigned int idx0, unsigned int idx1, unsigned int idx2)
	{
		unsigned int fId = (unsigned int)m_triangles.size();
		Triangle triangle(idx0, idx1, idx2);
		m_triangles.push_back(triangle);
		return fId;
	}

	void addTriangle(Triangle triangle)
	{
		m_triangles.push_back(triangle);
	}

	std::vector<Vertex> &getVertices()
	{
		return m_vertices;
	}

	const std::vector<Vertex> &getVertices() const
	{
		return m_vertices;
	}

	std::vector<Triangle> &getTriangles()
	{
		return m_triangles;
	}

	const std::vector<Triangle> &getTriangles() const
	{
		return m_triangles;
	}

	void transform(const Matrix4f &transformation)
	{
		for (Vertex &v : m_vertices)
		{
			v.position = transformation * v.position;
		}
	}

	bool loadMesh(const std::string &filename)
	{
		// Read off file (Important: Only .off files are supported).
		m_vertices.clear();
		m_triangles.clear();

		std::ifstream file(filename);
		if (!file.is_open())
		{
			std::cout << "Mesh file wasn't read successfully." << std::endl;
			return false;
		}

		// First line should say 'COFF'.
		char string1[5];
		file >> string1;

		// Read header.
		unsigned int numV = 0;
		unsigned int numP = 0;
		unsigned int numE = 0;
		file >> numV >> numP >> numE;

		m_vertices.reserve(numV);
		m_triangles.reserve(numP);

		// Read vertices.
		if (std::string(string1).compare("NCOFF") == 0)
		{
			// We have color information.
			for (unsigned int i = 0; i < numV; i++)
			{
				Vertex v;
				file >> v.position.x() >> v.position.y() >> v.position.z();
				v.position.w() = 1.f;
				// Colors are stored as integers. We need to convert them.
				Vector4i colorInt;
				file >> colorInt.x() >> colorInt.y() >> colorInt.z() >> colorInt.w();
				v.color = Vector4uc((unsigned char)colorInt.x(), (unsigned char)colorInt.y(), (unsigned char)colorInt.z(), (unsigned char)colorInt.w());
				file >> v.normal.x() >> v.normal.y() >> v.normal.z();
				m_vertices.push_back(v);
			}
		}
		else if (std::string(string1).compare("COFF") == 0)
		{
			// We have color information.
			for (unsigned int i = 0; i < numV; i++)
			{
				Vertex v;
				file >> v.position.x() >> v.position.y() >> v.position.z();
				v.position.w() = 1.f;
				// Colors are stored as integers. We need to convert them.
				Vector4i colorInt;
				file >> colorInt.x() >> colorInt.y() >> colorInt.z() >> colorInt.w();
				v.color = Vector4uc((unsigned char)colorInt.x(), (unsigned char)colorInt.y(), (unsigned char)colorInt.z(), (unsigned char)colorInt.w());
				m_vertices.push_back(v);
			}
		}
		else if (std::string(string1).compare("OFF") == 0)
		{
			// We only have vertex information.
			for (unsigned int i = 0; i < numV; i++)
			{
				Vertex v;
				file >> v.position.x() >> v.position.y() >> v.position.z();
				v.position.w() = 1.f;
				v.color.x() = 0;
				v.color.y() = 0;
				v.color.z() = 0;
				v.color.w() = 255;
				m_vertices.push_back(v);
			}
		}
		else
		{
			std::cout << "Incorrect mesh file type." << std::endl;
			return false;
		}

		// Read faces (i.e. triangles).
		for (unsigned int i = 0; i < numP; i++)
		{
			unsigned int num_vs;
			file >> num_vs;
			ASSERT((num_vs == 3) && "We can only read triangular mesh.");

			Triangle t;
			file >> t.idx0 >> t.idx1 >> t.idx2;
			m_triangles.push_back(t);
		}

		return true;
	}

	bool writeMesh(const std::string &filename)
	{
		// Write off file.
		std::ofstream outFile(filename);
		if (!outFile.is_open())
			return false;

		// Write header.
		outFile << "COFF" << std::endl;
		outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

		// Save vertices.
		for (unsigned int i = 0; i < m_vertices.size(); i++)
		{
			const auto &vertex = m_vertices[i];
			if (vertex.position.allFinite())
				outFile << vertex.position.x() << " " << vertex.position.y() << " " << vertex.position.z() << " "
						<< int(vertex.color.x()) << " " << int(vertex.color.y()) << " " << int(vertex.color.z()) << " " << int(vertex.color.w()) << std::endl;
			else
				outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
		}

		// Save faces.
		for (unsigned int i = 0; i < m_triangles.size(); i++)
		{
			outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2 << std::endl;
		}

		// Close file.
		outFile.close();

		return true;
	}

	/**
	 * Joins two meshes together by putting them into the common mesh and transforming the vertex positions of
	 * mesh1 with transformation 'pose1to2'.
	 */
	static SimpleMesh joinMeshes(const SimpleMesh &mesh1,
								 const SimpleMesh &mesh2,
								 Matrix4f pose1to2 = Matrix4f::Identity(),
								 bool saveColor = false)
	{
		SimpleMesh joinedMesh;
		const auto &vertices1 = mesh1.getVertices();
		const auto &triangles1 = mesh1.getTriangles();
		const auto &vertices2 = mesh2.getVertices();
		const auto &triangles2 = mesh2.getTriangles();

		auto &joinedVertices = joinedMesh.getVertices();
		auto &joinedTriangles = joinedMesh.getTriangles();

		const unsigned nVertices1 = vertices1.size();
		const unsigned nVertices2 = vertices2.size();
		joinedVertices.reserve(nVertices1 + nVertices2);

		const unsigned nTriangles1 = triangles1.size();
		const unsigned nTriangles2 = triangles2.size();
		joinedTriangles.reserve(nVertices1 + nVertices2);

		// Add all vertices (we need to transform vertices of mesh 1).
		// Change colors for visualization.
		for (int i = 0; i < nVertices1; ++i)
		{
			const auto &v1 = vertices1[i];
			Vertex v;
			v.position = pose1to2 * v1.position;
			// Substitute color if we want to emphasize difference on aligned meshes.
			if (saveColor)
			{
				v.color = v1.color;
			}
			else
			{
				v.color = Vector4uc(0, 255, 0, 255);
			}
			joinedVertices.push_back(v);
		}
		for (int i = 0; i < nVertices2; ++i)
		{
			const auto &v2 = vertices2[i];
			Vertex v;
			v.position = v2.position;
			if (saveColor)
			{
				v.color = v2.color;
			}
			else
			{
				v.color = Vector4uc(255, 0, 0, 255);
			}
			joinedVertices.push_back(v);
		}
		// Add all faces (the indices of the second mesh need to be added an offset).
		for (int i = 0; i < nTriangles1; ++i)
			joinedTriangles.push_back(triangles1[i]);
		for (int i = 0; i < nTriangles2; ++i)
		{
			const auto &t2 = triangles2[i];
			Triangle t{t2.idx0 + nVertices1, t2.idx1 + nVertices1, t2.idx2 + nVertices1};
			joinedTriangles.push_back(t);
		}

		return joinedMesh;
	}

	/**
	 * Generates a camera object with a given pose.
	 */
	static SimpleMesh camera(const Matrix4f &cameraPose, float scale = 1.f, Vector4uc color = {255, 0, 0, 255})
	{
		SimpleMesh mesh;
		Matrix4f cameraToWorld = cameraPose.inverse();

		// These are precomputed values for sphere aproximation.
		std::vector<double> vertexComponents = {25, 25, 0, -50, 50, 100, 49.99986, 49.9922, 99.99993, -24.99998, 25.00426, 0.005185,
												25.00261, -25.00023, 0.004757, 49.99226, -49.99986, 99.99997, -50, -50, 100, -25.00449, -25.00492, 0.019877};
		const std::vector<unsigned> faceIndices = {1, 2, 3, 2, 0, 3, 2, 5, 4, 4, 0, 2, 5, 6, 7, 7, 4, 5, 6, 1, 7, 1, 3, 7, 3, 0, 4, 7, 3, 4, 5, 2, 1, 5, 1, 6};

		// Add vertices.
		for (int i = 0; i < 8; ++i)
		{
			Vertex v;
			v.position = cameraToWorld * Vector4f{scale * float(vertexComponents[3 * i + 0]), scale * float(vertexComponents[3 * i + 1]), scale * float(vertexComponents[3 * i + 2]), 1.f};
			v.color = color;
			mesh.addVertex(v);
		}

		// Add faces.
		for (int i = 0; i < 12; ++i)
		{
			mesh.addFace(faceIndices[3 * i + 0], faceIndices[3 * i + 1], faceIndices[3 * i + 2]);
		}

		return mesh;
	}

	void downSampleMesh(float ratio, std::string strategy)
	{

		unsigned int numSamples = std::round(m_triangles.size() * ratio);
		std::vector<Triangle> sampled_triangles;

		if (strategy == "FULL")
			return;
		else if (strategy == "UNIF")
			sampled_triangles = sampleUniformPointsFromMesh(m_vertices, m_triangles, numSamples);
		else if (strategy == "NSPACE")
			sampled_triangles = sampleNormalSpaceFromMesh(m_vertices, m_triangles, numSamples);
		else
			std::cerr << "Unknown sampling strategy." << std::endl;

		std::vector<Vertex> new_m_vertices;
		std::vector<Triangle> new_m_triangles;

		// Step 1: Collect all unique vertex indices from sampled_triangles
		std::unordered_set<unsigned int> unique_indices;
		for (const auto &triangle : sampled_triangles)
		{
			unique_indices.insert(triangle.idx0);
			unique_indices.insert(triangle.idx1);
			unique_indices.insert(triangle.idx2);
		}

		// Step 2: Create a mapping from old vertex indices to new vertex indices
		std::unordered_map<unsigned int, unsigned int> old_to_new_index;
		new_m_vertices.clear();
		unsigned int new_index = 0;
		for (unsigned int old_index : unique_indices)
		{
			old_to_new_index[old_index] = new_index++;
			new_m_vertices.push_back(m_vertices[old_index]);
		}

		// Step 3: Populate new_m_triangles using the mapping
		new_m_triangles.clear();
		for (const auto &triangle : sampled_triangles)
		{
			Triangle new_triangle;
			new_triangle.idx0 = old_to_new_index[triangle.idx0];
			new_triangle.idx1 = old_to_new_index[triangle.idx1];
			new_triangle.idx2 = old_to_new_index[triangle.idx2];
			new_m_triangles.push_back(new_triangle);
		}

		m_vertices = std::move(new_m_vertices);
		m_triangles = std::move(new_m_triangles);
	}

	Eigen::Vector3f _computeTriangleNormal(const Eigen::Vector3f &v0,
										   const Eigen::Vector3f &v1,
										   const Eigen::Vector3f &v2)
	{
		Eigen::Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();
		return normal;
	}

	std::pair<float, float> _convertToSphericalCoords(const Eigen::Vector3f &normal)
	{
		float azimuth = std::atan2(normal.y(), normal.x()); // Angle in the x-y plane
		float elevation = std::acos(normal.z());			// Angle from the z-axis
		return {azimuth, elevation};
	}

	std::vector<Triangle> sampleNormalSpaceFromMesh(const std::vector<Vertex> &vertices,
													const std::vector<Triangle> &triangles,
													unsigned int numSamples,
													int azimuthBins = 36,
													int elevationBins = 36)
	{
		std::vector<Triangle> samples;
		std::unordered_map<int, std::vector<size_t>> normalBuckets;

		for (size_t i = 0; i < triangles.size(); ++i)
		{
			const auto &t = triangles[i];
			Eigen::Vector3f normal = _computeTriangleNormal(vertices[t.idx0].position.head(3),
															vertices[t.idx1].position.head(3),
															vertices[t.idx2].position.head(3));

			auto [azimuth, elevation] = _convertToSphericalCoords(normal);

			int azimuthIndex = static_cast<int>((azimuth + M_PI) / (2 * M_PI) * azimuthBins) % azimuthBins;
			int elevationIndex = static_cast<int>(elevation / M_PI * elevationBins);

			int bucketIndex = azimuthIndex * elevationBins + elevationIndex;

			normalBuckets[bucketIndex].push_back(i);
		}

		std::random_device rd;
		std::mt19937 gen(rd());

		while (samples.size() < numSamples)
		{
			for (auto &bucket : normalBuckets)
			{
				if (bucket.second.empty())
					continue;
				std::uniform_int_distribution<> dist(0, bucket.second.size() - 1);
				size_t triangleIdx = bucket.second[dist(gen)];
				samples.push_back(triangles[triangleIdx]);
				if (samples.size() >= numSamples)
					break;
			}
		}

		return samples;
	}

	float _computeTriangleArea(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2)
	{
		return ((v1 - v0).cross(v2 - v0)).norm() / 2.0f;
	}

	std::vector<Triangle> sampleUniformPointsFromMesh(const std::vector<Vertex> &vertices, const std::vector<Triangle> &triangles, unsigned int numSamples)
	{
		std::vector<Triangle> samples;

		std::vector<float> areas(triangles.size());
		float totalArea = 0.0f;
		for (size_t i = 0; i < triangles.size(); ++i)
		{
			const auto &t = triangles[i];
			float area = _computeTriangleArea(vertices[t.idx0].position.head(3),
											  vertices[t.idx1].position.head(3),
											  vertices[t.idx2].position.head(3));
			areas[i] = area;
			totalArea += area;
		}

		std::discrete_distribution<> areaDist(areas.begin(), areas.end());

		std::random_device rd;
		std::mt19937 gen(rd());
		for (unsigned int i = 0; i < numSamples; ++i)
		{
			int triangleIdx = areaDist(gen);
			const auto &t = triangles[triangleIdx];

			samples.push_back(t);
		}

		return samples;
	}

private:
	std::vector<Vertex> m_vertices;
	std::vector<Triangle> m_triangles;
};
