#include <cassert>
#include <iostream>
#include <fstream>
#include <Open3D/Open3D.h>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "PointCloud.h"
#include <yaml-cpp/yaml.h>

struct ICPConfiguration
{
	// Task
	bool runShapeICP = false;
	bool runSequenceICP = false;
	// ICP type
	bool useLinearICP = false;
	// ICP objective(s)
	bool usePointToPoint = false;
	bool usePointToPlane = false;
	bool useSymmetric = false;
	// Correspondence method (ANN / PROJ)
	CorrMethod correspondenceMethod = ANN;
	// Other settings
	float matchingMaxDistance = 0.0f;
	int nbOfIterations = 0;

	void loadFromYaml(const std::string &filename)
	{
		YAML::Node config = YAML::LoadFile(filename);
		runShapeICP = config["runShapeICP"].as<bool>(runShapeICP);
		runSequenceICP = config["runSequenceICP"].as<bool>(runSequenceICP);
		useLinearICP = config["useLinearICP"].as<bool>(useLinearICP);
		usePointToPoint = config["usePointToPoint"].as<bool>(usePointToPoint);
		usePointToPlane = config["usePointToPlane"].as<bool>(usePointToPlane);
		useSymmetric = config["useSymmetric"].as<bool>(useSymmetric);
		matchingMaxDistance = config["matchingMaxDistance"].as<float>(matchingMaxDistance);
		nbOfIterations = config["nbOfIterations"].as<int>(nbOfIterations);

		// Custom types.
		std::string method = config["correspondenceMethod"].as<std::string>();
		if (method == "ANN")
		{
			correspondenceMethod = ANN;
		}
		else if (method == "PROJ")
		{
			correspondenceMethod = PROJ;
		}
		else
		{
			throw std::runtime_error("Unknown correspondence method: " + method);
		}
	}
};

int alignBunnyWithICP(const ICPConfiguration &config)
{
	// Load the source and target mesh.
	const std::string filenameSource = std::string("../../Data/bunny_part2_trans.off");
	const std::string filenameTarget = std::string("../../Data/bunny_part1.off");
	const std::string filenameOutput = "./bunny_icp.off";

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource))
	{
		std::cout << "Mesh file wasn't read successfully at location: " << filenameSource << std::endl;
		return -1;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget))
	{
		std::cout << "Mesh file wasn't read successfully at location: " << filenameTarget << std::endl;
		return -1;
	}

	// Estimate the pose from source to target mesh with ICP optimization.
	ICPOptimizer *optimizer = nullptr;
	if (config.useLinearICP)
	{
		optimizer = new LinearICPOptimizer();
	}
	else
	{
		optimizer = new CeresICPOptimizer();
	}

	optimizer->setCorrespondenceMethod(config.correspondenceMethod);
	optimizer->setMatchingMaxDistance(config.matchingMaxDistance);
	optimizer->usePointToPointConstraints(config.usePointToPoint);
	optimizer->usePointToPlaneConstraints(config.usePointToPlane);
	optimizer->useSymmetricConstraints(config.useSymmetric);
	optimizer->setNbOfIterations(config.nbOfIterations);

	PointCloud source{sourceMesh};
	PointCloud target{targetMesh};

	Matrix4f estimatedPose = Matrix4f::Identity();
	optimizer->estimatePose(source, target, estimatedPose);

	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	resultingMesh.writeMesh(filenameOutput);
	std::cout << "Resulting mesh written." << std::endl;

	// Visualize the mesh with Open3D.
	auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

	if (!open3d::io::ReadTriangleMesh(filenameOutput, *mesh))
	{
		std::cerr << "Failed to read mesh from " << filenameOutput << std::endl;
		return 1;
	}

	if (!mesh->HasVertexNormals())
	{
		mesh->ComputeVertexNormals();
	}

	open3d::visualization::DrawGeometries({mesh}, "Mesh Visualization");

	delete optimizer;

	return 0;
}

int reconstructRoom(const ICPConfiguration &config)
{
	std::string filenameIn = std::string("../../Data/rgbd_dataset_freiburg1_xyz/");
	std::string filenameBaseOut = std::string("mesh_");

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	PointCloud target{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};

	// Setup the optimizer.
	ICPOptimizer *optimizer = nullptr;
	if (config.useLinearICP)
	{
		optimizer = new LinearICPOptimizer();
	}
	else
	{
		optimizer = new CeresICPOptimizer();
	}

	optimizer->setCorrespondenceMethod(config.correspondenceMethod);
	optimizer->setMatchingMaxDistance(config.matchingMaxDistance);
	optimizer->usePointToPointConstraints(config.usePointToPoint);
	optimizer->usePointToPlaneConstraints(config.usePointToPlane);
	optimizer->useSymmetricConstraints(config.useSymmetric);
	optimizer->setNbOfIterations(config.nbOfIterations);

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());

	int i = 0;
	const int iMax = 50;
	while (sensor.processNextFrame() && i <= iMax)
	{
		float *depthMap = sensor.getDepth();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8};
		optimizer->estimatePose(source, target, currentCameraToWorld);

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		if (i % 5 == 0)
		{
			// We write out the mesh to file for debugging.
			SimpleMesh currentDepthMesh{sensor, currentCameraPose, 0.1f};
			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

			std::stringstream ss;
			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
			std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
			if (!resultingMesh.writeMesh(ss.str()))
			{
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			}
		}

		i++;
	}

	delete optimizer;

	return 0;
}

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
		return -1;
	}

	// Load config from file.
	ICPConfiguration config;
	try
	{
		config.loadFromYaml(argv[1]);
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}

	if (config.useLinearICP)
	{
		assert(config.usePointToPoint + config.usePointToPlane + config.useSymmetric == 1);
	}
	else
	{
		assert(config.usePointToPoint + config.usePointToPlane + config.useSymmetric > 0);
	}

	int result = 0;
	if (config.runShapeICP)
		result += alignBunnyWithICP(config);
	if (config.runSequenceICP)
		result += reconstructRoom(config);

	return result;
}
