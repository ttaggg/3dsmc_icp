#include <stddef.h>			  // for size_t
#include <iostream>			  // for char_traits
#include <memory>			  // for unique_ptr
#include <random>			  // for mt19937
#include <string>			  // for basic_string
#include "DataLoader.h"		  // for DataLoader
#include "Eigen.h"			  // for vector
#include "Evaluator.h"		  // for Evaluator
#include "ICPConfiguration.h" // for ICPConfiguration
#include "ICPOptimizer.h"	  // for ICPOptimizer
#include "PointCloud.h"		  // for PointCloud
#include "SimpleMesh.h"		  // for SimpleMesh
#include "Utils.h"			  // for createOptimizer
#include "VirtualSensor.h"	  // for VirtualSensor

#define OPEN3D_ENABLED
#define MESH_ENABLED

int runShapeICP(const ICPConfiguration &config, const std::string directoryPath)
{
	// Reproducibility
	std::mt19937 rng(42);

	// Load the path to meshes.
	auto dataloader = createDataloader(directoryPath);
	dataloader->loadMeshPaths(directoryPath);

	ICPOptimizer *optimizer = createOptimizer(config);

	Evaluator evaluator(config);
	bool evaluate = (config.evaluateTime ||
					 config.evaluateTransforms ||
					 config.evaluateRMSENaive ||
					 config.evaluateRMSENearest ||
					 config.evaluateRMSENearestPlane);
	if (evaluate)
	{
		optimizer->setEvaluator(&evaluator);
	}

	Matrix4f gtTransform;		// True value of the transformation.
	SimpleMesh sourceMesh;		// Loaded mesh.
	SimpleMesh targetMesh;		// Mesh transformed by a random transformation.
	Matrix4f estimatedPose;		// Estimated transformation;
	std::string filenameOutput; // Where to write output.

	for (size_t i = 0; i < dataloader->size(); ++i)
	{
		gtTransform = getRandomTransformation(rng, 45, 0.5);
		dataloader->createMeshes(i, sourceMesh, targetMesh, gtTransform);

		filenameOutput = formatString({"./", dataloader->getName(i), "_joined.off"});
		estimatedPose = alignShapes(sourceMesh,
									targetMesh,
									gtTransform,
									optimizer,
									filenameOutput);

		evaluator.write(config.outputDir,
						config.experimentName,
						dataloader->getName(i));
		evaluator.reset();

#ifdef OPEN3D_ENABLED
		if (config.visualize)
		{
			visualize(filenameOutput);
		}
#endif
	}

	delete optimizer;

	return 0;
}

int runSequenceICP(const ICPConfiguration &config)
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
	PointCloud target{
		sensor.getDepth(),
		sensor.getDepthIntrinsics(),
		sensor.getDepthExtrinsics(),
		sensor.getDepthImageWidth(),
		sensor.getDepthImageHeight(),
		sensor.getColorRGBX(),
	};

	// Setup the optimizer.
	ICPOptimizer *optimizer = createOptimizer(config);

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());

	// std::vector<std::vector<double>> avg_metric;

	int i = 0;
	const int iMax = 50;
	while (sensor.processNextFrame() && i <= iMax)
	{
		float *depthMap = sensor.getDepth();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{sensor.getDepth(),
						  sensor.getDepthIntrinsics(),
						  sensor.getDepthExtrinsics(),
						  sensor.getDepthImageWidth(),
						  sensor.getDepthImageHeight(),
						  sensor.getColorRGBX(),
						  8};

		// TODO(oleg): fix this in the next commits.
		Matrix4f dummyGroundTruth = Matrix4f::Identity();
		optimizer->estimatePose(source, target, currentCameraToWorld, dummyGroundTruth);

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

#ifdef MESH_ENABLED
		if (i % 3 == 0)
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
#endif

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

	// TODO(oleg): directory should be in argv.
	const std::string directoryPath = std::string("../Data/greyc_partial/"); // Two partially complete meshes.
	// const std::string directoryPath = std::string("../Data/greyc_debug/"); // Two complete meshes.

	// Load config from file.
	ICPConfiguration config;
	config.loadFromYaml(argv[1]);
	config.show();

	int result = 0;
	if (config.runShapeICP)
	{
		result = runShapeICP(config, directoryPath);
	}
	else if (config.runSequenceICP)
	{
		result = runSequenceICP(config);
	}
	else
	{
		std::cerr << "Set the task to either runShapeICP or runSequenceICP." << std::endl;
		return -1;
	}

	return result;
}
