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

namespace fs = std::filesystem;

#define OPEN3D_ENABLED
#define MESH_ENABLED

int runShapeICP(const ICPConfiguration &config)
{
	// Reproducibility
	std::mt19937 rng(42);

	// Load the path to meshes.
	auto dataloader = createDataloader(config.dataDir);
	dataloader->loadMeshPaths(config.dataDir);

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

	Matrix4f gtTransform;	// True value of the transformation.
	SimpleMesh sourceMesh;	// Loaded mesh.
	SimpleMesh targetMesh;	// Mesh transformed by a random transformation.
	Matrix4f estimatedPose; // Estimated transformation;

	fs::path outputDir;
	fs::path filenameOutputColor; // Where to write output.
	fs::path filenameOutputRG;	  // Where to write output.

	for (size_t i = 0; i < dataloader->size(); ++i)
	{
		// Prepare data
		gtTransform = getRandomTransformation(rng, 45, 0.5);
		dataloader->createMeshes(i, sourceMesh, targetMesh, gtTransform);

		// Prepare where to write all metrics and meshes.
		outputDir = "../results" / fs::path{config.experimentName} / fs::path{dataloader->getName(i)};
		fs::create_directories(outputDir);
		filenameOutputColor = outputDir / fs::path{"mesh_joined_color.off"};
		filenameOutputRG = outputDir / fs::path{"mesh_joined_rg.off"};

		// Estimate pose.
		estimatedPose = alignShapes(sourceMesh,
									targetMesh,
									gtTransform,
									optimizer);
		// Write down meshes.
		writeShapeMesh(sourceMesh,
					   targetMesh,
					   estimatedPose,
					   filenameOutputColor,
					   filenameOutputRG);

		// Write down metrics.
		evaluator.write(outputDir);
		evaluator.reset();

#ifdef OPEN3D_ENABLED
		if (config.visualize)
		{
			visualize(filenameOutputRG);
		}
#endif
	}

	delete optimizer;

	return 0;
}

int runSequenceICP(const ICPConfiguration &config)
{

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(config.dataDir))
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

	fs::path outputDir = "../results " / fs::path{config.experimentName};
	fs::create_directories(outputDir);

	// We store the estimated camera poses.
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	Matrix4f initTrajectory = sensor.getTrajectory();
	Matrix4f groundTruth;
	Matrix4f currentTrajectory;

	int i = 0;
	const int iMax = 30;
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

		currentTrajectory = sensor.getTrajectory();
		groundTruth = (initTrajectory * currentTrajectory.inverse());
		optimizer->estimatePose(source,
								target,
								currentCameraToWorld,
								groundTruth);

		evaluator.write(outputDir / fs::path{std::to_string(sensor.getCurrentFrameCnt())});
		evaluator.reset();

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;

#ifdef MESH_ENABLED
		if (i % 3 == 0)
			writeRoomMesh(sensor, currentCameraPose, outputDir / fs::path{"meshes"});
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

	// Load config from file.
	ICPConfiguration config;
	config.loadFromYaml(argv[1]);
	config.show();

	int result = 0;
	if (config.runShapeICP)
	{
		result = runShapeICP(config);
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
