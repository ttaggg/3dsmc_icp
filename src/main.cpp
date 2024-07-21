#include <iostream>			  // for char_traits
#include <memory>			  // for unique_ptr
#include <random>			  // for mt19937
#include <string>			  // for basic_string
#include <filesystem>		  // for filesystem paths
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

void prepareOutputDirectories(const fs::path &baseDir,
							  const std::string &experimentName,
							  const std::string &meshName,
							  fs::path &outputDir,
							  fs::path &filenameOutputColor,
							  fs::path &filenameOutputRG)
{
	outputDir = baseDir / experimentName / meshName;
	fs::create_directories(outputDir);
	filenameOutputColor = outputDir / "mesh_joined_color.off";
	filenameOutputRG = outputDir / "mesh_joined_rg.off";
}

bool needsEvaluation(const ICPConfiguration &config)
{
	return config.evaluateTime ||
		   config.evaluateTransforms ||
		   config.evaluateRMSENaive ||
		   config.evaluateRMSENearest ||
		   config.evaluateRMSENearestPlane;
}

void setupOptimizerAndEvaluator(const ICPConfiguration &config, std::unique_ptr<ICPOptimizer> &optimizer, Evaluator &evaluator)
{
	optimizer = std::unique_ptr<ICPOptimizer>(createOptimizer(config));
	if (needsEvaluation(config))
	{
		optimizer->setEvaluator(&evaluator);
	}
}

int runShapeICP(const ICPConfiguration &config)
{
	// Reproducibility for random tranformations.
	std::mt19937 rng(42);

	// Load paths to meshes.
	auto dataloader = createDataloader(config.dataDir);
	dataloader->loadMeshPaths(config.dataDir);

	// Setup optimizer and evaluator.
	std::unique_ptr<ICPOptimizer> optimizer;
	Evaluator evaluator(config);
	setupOptimizerAndEvaluator(config, optimizer, evaluator);

	// Iterate through the dataloader.
	fs::path outputDir, filenameOutputColor, filenameOutputRG;
	for (size_t i = 0; i < dataloader->size(); ++i)
	{
		// Prepare data amd output directories.
		Matrix4f gtTransform = getRandomTransformation(rng, 45, 0.5);
		SimpleMesh sourceMesh, targetMesh;
		dataloader->createMeshes(i,
								 sourceMesh,
								 targetMesh,
								 gtTransform,
								 config.sampling,
								 config.samplingRatio);
		prepareOutputDirectories("../results", config.experimentName, dataloader->getName(i),
								 outputDir, filenameOutputColor, filenameOutputRG);

		// Run ICP.
		Matrix4f estimatedPose = alignShapes(sourceMesh,
											 targetMesh,
											 gtTransform,
											 optimizer.get());

		// Write meshes.
		if (config.writeMeshes)
			writeShapeMesh(sourceMesh,
						   targetMesh,
						   estimatedPose,
						   filenameOutputColor,
						   filenameOutputRG);

		// Write metrics.
		if (needsEvaluation(config))
		{
			evaluator.write(outputDir);
			evaluator.reset();
		}

#ifdef OPEN3D_ENABLED
		// Show mesh in open3D.
		if (config.visualize)
			visualize(filenameOutputRG);
#endif
	}
	return 0;
}

int runSequenceICP(const ICPConfiguration &config)
{

	// Load data
	VirtualSensor sensor;
	if (!sensor.init(config.dataDir))
	{
		std::cerr << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// Reference frame.
	sensor.processNextFrame();
	PointCloud target{
		sensor.getDepth(),
		sensor.getDepthIntrinsics(),
		sensor.getDepthExtrinsics(),
		sensor.getDepthImageWidth(),
		sensor.getDepthImageHeight(),
		sensor.getColorRGBX()};

	// Setup optimizer and evaluator.
	std::unique_ptr<ICPOptimizer> optimizer;
	Evaluator evaluator(config);
	setupOptimizerAndEvaluator(config, optimizer, evaluator);

	// Prepare output directory.
	fs::path outputDir = fs::path{"../results"} / config.experimentName;
	fs::create_directories(outputDir);

	// Iterate through the data.
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	Matrix4f initTrajectory = sensor.getTrajectory();
	int i = 0;
	const int iMax = 30;
	while (sensor.processNextFrame() && i <= iMax)
	{
		PointCloud source{
			sensor.getDepth(),
			sensor.getDepthIntrinsics(),
			sensor.getDepthExtrinsics(),
			sensor.getDepthImageWidth(),
			sensor.getDepthImageHeight(),
			sensor.getColorRGBX(),
			8};

		// Run ICP.
		Matrix4f currentTrajectory = sensor.getTrajectory();
		Matrix4f groundTruth = (initTrajectory * currentTrajectory.inverse());
		optimizer->estimatePose(source, target, currentCameraToWorld, groundTruth);

		// Write meshes.
		if (config.writeMeshes && i % 3 == 0)
		{
			Matrix4f currentCameraPose = currentCameraToWorld.inverse();
			writeRoomMesh(sensor,
						  currentCameraPose,
						  outputDir / "meshes");
		}

		// Write metrics.
		if (needsEvaluation(config))
		{
			evaluator.write(outputDir / std::to_string(sensor.getCurrentFrameCnt()));
			evaluator.reset();
		}

		++i;
	}
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
