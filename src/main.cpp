#include <iostream>
#include <fstream>
#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "ICPConfiguration.h"
#include "PointCloud.h"
#include "DataLoader.h"
#include "Utils.h"
#include "Evaluator.h"

#define MESH_ENABLED 1
#define OPEN3D_ENABLED 1
#ifndef OPEN3D_ENABLED 1
#include <Open3D/Open3D.h>
#endif

int runShapeICP(const ICPConfiguration &config, const std::string directoryPath)
{
	// Reproducibility
	std::mt19937 rng(42);

	// Load the path to meshes.
	auto dataloader = createDataloader(directoryPath);
	dataloader->loadMeshPaths(directoryPath);

	ICPOptimizer *optimizer = createOptimizer(config);

	Evaluator evaluator(config);
	if (config.evaluate_rmse_naive || config.evaluate_rmse_nn || config.evaluate_transforms)
	{
		optimizer->setEvaluator(evaluator);
	}

	Matrix4f gt_trans;			// True value of the transformation.
	SimpleMesh sourceMesh;		// Loaded mesh.
	SimpleMesh targetMesh;		// Mesh transformed by a random transformation.
	Matrix4f estimatedPose;		// Estimated transformation;
	std::string filenameOutput; // Where to write output.

	for (size_t i = 0; i < dataloader->size(); ++i)
	{
		gt_trans = getRandomTransformation(rng, 45, 0.5);
		dataloader->createMeshes(i, sourceMesh, targetMesh, gt_trans);

		std::vector<std::vector<double>> metric;
		filenameOutput = formatString({"./", dataloader->getName(i), "_joined.off"});
		estimatedPose = alignShapes(sourceMesh,
									targetMesh,
									optimizer,
									filenameOutput,
									metric);

		// Save error metric
		std::ofstream file;
		file.open("./metric.txt");
		for (int i = 0; i < metric.size(); i++)
		{
			file << i + 1 << "," << metric[i][0] << "," << metric[i][1] << "," << metric[i][2] << std::endl;
		}
		file.close();

#ifndef OPEN3D_ENABLED 1
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

	std::vector<std::vector<double>> avg_metric;

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
		optimizer->estimatePose(source, target, currentCameraToWorld, avg_metric);

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

#ifndef MESH_ENABLED 1
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

	std::ofstream file;
	file.open("./metric_room.txt");
	for (int k = 0; k < avg_metric.size(); k++)
	{
		file << k + 1 << "," << avg_metric[k][0] / i << "," << avg_metric[k][1] / i << "," << avg_metric[k][2] / i << std::endl;
	}
	file.close();

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
	const std::string directoryPath = std::string("../Data/greyc_partial_debug/"); // Two partially complete meshes.
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
