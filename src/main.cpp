#include <iostream>
#include <format>
#include <fstream>
#include <Open3D/Open3D.h>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ICPOptimizer.h"
#include "ICPConfiguration.h"
#include "PointCloud.h"
#include "MeshDataLoader.h"
#include "Utils.h"

int runShapeICP(const ICPConfiguration &config)
{
	// Reproducibility
	std::mt19937 rng(42);

	// Load the source mesh.
	const std::string directoryPath = std::string("../Data/greyc_debug/");
	MeshDataLoader dataloader(directoryPath);

	ICPOptimizer *optimizer = createOptimizer(config);
	Matrix4f gt_trans;			// True value of the transformation.
	SimpleMesh sourceMesh;		// Loaded mesh.
	SimpleMesh targetMesh;		// Mesh transformed by a random transformation.
	Matrix4f estimatedPose;		// Estimated transformation;
	std::string filenameOutput; // Where to write output.

	for (size_t i = 0; i < dataloader.size(); ++i)
	{
		dataloader.getMesh(i, sourceMesh);

		filenameOutput = fmt::format("./{}_joined.off", dataloader.getName(i));

		gt_trans = getRandomTransformation(rng, 45, 0.5);
		targetMesh = sourceMesh.transformMesh(gt_trans);

		estimatedPose = alignShapes(sourceMesh,
									targetMesh,
									optimizer,
									filenameOutput);

		if (config.visualize)
		{
			visualize(filenameOutput);
		}
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
		optimizer->estimatePose(source, target, currentCameraToWorld);

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl
				  << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

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
