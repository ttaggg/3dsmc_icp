#include <Eigen/Dense>
#include <random>
#include <cmath>

ICPOptimizer *createOptimizer(const ICPConfiguration &config)
{
    ICPOptimizer *optimizer = nullptr;

    if (config.useLinearICP)
    {
        optimizer = new LinearICPOptimizer();
    }
    else
    {
        optimizer = new CeresICPOptimizer();
    }

    optimizer->setCorrespondenceMethod(config.correspondenceMethod, config.useColors);
    optimizer->setMatchingMaxDistance(config.matchingMaxDistance);
    optimizer->usePointToPointConstraints(config.usePointToPoint, config.weightPointToPoint);
    optimizer->usePointToPlaneConstraints(config.usePointToPlane, config.weightPointToPlane);
    optimizer->useSymmetricConstraints(config.useSymmetric, config.weightSymmetric);
    optimizer->setNbOfIterations(config.nbOfIterations);

    return optimizer;
}