#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>

/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3f &input, T *output)
{
    output[0] = T(input[0]);
    output[1] = T(input[1]);
    output[2] = T(input[2]);
}

/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement
{
public:
    explicit PoseIncrement(T *const array) : m_array{array} {}

    void setZero()
    {
        for (int i = 0; i < 6; ++i)
            m_array[i] = T(0);
    }

    T *getData() const
    {
        return m_array;
    }

    /**
     * Applies the pose increment onto the input point and produces transformed output point.
     * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
     * beforehand).
     */
    void apply(T *inputPoint, T *outputPoint) const
    {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        const T *rotation = m_array;
        const T *translation = m_array + 3;

        T temp[3];
        ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

        outputPoint[0] = temp[0] + translation[0];
        outputPoint[1] = temp[1] + translation[1];
        outputPoint[2] = temp[2] + translation[2];
    }

    /**
     * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
     * transformation 4x4 matrix.
     */
    static Matrix4f convertToMatrix(const PoseIncrement<double> &poseIncrement)
    {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        double *pose = poseIncrement.getData();
        double *rotation = pose;
        double *translation = pose + 3;

        // Convert the rotation from SO3 to matrix notation (with column-major storage).
        double rotationMatrix[9];
        ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

        // Create the 4x4 transformation matrix.
        Matrix4f matrix;
        matrix.setIdentity();
        matrix(0, 0) = float(rotationMatrix[0]);
        matrix(0, 1) = float(rotationMatrix[3]);
        matrix(0, 2) = float(rotationMatrix[6]);
        matrix(0, 3) = float(translation[0]);
        matrix(1, 0) = float(rotationMatrix[1]);
        matrix(1, 1) = float(rotationMatrix[4]);
        matrix(1, 2) = float(rotationMatrix[7]);
        matrix(1, 3) = float(translation[1]);
        matrix(2, 0) = float(rotationMatrix[2]);
        matrix(2, 1) = float(rotationMatrix[5]);
        matrix(2, 2) = float(rotationMatrix[8]);
        matrix(2, 3) = float(translation[2]);

        return matrix;
    }

private:
    T *m_array;
};

/**
 * Optimization constraints.
 */
class PointToPointConstraint
{
public:
    PointToPointConstraint(const Vector3f &sourcePoint, const Vector3f &targetPoint, const float weight) : m_sourcePoint{sourcePoint},
                                                                                                           m_targetPoint{targetPoint},
                                                                                                           m_weight{weight}
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        PoseIncrement<T> poseIncrement(const_cast<T *>(pose));

        T source[3];
        fillVector(m_sourcePoint, source);

        T target[3];
        fillVector(m_targetPoint, target);

        T newSourse[3];
        poseIncrement.apply(source, newSourse);

        residuals[0] = T(m_weight) * (newSourse[0] - target[0]);
        residuals[1] = T(m_weight) * (newSourse[1] - target[1]);
        residuals[2] = T(m_weight) * (newSourse[2] - target[2]);

        return true;
    }

    static ceres::CostFunction *create(const Vector3f &sourcePoint, const Vector3f &targetPoint, const float weight)
    {
        return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
            new PointToPointConstraint(sourcePoint, targetPoint, weight));
    }

protected:
    const Vector3f m_sourcePoint;
    const Vector3f m_targetPoint;
    const float m_weight;
    const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint
{
public:
    PointToPlaneConstraint(const Vector3f &sourcePoint, const Vector3f &targetPoint, const Vector3f &targetNormal, const float weight) : m_sourcePoint{sourcePoint},
                                                                                                                                         m_targetPoint{targetPoint},
                                                                                                                                         m_targetNormal{targetNormal},
                                                                                                                                         m_weight{weight}
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        PoseIncrement<T> poseIncrement(const_cast<T *>(pose));

        T normal[3];
        fillVector(m_targetNormal, normal);

        T source[3];
        fillVector(m_sourcePoint, source);

        T target[3];
        fillVector(m_targetPoint, target);

        T newSourse[3];
        poseIncrement.apply(source, newSourse);

        residuals[0] = T(m_weight) * (normal[0] * (target[0] - newSourse[0]) + normal[1] * (target[1] - newSourse[1]) + normal[2] * (target[2] - newSourse[2]));

        return true;
    }

    static ceres::CostFunction *create(const Vector3f &sourcePoint, const Vector3f &targetPoint, const Vector3f &targetNormal, const float weight)
    {
        return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
            new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight));
    }

protected:
    const Vector3f m_sourcePoint;
    const Vector3f m_targetPoint;
    const Vector3f m_targetNormal;
    const float m_weight;
    const float LAMBDA = 1.0f;
};

class SymmetricConstraint
{
public:
    SymmetricConstraint(const Vector3f &sourcePoint, const Vector3f &targetPoint,
                        const Vector3f &sourceNormal, const Vector3f &targetNormal,
                        const float weight)
        : m_sourcePoint{sourcePoint},
          m_targetPoint{targetPoint},
          m_sourceNormal{sourceNormal},
          m_targetNormal{targetNormal},
          m_weight{weight}
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        PoseIncrement<T> poseIncrement(const_cast<T *>(pose));

        T sourceNormal[3];
        fillVector(m_sourceNormal, sourceNormal);

        T targetNormal[3];
        fillVector(m_targetNormal, targetNormal);

        T source[3];
        fillVector(m_sourcePoint, source);

        T target[3];
        fillVector(m_targetPoint, target);

        T newSource[3];
        poseIncrement.apply(source, newSource);

        T normalSum[3] = {
            sourceNormal[0] + targetNormal[0],
            sourceNormal[1] + targetNormal[1],
            sourceNormal[2] + targetNormal[2]};

        residuals[0] = T(m_weight) * (normalSum[0] * (newSource[0] - target[0]) +
                                      normalSum[1] * (newSource[1] - target[1]) +
                                      normalSum[2] * (newSource[2] - target[2]));

        return true;
    }

    static ceres::CostFunction *create(const Vector3f &sourcePoint, const Vector3f &targetPoint,
                                       const Vector3f &sourceNormal, const Vector3f &targetNormal,
                                       const float weight)
    {
        return new ceres::AutoDiffCostFunction<SymmetricConstraint, 1, 6>(
            new SymmetricConstraint(sourcePoint, targetPoint, sourceNormal, targetNormal, weight));
    }

protected:
    const Vector3f m_sourcePoint;
    const Vector3f m_targetPoint;
    const Vector3f m_sourceNormal;
    const Vector3f m_targetNormal;
    const float m_weight;
};
