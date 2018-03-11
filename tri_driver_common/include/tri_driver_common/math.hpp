#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <map>

namespace tri_driver_common
{
namespace math
{
///////////////////////////////////////////////////////////////
//// Typedefs for aligned STL containers using Eigen types ////
///////////////////////////////////////////////////////////////

using VectorVector2f
  = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
using VectorVector2d
  = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
using VectorVector3f
  = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
using VectorVector3d
  = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using VectorVector4f
  = std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>;
using VectorVector4d
  = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;
using VectorQuaternionf
  = std::vector<Eigen::Quaternionf,
                Eigen::aligned_allocator<Eigen::Quaternionf>>;
using VectorQuaterniond
  = std::vector<Eigen::Quaterniond,
                Eigen::aligned_allocator<Eigen::Quaterniond>>;
using VectorIsometry3f
  = std::vector<Eigen::Isometry3f,
                Eigen::aligned_allocator<Eigen::Isometry3f>>;
using VectorIsometry3d
  = std::vector<Eigen::Isometry3d,
                Eigen::aligned_allocator<Eigen::Isometry3d>>;
using MapStringVector2f
  = std::map<std::string, Eigen::Vector2f, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2f>>>;
using MapStringVector2d
  = std::map<std::string, Eigen::Vector2d, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2d>>>;
using MapStringVector3f
  = std::map<std::string, Eigen::Vector3f, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3f>>>;
using MapStringVector3d
  = std::map<std::string, Eigen::Vector3d, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>>;
using MapStringVector4f
  = std::map<std::string, Eigen::Vector4f, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4f>>>;
using MapStringVector4d
  = std::map<std::string, Eigen::Vector4d, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4d>>>;
using MapStringQuaternionf
  = std::map<std::string, Eigen::Quaternionf, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaternionf>>>;
using MapStringQuaterniond
  = std::map<std::string, Eigen::Quaterniond, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaterniond>>>;
using MapStringIsometry3f
  = std::map<std::string, Eigen::Isometry3f, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3f>>>;
using MapStringIsometry3d
  = std::map<std::string, Eigen::Isometry3d, std::less<std::string>,
  Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>;

bool Equal(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

bool CloseEnough(const double p1, const double p2, const double threshold);

bool CloseEnough(const Eigen::Vector3d& v1,
                 const Eigen::Vector3d& v2,
                 const double threshold);

Eigen::Vector3d RotateVector(const Eigen::Quaterniond& quat,
                             const Eigen::Vector3d& vec);

Eigen::Vector3d RotateVectorReverse(const Eigen::Quaterniond& quat,
                                    const Eigen::Vector3d& vec);

double EnforceContinuousRevoluteBounds(const double value);

Eigen::VectorXd SafeNormal(const Eigen::VectorXd& vec);

double SquaredNorm(const std::vector<double>& vec);

double Norm(const std::vector<double>& vec);

std::vector<double> Abs(const std::vector<double>& vec);

std::vector<double> Multiply(const std::vector<double>& vec,
                             const double scalar);

std::vector<double> Multiply(const std::vector<double>& vec1,
                             const std::vector<double>& vec2);

std::vector<double> Divide(const std::vector<double>& vec,
                           const double scalar);

std::vector<double> Divide(const std::vector<double>& vec1,
                           const std::vector<double>& vec2);

std::vector<double> Add(const std::vector<double>& vec,
                        const double scalar);

std::vector<double> Add(const std::vector<double>& vec1,
                        const std::vector<double>& vec2);

std::vector<double> Sub(const std::vector<double>& vec,
                        const double scalar);

std::vector<double> Sub(const std::vector<double>& vec1,
                        const std::vector<double>& vec2);

double Sum(const std::vector<double>& vec);

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector);

Eigen::Vector3d Unskew(const Eigen::Matrix3d& matrix);

Eigen::Matrix4d TwistHat(const Eigen::Matrix<double, 6, 1>& twist);

Eigen::Matrix<double, 6, 1> TwistUnhat(const Eigen::Matrix4d& hatted_twist);

Eigen::Matrix<double, 6, 6> AdjointFromTransform(
    const Eigen::Isometry3d& transform);

Eigen::Matrix<double, 6, 1> TransformTwist(
    const Eigen::Isometry3d& transform,
    const Eigen::Matrix<double, 6, 1>& initial_twist);

Eigen::Matrix<double, 6, 1> TwistBetweenTransforms(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end);

Eigen::Matrix3d ExpMatrixExact(const Eigen::Matrix3d& hatted_rot_velocity,
                               const double delta_t);

Eigen::Isometry3d ExpTwist(const Eigen::Matrix<double, 6, 1>& twist,
                           const double delta_t);

double Interpolate(const double p1, const double p2, const double ratio);

double InterpolateContinuousRevolute(const double p1,
                                     const double p2,
                                     const double ratio);

std::vector<double> Interpolate(const std::vector<double>& v1,
                                const std::vector<double>& v2,
                                const double ratio);

Eigen::Quaterniond Interpolate(const Eigen::Quaterniond& q1,
                               const Eigen::Quaterniond& q2,
                               const double ratio);

Eigen::VectorXd InterpolateXd(const Eigen::VectorXd& v1,
                              const Eigen::VectorXd& v2,
                              const double ratio);

Eigen::Vector3d Interpolate3d(const Eigen::Vector3d& v1,
                              const Eigen::Vector3d& v2,
                              const double ratio);

Eigen::Vector4d Interpolate4d(const Eigen::Vector4d& v1,
                              const Eigen::Vector4d& v2,
                              const double ratio);

Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& t1,
                              const Eigen::Isometry3d& t2,
                              const double ratio);

double SquaredDistance(const Eigen::Vector2d& v1,
                       const Eigen::Vector2d& v2);

double Distance(const Eigen::Vector2d& v1,
                const Eigen::Vector2d& v2);

double SquaredDistance(const Eigen::Vector3d& v1,
                       const Eigen::Vector3d& v2);

double Distance(const Eigen::Vector3d& v1,
                const Eigen::Vector3d& v2);

double SquaredDistance(const Eigen::VectorXd& v1,
                       const Eigen::VectorXd& v2);

double Distance(const Eigen::VectorXd& v1,
                const Eigen::VectorXd& v2);

double Distance(const Eigen::Quaterniond& q1,
                const Eigen::Quaterniond& q2);

double Distance(const Eigen::Isometry3d& t1,
                const Eigen::Isometry3d& t2,
                const double alpha=0.5);

double SquaredDistance(const std::vector<double>& p1,
                       const std::vector<double>& p2);

double Distance(const std::vector<double>& p1, const std::vector<double>& p2);

double ContinuousRevoluteSignedDistance(const double p1, const double p2);

double ContinuousRevoluteDistance(const double p1, const double p2);

double AddContinuousRevoluteValues(const double start, const double change);

double GetContinuousRevoluteRange(const double start, const double end);

bool CheckInContinuousRevoluteRange(const double start,
                                    const double range,
                                    const double val);

bool CheckInContinuousRevoluteBounds(const double start,
                                     const double end,
                                     const double val);

double AverageStdVectorDouble(
    const std::vector<double>& values,
    const std::vector<double>& weights=std::vector<double>());

double ComputeStdDevStdVectorDouble(const std::vector<double>& values,
                                    const double mean);

double ComputeStdDevStdVectorDouble(const std::vector<double>& values);

double WeightedDotProduct(const Eigen::VectorXd& vec1,
                          const Eigen::VectorXd& vec2,
                          const Eigen::VectorXd& weights);

double WeightedSquaredNorm(const Eigen::VectorXd& vec,
                           const Eigen::VectorXd weights);

double WeightedNorm(const Eigen::VectorXd& vec,
                    const Eigen::VectorXd& weights);

double WeightedCosineAngleBetweenVectors(const Eigen::VectorXd& vec1,
                                         const Eigen::VectorXd& vec2,
                                         const Eigen::VectorXd& weights);

double WeightedAngleBetweenVectors(const Eigen::VectorXd& vec1,
                                   const Eigen::VectorXd& vec2,
                                   const Eigen::VectorXd& weights);

Eigen::Vector3d AverageEigenVector3d(
    const VectorVector3d& vectors,
    const std::vector<double>& weights=std::vector<double>());

Eigen::VectorXd AverageEigenVectorXd(
    const std::vector<Eigen::VectorXd>& vectors,
    const std::vector<double>& weights=std::vector<double>());

Eigen::Quaterniond AverageEigenQuaterniond(
    const VectorQuaterniond& quaternions,
    const std::vector<double>& weights=std::vector<double>());

Eigen::Isometry3d AverageEigenIsometry3d(
    const VectorIsometry3d& transforms,
    const std::vector<double>& weights=std::vector<double>());

// This function does not actually deal with the continuous revolute
// space correctly, it just assumes a normal real Euclidean space
double AverageContinuousRevolute(
    const std::vector<double>& angles,
    const std::vector<double>& weights=std::vector<double>());

template <typename Derived>
inline Eigen::MatrixXd ClampNorm(
    const Eigen::MatrixBase<Derived>& item_to_clamp, const double max_norm)
{
  const double current_norm = item_to_clamp.norm();
  if (current_norm > max_norm)
  {
    return item_to_clamp * (max_norm / current_norm);
  }
  return item_to_clamp;
}

// This function is really only going to work well for "almost continuous"
// types, i.e. floats and doubles, due to the implementation
template<typename ScalarType, int Rows,
         typename Allocator=std::allocator<Eigen::Matrix<ScalarType, Rows, 1>>>
inline Eigen::Matrix<ScalarType, Rows, 1> AverageEigenVector(
    const std::vector<Eigen::Matrix<ScalarType, Rows, 1>, Allocator>& vectors,
    const std::vector<double>& weights = std::vector<double>())
{
  // Get the weights
  assert(vectors.size() > 0);
  assert((weights.size() == vectors.size()) || (weights.size() == 0));
  const bool use_weights = (weights.size() != 0);
  // Find the first element with non-zero weight
  size_t starting_idx = 0;
  while (starting_idx < weights.size() && weights[starting_idx] == 0.0)
  {
    starting_idx++;
  }
  // If all weights are zero, result is undefined
  assert(starting_idx < vectors.size());
  // Start the recursive definition with the base case
  Eigen::Matrix<ScalarType, Rows, 1> avg_vector = vectors[starting_idx];
  const double starting_weight = use_weights ? std::abs(weights[starting_idx])
                                             : 1.0;
  assert(starting_weight > 0.0);
  double weights_running_sum = starting_weight;
  // Do the weighted averaging on the rest of the vectors
  for (size_t idx = starting_idx + 1; idx < vectors.size(); ++idx)
  {
    const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
    weights_running_sum += weight;
    const double effective_weight = weight / weights_running_sum;
    const Eigen::Matrix<ScalarType, Rows, 1> prev_avg_vector = avg_vector;
    const Eigen::Matrix<ScalarType, Rows, 1>& current = vectors[idx];
    avg_vector = prev_avg_vector
                 + (effective_weight * (current - prev_avg_vector));
  }
  return avg_vector;
}

// Projects vector_to_project onto base_vector and
// returns the portion that is parallel to base_vector
template <typename DerivedB, typename DerivedV>
inline Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1>
VectorProjection(
    const Eigen::MatrixBase<DerivedB>& base_vector,
    const Eigen::MatrixBase<DerivedV>& vector_to_project)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedB);
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedV);
  EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedB, DerivedV)
  static_assert(std::is_same<typename DerivedB::Scalar,
                typename DerivedV::Scalar>::value,
                "vectors must have the same data type");
  // Perform projection
  const typename DerivedB::Scalar b_squared_norm = base_vector.squaredNorm();
  if (b_squared_norm > 0)
  {
    return (base_vector.dot(vector_to_project) / b_squared_norm) * base_vector;
  }
  else
  {
    return Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1>
        ::Zero(base_vector.rows());
  }
}

// Projects vector_to_project onto base_vector and
// returns the portion that is perpendicular to base_vector
template <typename DerivedB, typename DerivedV>
inline Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1>
VectorRejection(
    const Eigen::MatrixBase<DerivedB>& base_vector,
    const Eigen::MatrixBase<DerivedV>& vector_to_reject)
{
  // Rejection is defined in relation to projection
  return vector_to_reject - VectorProjection(base_vector, vector_to_reject);
}

template <typename DerivedV>
inline Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>
GetArbitraryOrthogonalVector(const Eigen::MatrixBase<DerivedV>& vector)
{
  // We're going to try arbitrary possibilities until one of them works
  const ssize_t vector_size = vector.size();
  if (vector_size > 0)
  {
    for (ssize_t idx = 0; idx < vector_size; idx++)
    {
      Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> test_vector
          = Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>
          ::Zero(vector_size);
      test_vector(idx) = (typename DerivedV::Scalar)1.0;
      const auto rejected_vector = VectorRejection(vector, test_vector);
      const typename DerivedV::Scalar rejected_vector_squared_norm
          = rejected_vector.squaredNorm();
      if (rejected_vector_squared_norm > 0)
      {
        return rejected_vector;
      }
    }
    throw std::runtime_error("Vector rejection failed to find orthogonal"
                             " vector, probably numerical error");
  }
  else
  {
    throw std::invalid_argument("Vector size is zero");
  }
}

template <typename DerivedV>
inline Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1>
GetArbitraryOrthogonalVectorToPlane(
    const Eigen::MatrixBase<DerivedV>& plane_vector1,
    const Eigen::MatrixBase<DerivedV>& plane_vector2,
    const Eigen::MatrixBase<DerivedV>& vector)
{
  const ssize_t vector_size = vector.size();
  if ((vector_size > 0)
      && (vector_size == plane_vector1.size())
      && (vector_size == plane_vector2.size()))
  {
    const Eigen::MatrixBase<DerivedV> unit_plane_vector1
        = plane_vector1 / plane_vector1.norm();
    const Eigen::MatrixBase<DerivedV> unit_plane_vector2
        = plane_vector2 / plane_vector2.norm();
    const typename DerivedV::Scalar plane_vector_dot_product_mag
        = std::abs(unit_plane_vector1.dot(unit_plane_vector2));
    if (plane_vector_dot_product_mag == 1.0)
    {
      throw std::invalid_argument("Plane vectors do not define a valid plane");
    }
    else
    {
      // Try both plane vectors
      // (by definition, one of the two MUST have an orthogonal component!)
      const auto rejected_vector1 = VectorRejection(vector, plane_vector1);
      const typename DerivedV::Scalar rejected_vector1_squared_norm
          = rejected_vector1.squaredNorm();
      if (rejected_vector1_squared_norm > 0)
      {
        return rejected_vector1;
      }
      else
      {
        const auto rejected_vector2 = VectorRejection(vector, plane_vector2);
        const typename DerivedV::Scalar rejected_vector2_squared_norm
            = rejected_vector2.squaredNorm();
        if (rejected_vector2_squared_norm > 0)
        {
          return rejected_vector2;
        }
        else
        {
          throw std::runtime_error("Vector rejection failed to find orthogonal"
                                   " vector, probably numerical error");
        }
      }
    }
  }
  else
  {
    throw std::invalid_argument("Vector size is zero");
  }
}
}
}