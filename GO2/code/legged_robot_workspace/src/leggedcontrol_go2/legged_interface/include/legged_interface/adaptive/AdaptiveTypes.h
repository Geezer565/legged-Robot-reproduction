#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_core/Types.h>

namespace legged {
namespace adaptive {

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

}  // namespace adaptive
}  // namespace legged
