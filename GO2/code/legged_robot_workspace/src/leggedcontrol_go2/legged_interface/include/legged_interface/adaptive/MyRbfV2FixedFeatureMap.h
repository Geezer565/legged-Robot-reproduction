#pragma once

#include "legged_interface/adaptive/MyRbfV2FeatureInput.h"
#include <cmath>
#include <stdexcept>

namespace legged { namespace adaptive { namespace v2 {

// Eight input coordinates per axis: residual, physical chi, base/reference
// velocity, and four contacts. tanh bounds the continuous coordinates. Every
// Gaussian participates in adaptation; contacts are coordinates, never gates.
class FixedFeatureMap final {
 public:
  using Point = std::array<double, 8>;
  using Centers = std::array<Point, 25>;
  FixedFeatureMap(Axes residualScale, Axes chiScale, Axes baseVelocityScale,
                  Axes referenceVelocityScale, Axes outputScale, double width)
      : residualScale_(residualScale), chiScale_(chiScale),
        baseVelocityScale_(baseVelocityScale), referenceVelocityScale_(referenceVelocityScale),
        outputScale_(outputScale), width_(width), centers_(makeCenters()) {
    for (unsigned i=0; i<6; ++i) {
      positive(residualScale_[i]); positive(chiScale_[i]);
      positive(baseVelocityScale_[i]); positive(referenceVelocityScale_[i]);
      positive(outputScale_[i]);
    }
    positive(width_);
  }

  Weights physicalGradient(const FeatureInput& input) const {
    Weights result{};
    for (unsigned i=0; i<6; ++i) {
      Point z{{embed(input.causalResidual[i], residualScale_[i]),
               embed(input.chi[i], chiScale_[i]),
               embed(input.baseVelocity[i], baseVelocityScale_[i]),
               embed(input.referenceVelocity[i], referenceVelocityScale_[i]),
               0,0,0,0}};
      for (unsigned k=0; k<4; ++k) z[4+k]=input.contacts[k] ? 1.0 : 0.0;
      result[i][0]=outputScale_[i];
      for (unsigned j=0; j<25; ++j) {
        double squaredDistance=0;
        for (unsigned k=0; k<8; ++k) {
          const double delta=(z[k]-centers_[j][k])/width_;
          squaredDistance+=delta*delta;
        }
        result[i][j+1]=outputScale_[i]*std::exp(-.5*squaredDistance);
      }
    }
    return result;
  }
  const Centers& centers() const { return centers_; }
  double width() const { return width_; }

 private:
  static void positive(double x) {
    if (!std::isfinite(x) || x<=0)
      throw std::invalid_argument("MyRBF V2 fixed feature scales/width must be positive and finite");
  }
  static double embed(double x, double scale) {
    if (!std::isfinite(x)) throw std::invalid_argument("MyRBF V2 feature inputs must be finite");
    return std::tanh(x/scale);
  }
  static Centers makeCenters() {
    Centers result{};
    // Fixed permutations spread centers without a high-dimensional grid.
    // The first 16 nodes cover all contact masks. No online tuning is exposed.
    constexpr unsigned stride[4]={1,7,11,17};
    for (unsigned j=0; j<25; ++j) {
      for (unsigned k=0; k<4; ++k) result[j][k]=-1.0+2.0*((j*stride[k]+3*k)%25)/24.0;
      for (unsigned k=0; k<4; ++k) result[j][4+k]=((j%16)>>k)&1u;
    }
    return result;
  }
  const Axes residualScale_, chiScale_, baseVelocityScale_, referenceVelocityScale_, outputScale_;
  const double width_;
  const Centers centers_;
};

}}}  // namespace legged::adaptive::v2

namespace legged { namespace adaptive {
using MyRbfV2FixedFeatureMap = v2::FixedFeatureMap;
}}
