// Deterministic mathematical asset, evaluated by the actual C++ feature map.
// This is not sampled robot data and does not demonstrate learned performance.
#include <legged_interface/adaptive/MyRbfV2FixedFeatureMap.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: export_v2_fixed_gaussian output.csv\n"; return 2; }
  using namespace legged::adaptive::v2;
  const Axes unit{{1,1,1,1,1,1}};
  const FixedFeatureMap map(unit,unit,unit,unit,unit,1.0);
  const unsigned node = 15, axis = 1, count = 81;
  const auto center = map.centers()[node];
  FeatureInput input;
  input.baseVelocity[axis] = std::atanh(center[2]);
  input.referenceVelocity[axis] = std::atanh(center[3]);
  input.contacts = {{true,true,true,true}};
  std::ofstream out(argv[1]);
  if (!out) throw std::runtime_error("cannot open output CSV");
  out << "z_residual,z_chi,normalized_phi,node_index,width,center_residual,center_chi,fixed_z_base_velocity,fixed_z_reference_velocity,contact_mask\n";
  out << std::setprecision(17);
  for (unsigned iy=0;iy<count;++iy) for(unsigned ix=0;ix<count;++ix) {
    const double x=-.98+1.96*ix/(count-1), y=-.98+1.96*iy/(count-1);
    input.causalResidual[axis]=std::atanh(x);
    input.chi[axis]=std::atanh(y);
    const double phi=map.physicalGradient(input)[axis][node+1];
    const double expected=std::exp(-.5*((x-center[0])*(x-center[0])+(y-center[1])*(y-center[1])));
    if(std::abs(phi-expected)>1e-12)throw std::runtime_error("feature slice disagrees with Gaussian definition");
    out << x << ',' << y << ',' << phi << ',' << node << ',' << map.width() << ','
        << center[0] << ',' << center[1] << ',' << center[2] << ',' << center[3] << ",15\n";
  }
  if(!out)throw std::runtime_error("failed writing deterministic data");
  return 0;
}
