#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <regex>
#include <stdexcept>
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureRbfWrenchInput.h"
#include "ocs2_legged_robot_adaptive/adaptive/TwoFeatureSixSubnetworkRbfWrenchNetwork.h"
#include "ocs2_legged_robot_adaptive/package_path.h"
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive { namespace {
vector6_t scale(){vector6_t s;s<<18.40147689318011,18.757520233566048,17.534482646627026,2.3370077074651903,5.466041588930613,1.7261893548458138;return s;}
TwoFeatureSixSubnetworkRbfWrenchSettings frozen(){
  TwoFeatureSixSubnetworkRbfWrenchSettings s;s.residualScale=s.discrepancyScale=s.outputScale=scale();s.centers.resize(6,50);s.widths.resize(6,25);
  const std::string basisFile = getPath() + "/config/rbf_wrench_two_feature_six_subnetwork_v1.info";
  std::ifstream f(basisFile);
  if (!f) throw std::runtime_error("Cannot open frozen two-feature RBF basis: " + basisFile);
  const std::regex entry(R"(\(([0-5]),([0-9]|1[0-9]|2[0-4]),([01])\)\s+([-+0-9.eE]+))");
  std::string line;
  std::size_t centerCount = 0;
  while (std::getline(f, line) && centerCount < 300) {
    for (std::sregex_iterator it(line.begin(), line.end(), entry), end; it != end; ++it) {
      const int axis = std::stoi((*it)[1].str());
      const int node = std::stoi((*it)[2].str());
      const int feature = std::stoi((*it)[3].str());
      s.centers(axis, 2 * node + feature) = std::stod((*it)[4].str());
      ++centerCount;
    }
  }
  if (centerCount != 300) throw std::runtime_error("Frozen two-feature RBF basis must contain exactly 300 centers.");
  const scalar_t w[6][25]={{.074189672217315053,.34477773789817767,.23396607697305802,.35094911545958707,.34477773789817734,.22985182526545164,.35094911545958696,.23396607697305802,.22333658337991755,.23300049944217155,.34477773789817739,.22985182526545164,.068063114426564403,.22985182526545164,.23396607697305802,.23396607697305802,.16501817993919096,.35094911545958712,.23396607697305802,.22985182526545161,.22985182526545161,.17600554752993672,.068827670887676523,.22985182526545164,.23396607697305802},{.0059882727553250518,.21819359794414664,.35194560359786781,.23463040239857813,.2346304023985783,.024637187006785535,.35194560359786714,.2346304023985783,.2346304023985783,.2346304023985783,.3519456035978672,.14752269723171615,.028963188227015064,.0068473943175004935,.23463040239857813,.028853433727663576,.023607518188913764,.012998405362189151,.0084538247362098695,.012062082517346969,.014038262985548328,.035885908613876437,.013836228199014318,.011510171302395299,.0073969402650420476},{.021815095978480033,.23431080510652835,.35146620765979264,.23431080510652844,.23517440638141571,.016487991405416243,.10369085566359904,.35146620765979253,.23431080510652835,.23431080510652844,.21685871673343915,.012190916612512391,.23431080510652846,.35232980893467986,.075087737254954545,.13557230872261994,.078845306341389781,.032534412728722768,.043620829346703943,.015603511601125067,.039114275374391903,.028566266866250974,.025696434021018094,.033035708821572099,.015849437219283346},{.21711396830943275,.1874189309976167,.35988752644746475,.21242196441492506,.11059622341663512,.18313736473851594,.49571474821631634,.22390821912759781,.51491885047882868,.34328003495266024,.37294917057498705,.34382427405586946,.19309211520632952,.32011832354765096,.3847814848887629,.26503896775397828,.12897788164455684,.17994297525187303,.31001699373572167,.24186421408181435,.22073144853754503,.1428011115611206,.13235179199797936,.40963268874038916,.251860903752331},{.17341682625462421,.17912990256780464,.26224294358550931,.19070066908519454,.45176541773123846,.22763486587222648,.18587630473154482,.27977778482042986,.18845644607326278,.3117275236526833,.39316837086745599,.24712686107349241,.23533833158668366,.29935704882657843,.37245234356813622,.2106530367121989,.18209102140171265,.21666722471490574,.22158793713099234,.22087488454979604,.15931462853009812,.24859997372882009,.36346284849061911,.29494404199065233,.17294296878619164},{.062200570235565043,.25471221277312606,.053955044269568285,.25284233867550399,.049087387617952991,.057853774380720924,.39883458671537581,.13532141571360978,.36649409303006553,.13103688489961979,.087403366302495886,.26549744029734706,.24463426793862841,.24474830373477394,.055003202696072874,.096303835538407392,.24801953104785845,.046968077149936498,.10220430318750864,.083501102519457443,.21734158504765466,.37672749703958125,.067071817271222536,.042756404607251129,.12463990293927328}};for(int i=0;i<6;++i)for(int j=0;j<25;++j)s.widths(i,j)=w[i][j];return s;}
TwoFeatureSixSubnetworkRbfWrenchNetwork make(){return TwoFeatureSixSubnetworkRbfWrenchNetwork(frozen());}
TEST(TwoFeatureNetwork,ProductionLoaderReadsFrozenBasis){const auto s=loadTwoFeatureSixSubnetworkRbfWrenchSettings(getPath()+"/config/rbf_wrench_two_feature_six_subnetwork_v1.info");EXPECT_NEAR(s.residualScale(0),18.40147689318011,1e-14);EXPECT_NEAR(s.centers(0,6),1.0027117584559631,1e-14);EXPECT_NEAR(s.widths(5,24),.12463990293927328,1e-14);}
TEST(TwoFeatureNetwork,SymmetricBasisCoversPositiveAndNegativeCycles){
  const auto settings=loadTwoFeatureSixSubnetworkRbfWrenchSettings(getPath()+"/config/rbf_wrench_two_feature_six_subnetwork_v2.info");
  const TwoFeatureSixSubnetworkRbfWrenchNetwork network(settings);
  for(Eigen::Index axis=0;axis<6;++axis){
    const scalar_t extent=axis<3?2.0:4.0;
    for(const scalar_t sign:{-1.0,1.0}){
      matrix_t feature=matrix_t::Zero(2,6);
      feature(0,axis)=sign*extent;
      feature(1,axis)=sign*extent;
      EXPECT_GT(network.computeBasis(feature).col(axis).tail(25).maxCoeff(),0.95)<<"axis="<<axis<<" sign="<<sign;
    }
  }
}
TEST(TwoFeatureNetwork,ZeroWeights){matrix_t z=matrix_t::Zero(2,6);EXPECT_TRUE(make().evaluate(z).physicalOutput.isZero());}
TEST(TwoFeatureNetwork,CenterActivation){auto n=make();matrix_t z=matrix_t::Zero(2,6);z(0,0)=n.centers()(0,6);z(1,0)=n.centers()(0,7);EXPECT_DOUBLE_EQ(n.computeBasis(z)(4,0),1.);}
TEST(TwoFeatureNetwork,BiasAndScale){auto n=make();matrix_t w=matrix_t::Zero(26,6);w(0,3)=1.;n.setWeights(w);auto e=n.evaluate(matrix_t::Zero(2,6));EXPECT_DOUBLE_EQ(e.normalizedOutput(3),1.);EXPECT_DOUBLE_EQ(e.physicalOutput(3),scale()(3));}
TEST(TwoFeatureNetwork,AxisIndependence){auto n=make();matrix_t a=matrix_t::Zero(2,6),b=a;b(0,0)=.2;b(1,0)=.3;auto x=n.computeBasis(a),y=n.computeBasis(b);EXPECT_FALSE(x.col(0).isApprox(y.col(0)));for(int i=1;i<6;++i)EXPECT_TRUE(x.col(i).isApprox(y.col(i)));}
TEST(TwoFeatureNetwork,WeightColumnIndependence){auto n=make();matrix_t w=matrix_t::Zero(26,6);w(0,4)=.5;n.setWeights(w);auto y=n.evaluateNormalized(matrix_t::Zero(2,6));EXPECT_DOUBLE_EQ(y(4),.5);EXPECT_TRUE(y.head<4>().isZero());EXPECT_DOUBLE_EQ(y(5),0.);}
TEST(TwoFeatureNetwork,WeightCountAndNodeSpecificWidths){auto n=make();EXPECT_EQ(n.weights().size(),156);EXPECT_NE(n.widths()(0,0),n.widths()(0,1));}
TEST(TwoFeatureNetwork,RejectsInvalidWidth){auto s=frozen();s.widths(0,0)=0.;EXPECT_THROW(static_cast<void>(TwoFeatureSixSubnetworkRbfWrenchNetwork(s)),std::invalid_argument);}
TEST(TwoFeatureNetwork,RejectsInvalidCenters){auto s=frozen();s.centers(0,0)=std::numeric_limits<scalar_t>::quiet_NaN();EXPECT_THROW(static_cast<void>(TwoFeatureSixSubnetworkRbfWrenchNetwork(s)),std::invalid_argument);}
TEST(TwoFeatureNetwork,RejectsInvalidFeature){matrix_t z=matrix_t::Zero(2,6);z(0,0)=std::numeric_limits<scalar_t>::infinity();EXPECT_THROW(make().computeBasis(z),std::invalid_argument);}
TEST(TwoFeatureNetwork,PythonConsistency){auto n=make();for(auto pair:{std::pair<scalar_t,scalar_t>{0.,0.},std::pair<scalar_t,scalar_t>{20./scale()(0),5./scale()(0)}}){matrix_t z=matrix_t::Zero(2,6);for(int i=0;i<6;++i){z(0,i)=pair.first;z(1,i)=pair.second;}auto p=n.computeBasis(z);const scalar_t expect0=pair.first==0.? .99803683470653615:.93357387799386304;EXPECT_NEAR(p.col(0).tail(25).maxCoeff(),expect0,1e-13);EXPECT_EQ((p.col(0).tail(25).array()>.1).count(),5);}}
TEST(TwoFeatureNetwork,InputIntegration){vector6_t r=vector6_t::Zero(),p=vector6_t::Zero();r(0)=20.;p(0)=15.;auto z=TwoFeatureRbfWrenchInput(scale(),scale()).compute(r,p).feature;auto n=make();matrix_t w=matrix_t::Zero(26,6);w(1,0)=1.;n.setWeights(w);auto y=n.evaluate(z).physicalOutput;EXPECT_GT(y(0),0.);EXPECT_TRUE(y.tail<5>().isZero());}
}}}}
