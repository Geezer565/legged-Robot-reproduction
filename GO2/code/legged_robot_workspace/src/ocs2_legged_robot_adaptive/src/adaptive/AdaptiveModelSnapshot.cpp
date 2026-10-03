#include "ocs2_legged_robot_adaptive/adaptive/AdaptiveModelSnapshot.h"
#include <cmath>
#include <stdexcept>
#include <utility>
namespace ocs2 { namespace legged_robot_adaptive { namespace adaptive {
AdaptiveModelSnapshot::AdaptiveModelSnapshot(parameter_vector_t e, scalar_t t, std::uint64_t v, bool valid) : estimate_(std::move(e)), time_(t), version_(v), valid_(valid) {}
AdaptiveModelSnapshotPtr AdaptiveModelSnapshot::createInitial(const parameter_vector_t& e) { if (e.size() <= 0 || !e.allFinite()) throw std::invalid_argument("[AdaptiveModelSnapshot] estimate 非有限或为空。"); return std::shared_ptr<const AdaptiveModelSnapshot>(new AdaptiveModelSnapshot(e, 0., 0, false)); }
AdaptiveModelSnapshotPtr AdaptiveModelSnapshot::createValid(const parameter_vector_t& e, scalar_t t, std::uint64_t v) { if (e.size() <= 0 || !e.allFinite() || !std::isfinite(t) || v == 0) throw std::invalid_argument("[AdaptiveModelSnapshot] 有效快照参数非法。"); return std::shared_ptr<const AdaptiveModelSnapshot>(new AdaptiveModelSnapshot(e, t, v, true)); }
} } }
