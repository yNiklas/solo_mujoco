#ifndef IK_SOLVER_
#define IK_SOLVER_

#include <vector>
#include <array>

namespace solo_mujoco {
// Inverse kinematics solver for the 2D planar case of each leg
class IKSolver {
public:
    // End-effector (foot) position (x,y,z) for each leg
    using FootTargets = std::array<std::array<double,3>,4>;
    // Joint angles (hip, knee) for each leg
    using JointAngles = std::array<std::array<double,2>,4>;

    JointAngles solve(const FootTargets &targets);
};
}

#endif
