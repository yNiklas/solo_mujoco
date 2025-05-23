// ik_solver.cpp
#include <vector>
#include <array>
#include <stdexcept>
#include <math.h>

// Dummy IK Solver using KDL or custom solver
class IKSolver {
public:
  // Fuß-Position [x, y, z] pro Bein
  using FootTargets = std::array<std::array<double,3>,4>;
  // Gelenkwinkel pro Bein (hip, knee)
  using JointAngles = std::array<std::array<double,2>,4>;

  JointAngles solve(const FootTargets &targets) {
    JointAngles angles;
    // Simplified planar 2-DoF leg IK for each leg
    for(size_t i=0; i<4; ++i) {
      double x = targets[i][0];
      double z = targets[i][2];
      double L1 = 0.16; // Hüft-Abstand
      double L2 = 0.16; // Beinlänge
      double D = (x*x + z*z - L1*L1 - L2*L2)/(2*L1*L2);
      if(std::abs(D)>1) throw std::runtime_error("IK Ziel unerreichbar");
      double q2 = std::atan2(-std::sqrt(1 - D*D), D);
      double q1 = std::atan2(z, x) - std::atan2(L2*std::sin(q2), L1 + L2*std::cos(q2));
      angles[i] = {{q1, q2-M_PI}};
    }
    return angles;
  }
};