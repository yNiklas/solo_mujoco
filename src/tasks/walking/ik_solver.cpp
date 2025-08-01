// ik_solver.cpp
#include "solo_mujoco/ik_solver.hpp"

#include <stdexcept>
#include <math.h>
#include <iostream>

namespace solo_mujoco {
IKSolver::JointAngles IKSolver::solve(const FootTargets &targets) {
  IKSolver::JointAngles angles;
  for(size_t i=0; i<4; ++i) {
    double x = targets[i][0];
    double z = targets[i][2];
    double L = 0.16;
    
    double distanceSquared = x * x + z * z;
    double cosKnee = (distanceSquared - 2 * L * L) / (2 * L * L);
    if (cosKnee < -1.0 || cosKnee > 1.0) {
      std::cout << "(" << x << "," << z << ")" << std::endl;
      throw std::runtime_error("Out of range");
    }
    // Use -acos(cosKnee) to obtain the knee-up (second) solution
    double kneeAngle = -std::acos(cosKnee);
    double k1 = L + L * std::cos(kneeAngle);
    double k2 = L * std::sin(kneeAngle);
    // -z since the ground is under the robot coordinate system
    double hipAngle = std::atan2(-z, x) - std::atan2(k2, k1);
    angles[i] = {hipAngle-M_PI/2, kneeAngle};
  }
  return angles;
}
}