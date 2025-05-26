#include <array>

class ZMPController {
public:
  // CoM adjustment, input desired joint angles
  using JointAngles = std::array<std::array<double,2>,4>;

  JointAngles adjust(const JointAngles &angles) {
    return angles;
    // Einfache ZMP-Korrektur: Halte Verteilung symmetrisch
    // Berechne mittlere Hüft- und Kniewinkel
    /*
    double sum_hip = 0.0, sum_knee = 0.0;
    for (const auto &leg : angles) {
      sum_hip += leg[0];
      sum_knee += leg[1];
    }
    double avg_hip = sum_hip / 4.0;
    double avg_knee = sum_knee / 4.0;

    JointAngles adjusted = angles;
    // Proportionale Korrektur: bringe jedes Bein näher zum Durchschnitt
    double kp = 0.1;
    for (size_t i = 0; i < 4; ++i) {
      adjusted[i][0] += kp * (avg_hip - angles[i][0]);
      adjusted[i][1] += kp * (avg_knee - angles[i][1]);
    }
    return adjusted;
    */
  }
};