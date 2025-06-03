#include <array>
#include <cmath>
#include <functional>
#include <iostream>

class GaitPlanner {
private:
    using FootTargets = std::array<std::array<double,3>,4>;
    std::array<bool, 4> stance = {{true, false, false, true}}; // FL FR HL HR
    int stance_phases_per_gait = 2; // 2 stance phases for trot
    int last_stance_phase = 0;
    std::function<void()> stance_shifter;

    double start_time = 0; // in seconds
    double step_frequency = 0.5; // steps (full gait cycle) per second
    double step_length = 0.1;
    double step_height = 0.07;
    double z_ground = -0.28;

    void init(std::function<void()> stance_shifter, double step_frequency, double step_length, double step_height, double z_ground) {
        this->stance_shifter = stance_shifter;
        this->step_frequency = step_frequency;
        this->step_length = step_length;
        this->step_height = step_height;
        this->z_ground = z_ground;
    }

    double calculatePhaseAndShiftStances(const double timestamp) {
        double delta = timestamp - start_time;
        double in_step = std::fmod(delta, 1/step_frequency);
        int stance = static_cast<int>(std::fmod((in_step/(1/step_frequency))*stance_phases_per_gait, stance_phases_per_gait));
        if (stance != last_stance_phase) {
            stance_shifter();
        }
        double stance_duration = 1/(step_frequency*stance_phases_per_gait);
        return (in_step - stance*stance_duration) / stance_duration;
    }

    void shift_trot_stances() {
        if (stance[0]) {
            stance[0] = false;
            stance[1] = true;
            stance[2] = true;
            stance[3] = false;
        } else {
            stance[0] = true;
            stance[1] = false;
            stance[2] = false;
            stance[3] = true;
        }
        last_stance_phase = (last_stance_phase+1) % stance_phases_per_gait;
    }

public:
    void initializeTrot(double step_frequency, double step_length, double step_height, double z_ground) {
        auto fnc = [this]() {shift_trot_stances();};
        this->init(fnc, step_frequency, step_length, step_height, z_ground);
    }

    FootTargets computeTrotFootTargets(const double timestamp) {
        if (start_time == 0) start_time = timestamp;

        double phase = calculatePhaseAndShiftStances(timestamp); // 0<=phase<=1
        FootTargets targets; // FL FR HL HR
        for (int i=0; i<4; ++i) {
            if (stance[i]) {
                // Stance -> Linear movement
                targets[i] = {{step_length/4 - step_length*phase/2, 0, z_ground}};
            } else {
                // Swing
                double x = -step_length / 2 + step_length * phase;
                double z = z_ground + step_height * std::sin(M_PI * phase);
                targets[i] = {{x,0,z}};
            }
        }
        return targets;
    }
};