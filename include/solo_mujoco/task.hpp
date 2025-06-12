#ifndef TASK_
#define TASK_

#include <string>
#include <array>
#include <algorithm>

class Task {
public:
    virtual ~Task() = default;

    virtual std::string getName() const = 0;

    /**
     * Starts the task.
     * This method is called before the first `execute` call for the task.
     * The `execute` method is called right after the `start` method call.
     * `timestamp` is the current simulation timestamp.
     */
    virtual void start([[maybe_unused]] const double timestamp) {}

    /**
     * Executes the task at the current simulation `timestamp.
     * Calles in the main control loop.
     */
    virtual void execute(const double timestamp) = 0;

    /**
     * Requests the end of the task by returning to the stable stand (as defined in the `stableStandAngles` attribute).
     * After the call of this method, the method `returnToStableStand` is called periodically (similar to the `execute` calls),
     * but `execute` is not called anymore.
     * The loop for `returnToStableStand` continues until `returnedToStableStand` is true.
     * `desired_timestamp` is the current simulation timestamp.
     * `desired_duration` is the duration the task have to return to the stable stand.
     */
    virtual void startReturnToStableStand(const double current_timestamp, const double desired_duration) = 0;

    /**
     * Returns to the stable stand (as defined in the `stableStandAngles` attribute).
     * This method gets called periodically (similar to the `execute` calls).
     * The loops stops when `returnedToStableStand` is true.
     * `timestamp` is the current simulation timestamp.
     */
    virtual void returnToStableStand(const double timestamp) = 0;

    /**
     * Requests the status of the return to the stable stand.
     * After the return finished, other tasks can be started by the task_controller.
     * `timestamp` is the current simulation timestamp.
     */
    virtual bool returnedToStableStand(const double timestamp) = 0;

protected:
    std::array<std::array<double, 2>, 4> stableStandAngles = {{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.4, -0.7}}
    }};

    std::array<std::array<double, 2>, 4> copyStableStandAngles() {
        std::array<std::array<double, 2>, 4> copy;
        std::copy(stableStandAngles.begin(), stableStandAngles.end(), copy.begin());
        return copy;
    }
};

#endif
