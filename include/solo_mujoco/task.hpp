#ifndef TASK_
#define TASK_

#include <string>

class Task {
public:
    virtual ~Task() = default;

    virtual std::string getName() const = 0;
    virtual void start([[maybe_unused]] const double timestamp) {}
    virtual void execute(const double timestamp) = 0;
    virtual void finish([[maybe_unused]] const double timestamp) {}
};

#endif
