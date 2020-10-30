/**
 * @file runtime_evaluator.cpp
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include "util/runtime_evaluator.h"

#include <sstream> // for output
#include <iomanip> // for formatting

namespace fastsense::util
{

using namespace std::chrono;

RuntimeEvaluator& RuntimeEvaluator::get_instance()
{
    static RuntimeEvaluator instance;
    return instance;
}

RuntimeEvaluator::RuntimeEvaluator() : forms_()
{
    // "unpause" for the first time
    start_ = high_resolution_clock::now();
}

void RuntimeEvaluator::pause()
{
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start_);

    // add new interval to all active measurements
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].active)
        {
            forms_[i].accumulate += duration.count();
        }
    }
}

void RuntimeEvaluator::resume()
{
    start_ = high_resolution_clock::now();
}

int RuntimeEvaluator::find_formular(const std::string& task_name)
{
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            return i;
        }
    }

    return -1;
}

void RuntimeEvaluator::start(const std::string& task_name)
{
    pause();

    // get or create task that is started
    int index = find_formular(task_name);

    if (index == -1)
    {
        index = forms_.size();
        forms_.push_back(EvaluationFormular(task_name));
    }
    else if (forms_[index].active)
    {
        throw RuntimeEvaluationException();
    }

    // start
    forms_[index].active = true;
    forms_[index].accumulate = 0;

    resume();
}

void RuntimeEvaluator::stop(const std::string& task_name)
{
    pause();

    // get task that is stopped
    auto index = find_formular(task_name);

    if (index == -1 || !forms_[index].active)
    {
        throw RuntimeEvaluationException();
    }

    // stop
    unsigned long long time = forms_[index].accumulate;
    forms_[index].active = false;
    forms_[index].count++;
    forms_[index].last = time;
    forms_[index].sum += time;
    if (time < forms_[index].min)
    {
        forms_[index].min = time;
    }
    if (time > forms_[index].max)
    {
        forms_[index].max = time;
    }

    resume();
}

const std::vector<EvaluationFormular> RuntimeEvaluator::get_forms()
{
    return forms_;
}

std::string RuntimeEvaluator::to_string()
{
    pause();

    std::stringstream ss;
    ss << " " << std::setw(16) << "task" << " | "
       << std::setw(10) << "count" << " | "
       << std::setw(10) << "last [µs]" << " | "
       << std::setw(10) << "sum [µs]" << " | "
       << std::setw(10) << "min [µs]" << " | "
       << std::setw(10) << "max [µs]" << " | "
       << std::setw(10) << "avg [µs]" << "\n-" << std::setfill('-')
       << std::setw(16) << "" << "-+-"
       << std::setw(10) << "" << "-+-"
       << std::setw(10) << "" << "-+-"
       << std::setw(10) << "" << "-+-"
       << std::setw(10) << "" << "-+-"
       << std::setw(10) << "" << "-+-"
       << std::setw(10) << "" << "-\n" << std::setfill(' ');
    for (const auto& ef : forms_)
    {
        unsigned long long avg = ef.sum / ef.count;
        // the name is displayed red if the measurement of the task is still active
        ss << " " << (ef.active ? "\033[31m" /* red */ : "") << std::setw(16) << ef.name << "\033[0m" /* reset */ << " | "
           << std::setw(10) << ef.count << " | "
           << std::setw(10) << ef.last << " | "
           << std::setw(10) << ef.sum << " | "
           << std::setw(10) << (ef.min == std::numeric_limits<unsigned long long>::max() ? "infinity" : std::to_string(ef.min)) << " | "
           << std::setw(10) << ef.max << " | "
           << std::setw(10) << avg << "\n";
    }

    resume();
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, RuntimeEvaluator& evaluator)
{
    os << evaluator.to_string();
    return os;
}

} // namespace fastsense::util
