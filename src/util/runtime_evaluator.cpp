/**
 * @file runtime_evaluator.cpp
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include "util/runtime_evaluator.h"

#include <sstream> // for output
#include <iomanip> // for formatting

using namespace std::chrono;

namespace fastsense::util
{

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
    auto duration = duration_cast<measurement_unit>(stop - start_);

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
#ifdef TIME_MEASUREMENT
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
#endif
}

void RuntimeEvaluator::stop(const std::string& task_name)
{
#ifdef TIME_MEASUREMENT
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
#endif
}

const std::vector<EvaluationFormular> RuntimeEvaluator::get_forms()
{
    return forms_;
}

std::string RuntimeEvaluator::to_string()
{
    pause();

    size_t task_length = std::string("task").length();
    for (const auto& ef : forms_)
    {
        task_length = std::max(task_length, ef.name.length());
    }

    std::stringstream ss;
    ss << "\n " << std::setw(task_length) << "task" << " | "
       << std::setw(6) << "count" << " | "
       << std::setw(8) << "last(ms)" << " | "
       //    << std::setw(10) << "sum(ms)" << " | "
       << std::setw(8) << "min(ms)" << " | "
       << std::setw(8) << "max(ms)" << " | "
       << std::setw(8) << "avg(ms)" << "\n-" << std::setfill('-')
       << std::setw(task_length) << "" << "-+-" // task
       << std::setw(6) << "" << "-+-" // count
       << std::setw(8) << "" << "-+-" // last
       //    << std::setw(10) << "" << "-+-" // sum
       << std::setw(8) << "" << "-+-" // min
       << std::setw(8) << "" << "-+-" // max
       << std::setw(8) << "" << "-\n" << std::setfill(' '); // avg
    for (const auto& ef : forms_)
    {
        unsigned long long avg = ef.sum / ef.count;
        // the name is displayed red if the measurement of the task is still active
        ss << " " << (ef.active ? "\033[31m" /* red */ : "") << std::setw(task_length) << ef.name << "\033[0m" /* reset */ << " | "
           << std::setw(6) << ef.count << " | "
           << std::setw(8) << ef.last / 1000 << " | "
           //    << std::setw(10) << ef.sum / 1000 << " | "
           << std::setw(8) << (ef.min == std::numeric_limits<unsigned long long>::max() ? "infinity" : std::to_string(ef.min / 1000)) << " | "
           << std::setw(8) << ef.max / 1000 << " | "
           << std::setw(8) << avg / 1000 << "\n";
    }

    resume();
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, RuntimeEvaluator& evaluator)
{
#ifdef TIME_MEASUREMENT
    os << evaluator.to_string();
#endif
    return os;
}

} // namespace fastsense::util
