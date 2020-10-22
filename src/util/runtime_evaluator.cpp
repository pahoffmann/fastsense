/**
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
    if (instance_ == nullptr)
    {
        instance_.reset(new RuntimeEvaluator());
    }
    return *instance_;
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

void RuntimeEvaluator::start(const std::string& task_name)
{
    pause();

    // get or create task that is started
    int index = -1;
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            if (forms_[i].active)
            {
                throw RuntimeEvaluationException();
            }
            index = i;
        }
    }
    if (index == -1)
    {
        index = forms_.size();
        forms_.push_back(EvaluationFormular(task_name));
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
    int index = -1;
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            if (!forms_[i].active)
            {
                throw RuntimeEvaluationException();
            }
            index = i;
        }
    }
    if (index == -1)
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

std::string RuntimeEvaluator::to_string()
{
    pause();


    // TODO: schön machen

    std::stringstream ss;
    //ss << "Time measurement (ms) (<name>: <current> | <average>)\n";
    ss << std::setw(16) << "taskname" << " | "
       << std::setw(8) << "sum" << " | "
       << std::setw(8) << "count" << " | "
       << std::setw(8) << "avg" << " | "
       << std::setw(8) << "min" << " | "
       << std::setw(8) << "max" << "\n"
       << "-----------------+----------+----------+----------+----------+----------\n";

    //auto total_avg_time = 0llu;
    for (const auto& ef : forms_)
    {
        ss << std::setw(16) << ef.name << " | "
           << std::setw(8) << ef.sum << " | "
           << std::setw(8) << ef.count << " | "
           << std::setw(8) << 21 << " | "
           << std::setw(8) << ef.min << " | "
           << std::setw(8) << ef.max << "\n";

        //ss << std::setw(10) << entry.first << ":" << std::setw(10) << avg_time << "\n";
        //ss << '\t' << entry.first << ": " << avg_time << "\n"; 

        //total_curr_time += formular.curr_time;
        //total_avg_time += avg_time;
    }
    //ss << '\t' << "total: " << total_avg_time;

    resume();
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, RuntimeEvaluator& evaluator)
{
    os << evaluator.to_string();
    return os;
}

} // namespace fastsense::util
