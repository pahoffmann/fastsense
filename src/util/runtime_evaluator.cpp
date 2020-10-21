/**
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include "util/runtime_evaluator.h"

#include <sstream>
#include <iomanip>

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

void RuntimeEvaluator::start(const std::string& task_name)
{
    // pause
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start_);

    // add new interval to all active measurements
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].active)
        {
            forms_[i].curr += duration.count();
        }
    }

    // get or create task that is started
    int index = -1;
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            if (forms_[i].active)
            {
                throw -1;
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
    forms_[index].curr = 0;

    // unpause
    start_ = high_resolution_clock::now();
}

void RuntimeEvaluator::stop(const std::string& task_name)
{
    // pause
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start_);

    // add new interval to all active measurements
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].active)
        {
            forms_[i].curr += duration.count();
        }
    }

    // get task that is started
    int index = -1;
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            if (!forms_[i].active)
            {
                throw -1;
            }
            index = i;
        }
    }
    if (index == -1)
    {
        throw -1;
    }

    // stop
    forms_[index].active = false;
    forms_[index].sum += forms_[index].curr;
    forms_[index].count++;

    // unpause
    start_ = high_resolution_clock::now();
}

std::string RuntimeEvaluator::to_string() const
{
    std::stringstream ss;
    //ss << "Time measurement (ms) (<name>: <current> | <average>)\n";
    ss << std::setw(16) << "taskname" << " | "
       << std::setw(8) << "sum" << " | "
       << std::setw(8) << "count" << " | "
       << std::setw(8) << "avg nix" << " | "
       << std::setw(8) << "min act" << " | "
       << std::setw(8) << "max curr" << "\n"
       << "-----------------+----------+----------+----------+----------+----------\n";

    //auto total_avg_time = 0llu;
    for (const auto& ef : forms_)
    {
        ss << std::setw(16) << ef.name << " | "
           << std::setw(8) << ef.sum << " | "
           << std::setw(8) << ef.count << " | "
           << std::setw(8) << 0 << " | "
           << std::setw(8) << ef.active << " | "
           << std::setw(8) << ef.curr << "\n";

        //ss << std::setw(10) << entry.first << ":" << std::setw(10) << avg_time << "\n";
        //ss << '\t' << entry.first << ": " << avg_time << "\n"; 

        //total_curr_time += formular.curr_time;
        //total_avg_time += avg_time;
    }
    //ss << '\t' << "total: " << total_avg_time; 
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const RuntimeEvaluator& evaluator)
{
    os << evaluator.to_string();
    return os;
}

} // namespace fastsense::util
