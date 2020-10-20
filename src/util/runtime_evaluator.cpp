/**
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include "util/runtime_evaluator.h"

#include <sstream>

namespace fastsense::util
{

using namespace std::chrono;

RuntimeEvaluator& RuntimeEvaluator::get_instance()
{
    if (instance_ == nullptr)
    {
        instance_ = std::unique_ptr<RuntimeEvaluator>(new RuntimeEvaluator());
    }
    return *instance_;
}

RuntimeEvaluator::RuntimeEvaluator() : started_(false) {}

void RuntimeEvaluator::start(const std::string& task_name)
{
    if (started_)
    {
        throw -1;
    }
    curr_task_name_ = task_name;
    started_ = true;
    start_ = high_resolution_clock::now();
}

void RuntimeEvaluator::stop()
{
    auto stop = high_resolution_clock::now();
    if (!started_)
    {
        throw -1;
    }
    auto duration = duration_cast<milliseconds>(stop - start_);
    auto time = duration.count();
    auto entry = measure_table_.find(curr_task_name_);
    if (entry == measure_table_.end())
    {
        measure_table_[curr_task_name_] = EvaluationFormular(time);
    }
    else
    {
        auto& formular = entry->second;
        formular.curr_time = time;
        formular.time_sum += time;
        ++formular.measure_count;
    }
    started_ = false;
}

std::string RuntimeEvaluator::to_string() const
{
    std::stringstream ss;
    ss << "Time measurement (ms) (<name>: <current> | <average>)\n"; 
    auto total_curr_time = 0llu;
    auto total_avg_time = 0llu;
    for (const auto& entry : measure_table_)
    {
        const auto& formular = entry.second;
        auto avg_time = formular.time_sum / formular.measure_count;

        ss << '\t' << entry.first << ": " << formular.curr_time << " | " << avg_time << "\n"; 

        total_curr_time += formular.curr_time;
        total_avg_time += avg_time;
    }
    ss << '\t' << "total: " << total_curr_time << " | " << total_avg_time; 
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const RuntimeEvaluator& evaluator)
{
    os << evaluator.to_string();
    return os;
}

} // namespace fastsense::util
