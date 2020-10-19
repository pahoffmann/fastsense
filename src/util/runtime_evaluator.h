#pragma once

/**
 * @author Marc Eisoldt (meisoldt)
 */

#include <iostream>
#include <map>
#include <tuple>
#include <chrono>

#define TIME_MEASUREMENT

namespace fastsense::util
{

/**
 * @brief Helper Struct for managing the different measurement variables for every considered task
 */
struct EvaluationFormular
{
    EvaluationFormular() : curr_time(0), 
                           time_sum(0), 
                           measure_count(0) {}

    EvaluationFormular(unsigned long long first_time) : curr_time(first_time),
                                                        time_sum(first_time),
                                                        measure_count(1) {}

    /// Current measured runtime for the task 
    unsigned long long curr_time;
    /// Current sum of all measured runtimes for the task
    unsigned long long time_sum;
    /// Number of measurements for this task
    unsigned long long measure_count;
}; 

/**
 * @brief Encapsulates the runtime measurement for different tasks. Only one task can be measured at the same sime
 */
class RuntimeEvaluator
{

public:

    /**
     * @brief Starts a new measuremt for a task. Only one measurement can be started at a time.
     *        So it is important to call the stop function before calling this function again
     * 
     * @param task_name Name of the task, which will be shown in the printed overview 
     *                  and is considered as identifier to find the right measurement variables
     * @throws int -1 if a measurement was already started
     */
    void start(const std::string& task_name);
    
    /**
     * @brief Stops the current measurement and updates the variables for the considered task.
     *        This function can only be called once after the start function was called
     * @throws int -1 if no measurement was started yet
     */
    void stop();

    /**
     * @brief Creates a string from the current measurements for every considered task
     * 
     * @return std::string String that represents a measurement overview 
     */
    std::string to_string() const;

private:

    /// Stores the Different measurement variables for every considered task with the name of the task as key 
    std::map<std::string, EvaluationFormular> measure_table_;
    /// Temporary variable for storing the start time of the current measurement
    std::chrono::_V2::system_clock::time_point start_;
    /// Temporary variable for storing the name of the task which is considered in the current measurement
    std::string curr_task_name_;
    /// Was a measurement already started?
    bool started_ = false;
};

/**
 * @brief Puts the current measured variables of every task considerd by a given evaluator into a given stream  
 */
std::ostream& operator<<(std::ostream& os, const RuntimeEvaluator& evaluator);

} // namespace fastsense::util
