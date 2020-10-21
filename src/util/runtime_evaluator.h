#pragma once

/**
 * @author Marc Eisoldt (meisoldt)
 * @author Steffen Hinderink
 */

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

#define TIME_MEASUREMENT

namespace fastsense::util
{

/**
 * @brief Helper Struct for managing the different measurement variables for every considered task
 */
struct EvaluationFormular
{
    EvaluationFormular(const std::string& name) : name(name), active(true), curr(0), sum(0), count(0) {}
    std::string name;
    bool active;
    unsigned long long curr;


    /// Current sum of all measured runtimes for the task
    unsigned long long sum;
    /// Number of measurements for this task
    unsigned int count;
};

/**
 * @brief Encapsulates the runtime measurement for different tasks. Only one task can be measured at the same sime.
 * It is implemented as a singleton.
 */
class RuntimeEvaluator
{

public:

    /**
     * @brief Don't allow copies of the instance to ensure singleton property
     */
    RuntimeEvaluator& operator=(RuntimeEvaluator&) = delete;
    RuntimeEvaluator(RuntimeEvaluator&) = delete;

    /**
     * Returns the singleton instance. Creates a new one the first time.
     * @return Singleton instance
     */
    static RuntimeEvaluator& get_instance();

    /**
     * @brief Starts a new measuremt for a task. Only one measurement can be started at a time.
     *        So it is important to call the stop function before calling this function again
     * @param task_name Name of the task, which will be shown in the printed overview 
     *                  and is considered as identifier to find the right measurement variables
     * @throws int -1 if a measurement was already started
     */
    void start(const std::string& task_name);
    
    /**
     * @brief Stops the current measurement and updates the variables for the considered task.
     *        This function can only be called once after the start function was called
     * @param task_name Name of the task, which will be shown in the printed overview 
     *                  and is considered as identifier to find the right measurement variables
     * @throws int -1 if no measurement was started yet
     */
    void stop(const std::string& task_name);

    /**
     * @brief Creates a string from the current measurements for every considered task
     * 
     * @return std::string String that represents a measurement overview 
     */
    std::string to_string() const;

private:

    /// Singleton instance
    inline static std::unique_ptr<RuntimeEvaluator> instance_ = nullptr;
    /// Private constructor to ensure singleton property
    RuntimeEvaluator();
    /// Stores the Different measurement variables for every considered task with the name of the task as key 
    std::vector<EvaluationFormular> forms_;
    /// Temporary variable for storing the start time of the current measurement
    std::chrono::_V2::system_clock::time_point start_;
    /// Temporary variable for storing the name of the task which is considered in the current measurement
    //std::string curr_task_name_;
    /// Was a measurement already started?
    //bool started_;
};

/**
 * @brief Puts the current measured variables of every task considerd by a given evaluator into a given stream  
 */
std::ostream& operator<<(std::ostream& os, const RuntimeEvaluator& evaluator);

} // namespace fastsense::util
