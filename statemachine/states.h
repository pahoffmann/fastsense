#pragma once

/**
  * @file ConcreteLightStates.h
  * @author julian 
  * @date 1/15/21
 */
#pragma once
#include "base_state.h"
#include "app.h"

class Idle : public BaseState
{
public:
    void enter(App* light) final {}
    void next(App* light) final;
    void exit(App* light) final {}
    std::string description()final;
    static BaseState& getInstance();

    Idle(const Idle& other) = delete;
    Idle& operator=(const Idle& other) = delete;
private:
    Idle() = default;
};

class ImuCalib : public BaseState
{
public:
    void enter(App* light) final {}
    void next(App* light) final;
    void exit(App* light) final {}
    std::string description()final;
    static BaseState& getInstance();

    ImuCalib(const ImuCalib& other) = delete;
    ImuCalib& operator=(const ImuCalib& other) = delete;
private:
    ImuCalib() = default;
};

class Ready : public BaseState
{
public:
    void enter(App* light) final {}
    void next(App* light) final;
    void exit(App* light) final {}
    std::string description() final;
    static BaseState& getInstance();

    Ready(const Ready& other) = delete;
    Ready& operator=(const Ready& other) = delete;
private:
    Ready() = default;
};

class SlamOn : public BaseState
{
public:
    void enter(App* light) final {}
    void next(App* light) final;
    void exit(App* light) final {}
    std::string description() final;
    static BaseState& getInstance();

    SlamOn(const SlamOn& other) = delete;
    SlamOn& operator=(const SlamOn& other) = delete;
private:
    SlamOn() = default;
};

class SlamOff : public BaseState
{
public:
    void enter(App* light) final {}
    void next(App* light) final;
    void exit(App* light) final {}
    std::string description() final;
    static BaseState& getInstance();

    SlamOff(const SlamOff& other) = delete;
    SlamOff& operator=(const SlamOff& other) = delete;

private:
    SlamOff() = default;
};
