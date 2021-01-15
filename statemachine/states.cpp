
/**
  * @file ConcreteLightStates.cpp
  * @author julian 
  * @date 1/15/21
 */

#include "states.h"

void Idle::next(App* light)
{
    light->setState(ImuCalib::getInstance());
}

BaseState& Idle::getInstance()
{
    static Idle singleton;
    return singleton;
}

std::string Idle::description()
{
    return "Idle";
}

void ImuCalib::next(App* light)
{
    light->setState(Ready::getInstance());
}

BaseState& ImuCalib::getInstance()
{
    static ImuCalib singleton;
    return singleton;
}

std::string ImuCalib::description()
{
    return "ImuCalib";
}

void Ready::next(App* light)
{
    light->setState(SlamOn::getInstance());
}

BaseState& Ready::getInstance()
{
    static Ready singleton;
    return singleton;
}

std::string Ready::description()
{
    return "Ready";
}

void SlamOff::next(App* light)
{
    light->setState(SlamOn::getInstance());
}

BaseState& SlamOff::getInstance()
{
    static SlamOff singleton;
    return singleton;
}

std::string SlamOff::description()
{
    return "SLAM off";
}

std::string SlamOn::description()
{
    return "SLAM on";
}

BaseState &SlamOn::getInstance()
{
    static SlamOn singleton;
    return singleton;
}

void SlamOn::next(App *light)
{
    light->setState(SlamOff::getInstance());
}
