#pragma once

/**
  * @file LightState.h
  * @author julian 
  * @date 1/15/21
 */
#include "app.h"
#include <string>

class App;

class BaseState
{
public:
    virtual void enter(App* light) = 0;
    virtual void next(App* light) = 0;
    virtual void exit(App* light) = 0;
    virtual std::string description() = 0;
    virtual ~BaseState() = default;
};
