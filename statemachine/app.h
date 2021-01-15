#pragma once

/**
  * @file Light.h
  * @author julian 
  * @date 1/15/21
 */

#pragma once
#include "base_state.h"

class BaseState;

class App
{
public:
    App();
    inline BaseState* getCurrentState() const { return currentState; }
    void next();
    void setState(BaseState& newState);

private:
    BaseState* currentState;
};
