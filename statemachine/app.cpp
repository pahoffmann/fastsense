
/**
  * @file Light.cpp
  * @author julian 
  * @date 1/15/21
 */

#include "app.h"
#include "states.h"

class Idle;

// TODO: set the initial state here
App::App()
    : currentState{&Idle::getInstance()}
{
}

void App::setState(BaseState& newState)
{
    currentState->exit(this);  // do stuff before we change state
    currentState = &newState;  // change state
    currentState->enter(this); // do stuff after we change state
}

void App::next()
{
    // Delegate the task of determining the next state to the current state!
    currentState->next(this);
}
