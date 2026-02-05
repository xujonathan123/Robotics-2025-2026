#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::Motor intake;
extern pros::Motor intake_stage2;

// Matchload ramp (pneumatics)
void matchload_set(bool raised);
void matchload_toggle();
bool matchload_is_raised();

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
