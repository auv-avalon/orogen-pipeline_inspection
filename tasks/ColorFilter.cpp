/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ColorFilter.hpp"

using namespace pipeline_inspection;

ColorFilter::ColorFilter(std::string const& name, TaskCore::TaskState initial_state)
    : ColorFilterBase(name, initial_state)
{
}

ColorFilter::ColorFilter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ColorFilterBase(name, engine, initial_state)
{
}

ColorFilter::~ColorFilter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ColorFilter.hpp for more detailed
// documentation about them.

bool ColorFilter::configureHook()
{
    if (! ColorFilterBase::configureHook())
        return false;
    return true;
}
bool ColorFilter::startHook()
{
    if (! ColorFilterBase::startHook())
        return false;
    return true;
}
void ColorFilter::updateHook()
{
    ColorFilterBase::updateHook();
}
void ColorFilter::errorHook()
{
    ColorFilterBase::errorHook();
}
void ColorFilter::stopHook()
{
    ColorFilterBase::stopHook();
}
void ColorFilter::cleanupHook()
{
    ColorFilterBase::cleanupHook();
}
