/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OutlierFilter.hpp"
#include <depth_map_preprocessing/Filters.hpp>

using namespace depth_map_preprocessing;

OutlierFilter::OutlierFilter(std::string const& name)
    : OutlierFilterBase(name)
{
}

OutlierFilter::OutlierFilter(std::string const& name, RTT::ExecutionEngine* engine)
    : OutlierFilterBase(name, engine)
{
}

OutlierFilter::~OutlierFilter()
{
}

bool OutlierFilter::setMaximum_angle_to_neighbor(double value)
{
    maximum_angle_to_neighbor = value;
    return true;
}

bool OutlierFilter::setValid_neighbors(boost::int32_t value)
{
    minimum_valid_neighbors = value;
    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OutlierFilter.hpp for more detailed
// documentation about them.

bool OutlierFilter::configureHook()
{
    if (! OutlierFilterBase::configureHook())
        return false;

    maximum_angle_to_neighbor = _maximum_angle_to_neighbor.value();
    minimum_valid_neighbors = _valid_neighbors.value();

    return true;
}
bool OutlierFilter::startHook()
{
    if (! OutlierFilterBase::startHook())
        return false;
    return true;
}
void OutlierFilter::updateHook()
{
    OutlierFilterBase::updateHook();

    base::samples::DepthMap depth_map;
    while(_depth_map.read(depth_map, false) == RTT::NewData)
    {
        Filters::filterOutliers(depth_map, maximum_angle_to_neighbor, minimum_valid_neighbors);
        _filtered_depth_map.write(depth_map);
    }
}
void OutlierFilter::errorHook()
{
    OutlierFilterBase::errorHook();
}
void OutlierFilter::stopHook()
{
    OutlierFilterBase::stopHook();
}
void OutlierFilter::cleanupHook()
{
    OutlierFilterBase::cleanupHook();
}
