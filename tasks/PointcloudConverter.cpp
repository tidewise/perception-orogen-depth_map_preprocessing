/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudConverter.hpp"

using namespace depth_map_preprocessing;

PointcloudConverter::PointcloudConverter(std::string const& name)
    : PointcloudConverterBase(name)
{
}

PointcloudConverter::PointcloudConverter(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudConverterBase(name, engine)
{
}

PointcloudConverter::~PointcloudConverter()
{
}

void PointcloudConverter::newSampleCallback(const base::Time& ts, const base::samples::DepthMap& depth_map_sample)
{
    base::samples::Pointcloud pointcloud;
    pointcloud.time = ts;
    if(convertToPointCloud(ts, depth_map_sample, pointcloud.points))
        _pointcloud.write(pointcloud);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PointcloudConverter.hpp for more detailed
// documentation about them.

bool PointcloudConverter::configureHook()
{
    if (! PointcloudConverterBase::configureHook())
        return false;
    return true;
}
bool PointcloudConverter::startHook()
{
    if (! PointcloudConverterBase::startHook())
        return false;
    return true;
}
void PointcloudConverter::updateHook()
{
    PointcloudConverterBase::updateHook();
}
void PointcloudConverter::errorHook()
{
    PointcloudConverterBase::errorHook();
}
void PointcloudConverter::stopHook()
{
    PointcloudConverterBase::stopHook();
}
void PointcloudConverter::cleanupHook()
{
    PointcloudConverterBase::cleanupHook();
}
