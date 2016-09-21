/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConverterBase.hpp"

using namespace depth_map_preprocessing;

ConverterBase::ConverterBase(std::string const& name)
    : ConverterBaseBase(name)
{
}

ConverterBase::ConverterBase(std::string const& name, RTT::ExecutionEngine* engine)
    : ConverterBaseBase(name, engine)
{
}

ConverterBase::~ConverterBase()
{
}

void ConverterBase::depth_mapStartTimeTransformerCallback(const base::Time& ts, const base::Time& depth_map_sample_start)
{
    if(getTransformations(ts, depth_map_start_transforms))
        depth_map_start_transforms.sample_time_id = depth_map_sample_start;
    else
        depth_map_start_transforms.sample_time_id.microseconds = 0;
}

void ConverterBase::depth_mapTransformerCallback(const base::Time &ts, const ::base::samples::DepthMap &depth_map_sample)
{
    newSampleCallback(ts, depth_map_sample);
}

void ConverterBase::pullPorts()
{
    bool keepGoing = true;
    bool hasData =  true ;

    while(keepGoing)
    {
        keepGoing = false;

        if(hasData && _depth_map.read(port_listener_depth_map_sample, false) == RTT::NewData )
        {
            if(_motion_compensation.value() && port_listener_depth_map_sample.timestamps.size() >= 2)
            {
                if(port_listener_depth_map_sample.timestamps.front() <= port_listener_depth_map_sample.timestamps.back())
                {
                    _transformer.pushData(depth_map_start_time_idx_tr, port_listener_depth_map_sample.timestamps.front(), port_listener_depth_map_sample.time);
                    _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.timestamps.back(), port_listener_depth_map_sample);
                }
                else
                {
                    _transformer.pushData(depth_map_start_time_idx_tr, port_listener_depth_map_sample.timestamps.back(), port_listener_depth_map_sample.time);
                    _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.timestamps.front(), port_listener_depth_map_sample);
                }
            }
            else
                _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.time, port_listener_depth_map_sample);
            keepGoing = true;
        }
        else
            hasData = false;
    }
}

bool ConverterBase::getTransformations(const base::Time& ts, SampleTransforms& transformation) const
{
    // get transformations
    if (!_laser2odometry.get(ts, transformation.laser_in_odometry, true))
    {
        LOG_ERROR_S << "skip interpolation, have no laserInOdometry transformation sample!";
        return false;
    }
    return true;
}

Eigen::Affine3d ConverterBase::computeLocalTransfromation(const ConverterBase::SampleTransforms& start, const ConverterBase::SampleTransforms& end) const
{
    // computes the pose of the laser at t_end expressed in t_start;
    return start.laser_in_odometry.inverse() * end.laser_in_odometry;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConverterBase.hpp for more detailed
// documentation about them.

bool ConverterBase::configureHook()
{
    if (! ConverterBaseBase::configureHook())
        return false;

    depth_map_start_time_idx_tr = _transformer.registerDataStream< base::Time >(
            base::Time::fromSeconds(_depth_map_period.value()),
            boost::bind( &ConverterBase::depth_mapStartTimeTransformerCallback, this, _1, _2), -1, "depth_map_start_time");

    depth_map_start_transforms.sample_time_id.microseconds = 0;

    motion_compensation = _motion_compensation.value();

    return true;
}
bool ConverterBase::startHook()
{
    if (! ConverterBaseBase::startHook())
        return false;

    if( !_depth_map.connected() ) _transformer.disableStream(depth_map_start_time_idx_tr);

    return true;
}
void ConverterBase::updateHook()
{
    ConverterBaseBase::updateHook();
}
void ConverterBase::errorHook()
{
    ConverterBaseBase::errorHook();
}
void ConverterBase::stopHook()
{
    ConverterBaseBase::stopHook();
}
void ConverterBase::cleanupHook()
{
    ConverterBaseBase::cleanupHook();

    _transformer.unregisterDataStream(depth_map_start_time_idx_tr);
}
