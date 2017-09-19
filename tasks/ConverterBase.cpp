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

void ConverterBase::depth_mapAcquisitionTimeTransformerCallback(const base::Time& ts, const base::Time& depth_map_sample_start)
{
    // get current transforms set
    std::list<SampleTransforms>::reverse_iterator current_acquisition = depth_map_acquisition_transforms.rbegin();
    if(current_acquisition->sample_time_id != depth_map_sample_start)
    {
        // create next set
        depth_map_acquisition_transforms.push_back(SampleTransforms(depth_map_sample_start));
        current_acquisition = depth_map_acquisition_transforms.rbegin();
        current_acquisition->laser_in_odometry.reserve(acquisition_timestamps);
    }

    // receive transformation
    Eigen::Affine3d laserInOdometry;
    if(_laser2odometry.get(ts, laserInOdometry, true))
        current_acquisition->laser_in_odometry.push_back(laserInOdometry);
    else
        LOG_DEBUG_S << "Failed to receive a depth map acquisition transformation. Interpolation will be skipped!";
}

void ConverterBase::depth_mapTransformerCallback(const base::Time &ts, const ::base::samples::DepthMap &depth_map_sample)
{
    newSampleCallback(ts, depth_map_sample);
}

void ConverterBase::pullPorts()
{
    while( _depth_map.read(port_listener_depth_map_sample, false) == RTT::NewData )
    {
        // lazy registration of acquisition time stream
        if(depth_map_acquisition_times_idx_tr < 0 && motion_compensation != depth_map_preprocessing::NoCompensation)
            registerAcquisitionTimeStream();

        if((motion_compensation == depth_map_preprocessing::HorizontalInterpolation ||
            motion_compensation == depth_map_preprocessing::VerticalInterpolation) &&
            port_listener_depth_map_sample.timestamps.size() >= 2)
        {
            if(port_listener_depth_map_sample.timestamps.front() <= port_listener_depth_map_sample.timestamps.back())
            {
                _transformer.pushData(depth_map_acquisition_times_idx_tr, port_listener_depth_map_sample.timestamps.front(), port_listener_depth_map_sample.time);
                _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.timestamps.back(), port_listener_depth_map_sample);
            }
            else
            {
                _transformer.pushData(depth_map_acquisition_times_idx_tr, port_listener_depth_map_sample.timestamps.back(), port_listener_depth_map_sample.time);
                _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.timestamps.front(), port_listener_depth_map_sample);
            }
        }
        else if((motion_compensation == depth_map_preprocessing::Horizontal && port_listener_depth_map_sample.timestamps.size() == port_listener_depth_map_sample.horizontal_size) ||
                (motion_compensation == depth_map_preprocessing::Vertical && port_listener_depth_map_sample.timestamps.size() == port_listener_depth_map_sample.vertical_size))
        {
            acquisition_timestamps = port_listener_depth_map_sample.timestamps.size();
            if(port_listener_depth_map_sample.timestamps.front() <= port_listener_depth_map_sample.timestamps.back())
                pushDepthMapTimestamps(port_listener_depth_map_sample.timestamps.begin(), port_listener_depth_map_sample.timestamps.end()-1, port_listener_depth_map_sample);
            else
                pushDepthMapTimestamps(port_listener_depth_map_sample.timestamps.rbegin(), port_listener_depth_map_sample.timestamps.rend()-1, port_listener_depth_map_sample);
        }
        else
            _transformer.pushData(depth_map_idx_tr, port_listener_depth_map_sample.time, port_listener_depth_map_sample);
    }
}

void ConverterBase::registerAcquisitionTimeStream()
{
    // compute stream period
    double stream_period = _depth_map_period.value();
    if(motion_compensation == depth_map_preprocessing::Horizontal)
        stream_period /= (double)port_listener_depth_map_sample.horizontal_size;
    else if(motion_compensation == depth_map_preprocessing::Vertical)
        stream_period /= (double)port_listener_depth_map_sample.vertical_size;
    else
        stream_period *= 0.5;

    // register stream
    depth_map_acquisition_times_idx_tr = _transformer.registerDataStream< base::Time >(
            base::Time::fromSeconds(stream_period),
            boost::bind( &ConverterBase::depth_mapAcquisitionTimeTransformerCallback, this, _1, _2), -1, "depth_map_acquisition_times");
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConverterBase.hpp for more detailed
// documentation about them.

bool ConverterBase::configureHook()
{
    if (! ConverterBaseBase::configureHook())
        return false;

    depth_map_acquisition_times_idx_tr = -1;
    depth_map_acquisition_transforms.clear();
    depth_map_acquisition_transforms.push_back(SampleTransforms(base::Time::fromMicroseconds(0)));
    acquisition_timestamps = 1;
    motion_compensation = _motion_compensation.value();

    return true;
}
bool ConverterBase::startHook()
{
    if (! ConverterBaseBase::startHook())
        return false;

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

    if(depth_map_acquisition_times_idx_tr > 0)
        _transformer.unregisterDataStream(depth_map_acquisition_times_idx_tr);
}
