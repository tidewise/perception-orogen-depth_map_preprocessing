/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DEPTH_MAP_PREPROCESSING_CONVERTERBASE_TASK_HPP
#define DEPTH_MAP_PREPROCESSING_CONVERTERBASE_TASK_HPP

#include "depth_map_preprocessing/ConverterBaseBase.hpp"
#include "depth_map_preprocessingTypes.hpp"

namespace depth_map_preprocessing{

    /*! \class ConverterBase
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','depth_map_preprocessing::ConverterBase')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class ConverterBase : public ConverterBaseBase
    {
	friend class ConverterBaseBase;
    protected:

        struct SampleTransforms
        {
            base::Time sample_time_id;
            std::vector<Eigen::Affine3d> laser_in_odometry;

            SampleTransforms(const base::Time& sample_time_id) : sample_time_id(sample_time_id) {}

            void clear()
            {
                sample_time_id.microseconds = 0;
                laser_in_odometry.clear();
            }
        };


        /**
         * Will be called if a new sample is availale and must be implemented in subclasses.
         * Should call @convertToPointCloud to convert the sample to the pointcloud of choice.
         */
        virtual void newSampleCallback(const base::Time &ts, const base::samples::DepthMap &depth_map_sample) = 0;

        /**
         * Transformer callback with the purpose to receive the depth map acquisition transformations
         */
        virtual void depth_mapAcquisitionTimeTransformerCallback(const base::Time &ts, const base::Time &depth_map_sample_start);

        /**
         * Transformer callback of the actual sample
         * Only calls @newSampleCallback
         */
        virtual void depth_mapTransformerCallback(const base::Time &ts, const base::samples::DepthMap &depth_map_sample);

        /**
         * Reads out the input ports and forwards the samples to the transformer
         */
        virtual void pullPorts();

        /**
         * Converts the given sample to a pointcloud while applying the selected motion compensation option.
         */
        template<class T>
        bool convertToPointCloud(const base::Time &ts, const base::samples::DepthMap &depth_map_sample, std::vector<T>& pointcloud);

        /**
         * Helper method to register the depth map acquisition transformations stream in a lazy fasion.
         * This is important since the period information might be computed based on the amount of timestamps.
         */
        void registerAcquisitionTimeStream();

        /**
         * Helper method to compute the relative transformations inside of one scan
         */
        void computeLocalTransfromations(const SampleTransforms& transformations, const Eigen::Affine3d& latest, std::vector<Eigen::Affine3d>& laserLinesToLatestLine) const;

        /**
         * Helper method to push acquisition timestamps to the transformer
         */
        template<class ElementIterator>
        void pushDepthMapTimestamps(ElementIterator it_first, ElementIterator it_last, const base::samples::DepthMap &depth_map_sample);

    protected:

        int depth_map_acquisition_times_idx_tr;
        std::list<SampleTransforms> depth_map_acquisition_transforms;
        MotionCompensation motion_compensation;
        unsigned acquisition_timestamps;

    public:
        /** TaskContext constructor for ConverterBase
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        ConverterBase(std::string const& name = "depth_map_preprocessing::ConverterBase");

        /** TaskContext constructor for ConverterBase
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        ConverterBase(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of ConverterBase
         */
	~ConverterBase();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };


template<class T>
bool ConverterBase::convertToPointCloud(const base::Time& ts, const base::samples::DepthMap& depth_map_sample, std::vector< T >& pointcloud)
{
    if(motion_compensation == NoCompensation)
    {
        depth_map_sample.convertDepthMapToPointCloud(pointcloud, true);
        return true;
    }

    // drop older acquisition transforms sets
    std::list<SampleTransforms>::iterator acquisition_transforms = depth_map_acquisition_transforms.begin();
    while(acquisition_transforms->sample_time_id < depth_map_sample.time && depth_map_acquisition_transforms.size() > 1)
        acquisition_transforms = depth_map_acquisition_transforms.erase(acquisition_transforms);

    // check if acquisition transforms are the right ones
    if(acquisition_transforms->sample_time_id == depth_map_sample.time)
    {
        // convert scan to pointcloud with motion compensation
        Eigen::Affine3d depth_map_end_transform;
        if(_laser2odometry.get(ts, depth_map_end_transform, true))
        {
            std::vector<Eigen::Affine3d> laserLinesToLatestLine;
            computeLocalTransfromations(*acquisition_transforms, depth_map_end_transform, laserLinesToLatestLine);

            if((motion_compensation == HorizontalInterpolation || motion_compensation == VerticalInterpolation) && laserLinesToLatestLine.size() == 1)
            {
                if(depth_map_sample.timestamps.front() <= depth_map_sample.timestamps.back())
                    depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine.front(), Eigen::Affine3d::Identity(),
                                                                 true, true, motion_compensation == HorizontalInterpolation ? false : true);
                else
                    depth_map_sample.convertDepthMapToPointCloud(pointcloud, Eigen::Affine3d::Identity(), laserLinesToLatestLine.front().inverse(),
                                                                 true, true, motion_compensation == HorizontalInterpolation ? false : true);
                return true;
            }
            else if((motion_compensation == Horizontal && laserLinesToLatestLine.size() == depth_map_sample.horizontal_size-1) ||
                    (motion_compensation == Vertical && laserLinesToLatestLine.size() == depth_map_sample.vertical_size-1))
            {
                laserLinesToLatestLine.push_back(Eigen::Affine3d::Identity());
                if(depth_map_sample.timestamps.front() <= depth_map_sample.timestamps.back())
                    depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine,
                                                                 true, true, motion_compensation == Horizontal ? false : true);
                else
                {
                    std::vector<Eigen::Affine3d> laserLinesToLatestLine_reverse(laserLinesToLatestLine.size());
                    for(unsigned i = 1; i <= laserLinesToLatestLine.size(); i++)
                        laserLinesToLatestLine_reverse[laserLinesToLatestLine.size()-i] = laserLinesToLatestLine[i-1].inverse();
                    depth_map_sample.convertDepthMapToPointCloud(pointcloud, laserLinesToLatestLine_reverse,
                                                                 true, true, motion_compensation == Horizontal ? false : true);
                }
                return true;
            }
        }
    }
    else
        LOG_DEBUG_S << "No acquisition transforms for the current sample available!";

    LOG_ERROR_S << "Failed to convert depth map to point cloud, missing scan acquisition transformations!";
    return false;
}

template<class ElementIterator>
void ConverterBase::pushDepthMapTimestamps(ElementIterator it_first, ElementIterator it_last, const base::samples::DepthMap& depth_map_sample)
{
    for(;it_first != it_last; it_first++)
        _transformer.pushData(depth_map_acquisition_times_idx_tr, *it_first, depth_map_sample.time);
    _transformer.pushData(depth_map_idx_tr, *it_last, depth_map_sample);
}

}

#endif
