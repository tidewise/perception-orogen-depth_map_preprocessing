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
            Eigen::Affine3d laser_in_odometry;
        };

        virtual void depth_mapStartTimeTransformerCallback(const base::Time &ts, const base::Time &depth_map_sample_start);
        virtual void depth_mapTransformerCallback(const base::Time &ts, const base::samples::DepthMap &depth_map_sample);
        virtual void pullPorts();
        virtual void newSampleCallback(const base::Time &ts, const base::samples::DepthMap &depth_map_sample) = 0;

        bool getTransformations(const base::Time& ts, SampleTransforms& transformation) const;
        Eigen::Affine3d computeLocalTransfromation(const SampleTransforms& start, const SampleTransforms& end) const;

        template<class T>
        bool convertToPointCloud(const base::Time &ts, const base::samples::DepthMap &depth_map_sample, std::vector<T>& pointcloud);

        int depth_map_start_time_idx_tr;
        SampleTransforms depth_map_start_transforms;
        MotionCompensation motion_compensation;

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
    else if(depth_map_start_transforms.sample_time_id == depth_map_sample.time)
    {
        // convert scan to pointcloud with motion compensation
        SampleTransforms depth_map_end_transforms;
        if(getTransformations(ts, depth_map_end_transforms))
        {
            Eigen::Affine3d laser_endInLaser_start = computeLocalTransfromation(depth_map_start_transforms, depth_map_end_transforms);
            if(depth_map_sample.timestamps.front() <= depth_map_sample.timestamps.back())
                depth_map_sample.convertDepthMapToPointCloud(pointcloud, laser_endInLaser_start, Eigen::Affine3d::Identity(), true, true, motion_compensation == HorizontalInterpolation ? false : true);
            else
                depth_map_sample.convertDepthMapToPointCloud(pointcloud, Eigen::Affine3d::Identity(), laser_endInLaser_start.inverse(), true, true, motion_compensation == HorizontalInterpolation ? false : true);
            return true;
        }
    }
    return false;
}

}

#endif

