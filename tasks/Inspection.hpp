/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PIPELINE_INSPECTION_INSPECTION_TASK_HPP
#define PIPELINE_INSPECTION_INSPECTION_TASK_HPP

#include "pipeline_inspection/InspectionBase.hpp"
#include "pipeline_inspection/DetectorTypes.hpp"
#include "pipeline_inspection/Detector.hpp"

namespace pipeline_inspection {

    /*! \class Inspection 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pipeline_inspection::Inspection')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Inspection : public InspectionBase
    {
	friend class InspectionBase;
    protected:
	RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> output_frame;

    public:
        /** TaskContext constructor for Inspection
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Inspection(std::string const& name = "pipeline_inspection::Inspection", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Inspection 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Inspection(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Inspection
         */
	~Inspection();

        bool configureHook();

        bool startHook();

        void updateHook();
	
    private:
	Detector detector;
	DetectorCalib calib;
	controlData::Pipeline lastPipe;
        
        base::samples::RigidBodyState lastPosition;
	
	double debug_minZ, debug_minY;
	double debug_maxZ, debug_maxY;
        
        void drawDebug(std::vector<base::Vector3d> &points, InspectionStatus &is);

    };
}

#endif

