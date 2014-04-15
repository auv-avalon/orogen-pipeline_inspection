/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PIPELINE_INSPECTION_LASERSIMULATION_TASK_HPP
#define PIPELINE_INSPECTION_LASERSIMULATION_TASK_HPP

#include "pipeline_inspection/LaserSimulationBase.hpp"
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <vector>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace pipeline_inspection {

    /*! \class LaserSimulation 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pipeline_inspection::LaserSimulation')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class LaserSimulation : public LaserSimulationBase
    {
	friend class LaserSimulationBase;
    protected:



    public:
        /** TaskContext constructor for LaserSimulation
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        LaserSimulation(std::string const& name = "pipeline_inspection::LaserSimulation", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for LaserSimulation 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        LaserSimulation(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of LaserSimulation
         */
	~LaserSimulation();

        bool configureHook();

        bool startHook();

        void updateHook();
        
        
        
    private:
      
      void createPoints();

      void projectToLaserPlane();

      void convertToCameraFrame();
      
      void createNoise();
      
      std::vector<base::Vector3d> points;
      
      boost::variate_generator<boost::mt19937, boost::normal_distribution<> > *noise;

    };
}

#endif

