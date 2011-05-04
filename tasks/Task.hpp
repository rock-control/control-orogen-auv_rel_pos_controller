/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_REL_POS_CONTROLLER_TASK_TASK_HPP
#define AUV_REL_POS_CONTROLLER_TASK_TASK_HPP

#include "auv_rel_pos_controller/TaskBase.hpp"
#include <PID.hpp>

namespace auv_rel_pos_controller {
    
    const static double xyConstraint = 10;
    const static double zConstraint = 3;
    const static double angleConstraint = M_PI;
    
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        void constrainValues(base::AUVPositionCommand& posCommand);
        void constrainValue(double& value, const double& contraint);
        void constrainAngle(double& angle);
        
        motor_controller::PID xPID;
        motor_controller::PID yPID;
        base::AUVPositionCommand positionCommand;
        base::samples::RigidBodyState bodyState;
        double taskPeriod;
        double timeout;
        bool validBodyState;

    public:
        Task(std::string const& name = "auv_rel_pos_controller::Task");

	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
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
         * Call recovered() to go back in the Runtime state.
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
}

#endif

