/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace auv_rel_pos_controller;
const float message_interval = 1;           

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    return true;
}
bool Task::startHook()
{
    base::Time time = base::Time::now();
    last_valid_motion_command = time;

    x_pid.reset();
    y_pid.reset();
    x_pid.setPIDSettings(_controller_x.get());
    y_pid.setPIDSettings(_controller_y.get());
    x_pid.printCoefficients();
    y_pid.printCoefficients();
    inital_heading = std::numeric_limits<double>::infinity();
    body_state.invalidate();
    last_state = RUNNING;
    return true;
}

double Task::constrainAngle(double angle)
{
    return angle -M_PI*((int)(angle/M_PI));
}

void Task::updateHook()
{
    States act_state = RUNNING;
    base::Time time = base::Time::now();

    //read new body state
    if(_position_sample.read(body_state_temp)==RTT::NewData)
    {
        body_state = body_state_temp;
        last_position_sample_update = time;             //we do not want to get confused if
    }                                                   //the time of an other computer differs 

    //check for timeout 
    //if the body_state is too old it is no longer valid
    if((time-last_position_sample_update).toSeconds() > _valid_timespan.get()) 
    {
        if((time-last_log_message).toSeconds()>message_interval) 
        {
            std::cout << TaskContext::getName() << ": " 
                << "Receiving no valid RigidBodyState data for "
                << (time-last_position_sample_update).toSeconds() << " seconds." << std::endl;
        }
        body_state.invalidate();
        act_state = WAITING_FOR_VALID_BODYSTATE;
    }
//    if(!body_state.hasValidOrientation())
//        act_state = WAITING_FOR_VALID_BODYSTATE;


    //read new position command
    if(_position_command.read(position_command_temp) == RTT::NewData) 
    {
        position_command = position_command_temp;
        last_position_command_update = time;

        //special mode if in rel heading mode-- keep inital heading
        if(std::numeric_limits<double>::infinity() == position_command.heading && 
                _rel_heading.get())
        {
            //set inital heading
            if(inital_heading == std::numeric_limits<double>::infinity())
            {
                position_command.heading = 0;
                if(act_state == RUNNING)
                {
                    inital_heading = base::getYaw(body_state.orientation);
                    std::cout << "Set inital heading to: " << inital_heading << std::endl;
                }
                else
                    std::cout << "Could not set inital heading" << std::endl;
            }
            else
                position_command.heading = constrainAngle(inital_heading-base::getYaw(body_state.orientation));
        }
        else
        {
            inital_heading = std::numeric_limits<double>::infinity();
        }

        //check values and go into exception if someone
        //is sending invalid commands
        if(std::abs(position_command.heading) > M_PI || 
                std::numeric_limits<double>::infinity() == position_command.x ||
                std::numeric_limits<double>::infinity() == position_command.y ||
                std::numeric_limits<double>::infinity() == position_command.z)
        {
            std::cerr << "Invalid PositionCommand: " << std::endl;
            std::cerr << "posx=: " << position_command.x << ", posy=: "<< position_command.y << 
                "posy=: " << position_command.y << ", heading=: "<< position_command.heading << std::endl;
            return exception(INVALID_POSITION_COMMAND);
        }
    }

    ////////////check for timeout///////////////
    if((time-last_position_command_update).toSeconds() > _valid_timespan.get())
    {
        if((time-last_log_message).toSeconds()>message_interval) 
        {
            std::cout << TaskContext::getName() << ": " 
                << "Receiving no valid PositionCommand data for "
                << (time-last_position_command_update).toSeconds() << " seconds." << std::endl;
        }
        act_state = WAITING_FOR_POSITION_COMMAND;
    }

    /////////////////////////////////////////////
    //////////generate motion command////////////
    /////////////////////////////////////////////
    if(act_state == RUNNING)
    {
        // set x,y speed
        motion_command.x_speed = x_pid.update(-position_command.x,0);
        motion_command.y_speed = y_pid.update(-position_command.y,0);

        if (_rel_z.get())
            motion_command.z = body_state.position.z() + position_command.z;
        else
            motion_command.z = position_command.z;

        // set heading
        if (_rel_heading.get())
            motion_command.heading = constrainAngle(base::getYaw(body_state.orientation)+position_command.heading);
        else
            motion_command.heading = position_command.heading;
        last_valid_motion_command = time;
    }
    else
    {
        if((time-last_log_message).toSeconds() > message_interval)
            last_log_message = time;

        //generate a zero motion command
        motion_command.x_speed = 0;
        motion_command.y_speed = 0;
        motion_command.z = body_state.position.z();
        motion_command.heading = base::getYaw(body_state.orientation);

        //go into exception after n secs
        if((time-last_valid_motion_command).toSeconds() > _timeout.get()&& _timeout.get() > 0)
        {
            std::cerr << "No valid MotionCommands were generated for " <<
                (time-last_valid_motion_command).toSeconds() <<
                " seconds." << std::endl;
            return exception(TIMEOUT);
        }
    }

    //write state if it has changed
    if(last_state != act_state)
    {
        last_state = act_state;
        state(act_state);
    }
    _motion_command.write(motion_command);
}

void Task::errorHook()
{

}

void Task::stopHook()
{
}

void Task::cleanupHook()
{
}

