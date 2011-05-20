/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace auv_rel_pos_controller;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    
}

Task::~Task()
{
}

void Task::constrainAngle(double& angle)
{
    if (angle < -M_PI)
        angle = angle + 2 * M_PI;
    else if (angle > M_PI)
        angle = angle - 2 * M_PI;
}

void Task::constrainValue(double& value, const double& contraint)
{
    if (value > contraint)
        value = contraint;
    else if (value < -contraint)
        value = -contraint;
}

void Task::constrainValues(base::AUVPositionCommand& posCommand)
{
    constrainValue(posCommand.x, xyConstraint);
    constrainValue(posCommand.y, xyConstraint);
    
    if (_rel_z.get()) 
    {
        constrainValue(posCommand.z, zConstraint);
    }
    else 
    {
    //this does not work with altitude
   /*     if (posCommand.z > 0)
            posCommand.z = 0;
        if (_fixed_z.get() > 0)
            _fixed_z.set(0);*/
    }
    
    if(_rel_heading.get())
    {
        constrainValue(posCommand.heading, angleConstraint);
    }
    else
    {
        constrainAngle(posCommand.heading);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    bodyState.invalidate();
    taskPeriod = 0;
    timeout = 0;
    validBodyState = false;
    xPID.reset();
    yPID.reset();
    
    return true;
}
bool Task::startHook()
{
    xPID.setPIDSettings(_controller_x.get());
    yPID.setPIDSettings(_controller_y.get());


    
    return true;
}
void Task::updateHook()
{
    
    // check if input ports are connected
    if (!_position_command.connected())
    {
        std::cerr << TaskContext::getName() << ": " 
                    << "Input port 'position_command' is not connected." << std::endl;
        return error(NOT_CONNECTED);
    }
    else if (!_position_sample.connected())
    {
        std::cerr << TaskContext::getName() << ": "
                    << "Input port 'position_sample' is not connected." << std::endl;
        return error(NOT_CONNECTED);
    }

    // get triggering interval of this task context
    if (taskPeriod <= 0) 
    {
        taskPeriod = TaskContext::getPeriod();
        if (taskPeriod <= 0)
        {
            std::cerr << TaskContext::getName() << ": " 
                        << "This task needs to be a periodic triggering task." << std::endl;
            return;
        }
    }
    
    if(_position_sample.readNewest(bodyState) != RTT::NoData)
    {
        validBodyState = true;
    }
    else
    {
        if(!validBodyState && (_rel_z.get() || _rel_heading.get()))
        {
            std::cerr << TaskContext::getName() << ": " 
                        << "Waiting for a valid RigidBodyState to set relative values"
                            << " for z or heading." << std::endl;
            return;
        }
    }
    
    if(_position_command.readNewest(positionCommand) != RTT::NoData) 
    {
        timeout = 0;
        constrainValues(positionCommand);
    }
    else 
    {
        timeout += taskPeriod;
        if (timeout >= _timeout.get()) 
        {
            std::cerr << TaskContext::getName() << ": "
                        << "Receiving no new AUVPositionCommand data for longer than "
                            << _timeout.get() << " seconds." << std::endl;
            std::cerr << TaskContext::getName() << ": "
                        << "Switching to task state RUNTIME_ERROR now!" << std::endl;

            return error(TIMEOUT);
        }
    }
    
    base::AUVMotionCommand motion_command;
    // set x,y speed
    if (positionCommand.x == 0)
        motion_command.x_speed = 0;
    else
        motion_command.x_speed = xPID.update(-positionCommand.x,0);
    if (positionCommand.y == 0)
        motion_command.y_speed = 0;
    else
        motion_command.y_speed = yPID.update(-positionCommand.y,0);
    
    // set depth
    if (_fixed_z.get() > -9999)
    {
        motion_command.z = _fixed_z.get();
    }
    else if (_rel_z.get())
    {
        motion_command.z = bodyState.position.z() + positionCommand.z;
        if (motion_command.z > 0)
            motion_command.z = 0;
    }
    else
    {
        motion_command.z = positionCommand.z;
    }
    
    // set heading
    if (_rel_heading.get())
    {
        double heading = base::getYaw(bodyState.orientation);
        motion_command.heading = heading + positionCommand.heading;
        constrainAngle(motion_command.heading);
    }
    else
    {
        motion_command.heading = positionCommand.heading;
    }
    
    // write motion command
    if(_motion_command.connected())
        _motion_command.write(motion_command);
    else
        std::cerr << TaskContext::getName() << ": " 
                    << "Output port motion_command doesn't seem to be connected." << std::endl;
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

