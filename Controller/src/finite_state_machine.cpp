/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2019]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <finite_state_machine.hpp>

finite_state_machine::finite_state_machine(const int num_of_joints,
                                           const int num_of_segments,
                                           const int num_of_frames,
                                           const int num_of_constraints):
    NUM_OF_JOINTS_(num_of_joints), NUM_OF_SEGMENTS_(num_of_segments),
    NUM_OF_FRAMES_(num_of_frames), NUM_OF_CONSTRAINTS_(num_of_constraints),
    END_EFF_(NUM_OF_SEGMENTS_ - 1), desired_task_model_(task_model::full_pose),
    motion_profile_(m_profile::CONSTANT), total_control_time_sec_(0.0),
    goal_reached_(false), time_limit_reached_(false), contact_detected_(false),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    desired_state_(robot_state_), current_error_()
{
}

int finite_state_machine::initialize_with_moveConstrained_follow_path(const moveConstrained_follow_path_task &task,
                                                                      const int motion_profile)
{
    desired_task_model_               = task_model::moveConstrained_follow_path;
    moveConstrained_follow_path_task_ = task;
    motion_profile_                   = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveTo_follow_path(const moveTo_follow_path_task &task,
                                                             const int motion_profile)
{
    desired_task_model_      = task_model::moveTo_follow_path;
    moveTo_follow_path_task_ = task;
    motion_profile_          = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveTo(const moveTo_task &task,
                                                 const int motion_profile)
{
    desired_task_model_ = task_model::moveTo;
    moveTo_task_        = task;
    motion_profile_     = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::initialize_with_full_pose(const full_pose_task &task,
                                                    const int motion_profile)
{
    desired_task_model_ = task_model::full_pose;
    full_pose_task_     = task;
    motion_profile_     = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::update_moveConstrained_follow_path_task(state_specification &desired_state,
                                                                  const int tube_section_count)
{
    if (goal_reached_ || contact_detected_) return control_status::STOP_ROBOT;

    if (total_control_time_sec_ > moveConstrained_follow_path_task_.time_limit) 
    {
        #ifndef NDEBUG       
            printf("Time limit reached\n");
        #endif

        time_limit_reached_ = true;
        return control_status::STOP_CONTROL;
    }
    time_limit_reached_ = false;

    if (contact_detected(moveConstrained_follow_path_task_.contact_threshold_linear, 
                         moveConstrained_follow_path_task_.contact_threshold_angular))
    {
        #ifndef NDEBUG       
            printf("Contact occurred\n");
        #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return control_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    bool final_section_reached = false;
    if (tube_section_count == moveConstrained_follow_path_task_.tf_poses.size() - 1) final_section_reached = true;
    
    // Check if the current pose of the robot satisfies all 6D tolerances
    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[i]) count++;
    }

    // Check if the robot has reached end of the tube path
    if (count == NUM_OF_CONSTRAINTS_ && final_section_reached) 
    {   
        #ifndef NDEBUG       
            printf("Whole path covered\n");
        #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::STOP_ROBOT;
    }
    goal_reached_ = false;

    /*
     * The robot has reached goal x axis area but it is out of y and z area?
     * If yes command zero X linear velocity, to keep it in that x area.
     * Else go with initially commanded tube speed.
    */
    if ((count == NUM_OF_CONSTRAINTS_) || \
        ((count == NUM_OF_CONSTRAINTS_ - 1) && (std::fabs(current_error_(0)) > moveConstrained_follow_path_task_.tube_tolerances[0])))
    {
        double speed = 0.0;
        switch (motion_profile_)
        {
            case m_profile::STEP:
                speed = motion_profile::negative_step_function(std::fabs(current_error_(0)), 
                                                               moveConstrained_follow_path_task_.tube_speed, 
                                                               0.25, 0.4, 0.2);
                break;

            case m_profile::S_CURVE:
                speed = motion_profile::s_curve_function(std::fabs(current_error_(0)), 
                                                         0.05, 
                                                         moveConstrained_follow_path_task_.tube_speed, 5.0);
                break;

            default:
                speed = moveConstrained_follow_path_task_.tube_speed;
                break;
        }

        // Check for necessary direction of motion
        if ((sign(current_error_(0)) == -1) && final_section_reached) speed = -1 * speed;      
        desired_state.frame_velocity[END_EFF_].vel(0) = speed;      

        // Robot has crossed some tube section? If yes, switch to next one.
        if ((current_error_(0) < moveConstrained_follow_path_task_.tube_tolerances[0]) && !final_section_reached)
        {
            return control_status::CHANGE_TUBE_SECTION;
        }
        return control_status::CRUISE_THROUGH_TUBE;        
    }
    
    else
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::START_TO_CRUISE;
    }
}


int finite_state_machine::update_moveTo_follow_path_task(state_specification &desired_state,
                                                         const int tube_section_count)
{
    if (goal_reached_ || contact_detected_) return control_status::STOP_ROBOT;

    if (total_control_time_sec_ > moveTo_follow_path_task_.time_limit) 
    {
        #ifndef NDEBUG       
            if (!time_limit_reached_) printf("Time limit reached\n");
        #endif

        time_limit_reached_ = true;
        return control_status::STOP_CONTROL;
    }

    if (contact_detected(moveTo_follow_path_task_.contact_threshold_linear, 
                         moveTo_follow_path_task_.contact_threshold_angular))
    {
        #ifndef NDEBUG       
            printf("Contact occurred\n");
        #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return control_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    bool final_section_reached = false;
    if (tube_section_count == moveTo_follow_path_task_.tf_poses.size() - 1) final_section_reached = true;
    
    // Check if the current pose of the robot satisfies all 6D tolerances
    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveTo_follow_path_task_.tube_tolerances[i]) count++;
    }

    // Check if the robot has reached end of the tube path
    if (count == NUM_OF_CONSTRAINTS_ && final_section_reached) 
    {   
        #ifndef NDEBUG       
            printf("Whole path covered\n");
        #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::STOP_ROBOT;
    }
    goal_reached_ = false;

    /*
     * The robot has reached goal x axis area but it is out of y and z area?
     * If yes command zero X linear velocity, to keep it in that x area.
     * Else go with initially commanded tube speed.
    */
    if ((count == NUM_OF_CONSTRAINTS_) || \
        ((count == NUM_OF_CONSTRAINTS_ - 1) && (std::fabs(current_error_(0)) > moveTo_follow_path_task_.tube_tolerances[0])))
    {
        double speed = 0.0;
        switch (motion_profile_)
        {
            case m_profile::STEP:
                speed = motion_profile::negative_step_function(std::fabs(current_error_(0)), 
                                                               moveTo_follow_path_task_.tube_speed, 
                                                               0.25, 0.4, 0.2);
                break;

            case m_profile::S_CURVE:
                speed = motion_profile::s_curve_function(std::fabs(current_error_(0)), 
                                                         0.05, 
                                                         moveTo_follow_path_task_.tube_speed, 5.0);
                break;

            default:
                speed = moveTo_follow_path_task_.tube_speed;
                break;
        }

        // Check for necessary direction of motion
        if ((sign(current_error_(0)) == -1) && final_section_reached) speed = -1 * speed;      
        desired_state.frame_velocity[END_EFF_].vel(0) = speed;      

        // Robot has crossed some tube section? If yes, switch to next one.
        if ((current_error_(0) < moveTo_follow_path_task_.tube_tolerances[0]) && !final_section_reached)
        {
            return control_status::CHANGE_TUBE_SECTION;
        }
        return control_status::CRUISE_THROUGH_TUBE;        
    }
    
    else
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::START_TO_CRUISE;
    }
}


int finite_state_machine::update_moveTo_task(state_specification &desired_state)
{
    if (goal_reached_ || contact_detected_) return control_status::STOP_ROBOT;

    if (total_control_time_sec_ > moveTo_task_.time_limit) 
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;

        #ifndef NDEBUG       
            if(!time_limit_reached_) printf("Time limit reached\n");
        #endif

        time_limit_reached_ = true;
        return control_status::STOP_CONTROL;
    }
    
    if(contact_detected(moveTo_task_.contact_threshold_linear, 
                        moveTo_task_.contact_threshold_angular))
    {
        #ifndef NDEBUG       
            printf("Contact occurred\n");
        #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return control_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveTo_task_.tube_tolerances[i]) count++;
    }
    
    if (count == NUM_OF_CONSTRAINTS_) 
    {
        #ifndef NDEBUG       
            printf("Goal area reached\n");
        #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::STOP_ROBOT;
    }
    goal_reached_ = false;

    /*
     * The robot has reached goal x area?
     * If yes, command zero X linear velocity to keep it in that area, until all DOFs gets back into tube.
     * Else go with initially commanded tube speed.
    */
    if ( (std::fabs(current_error_(0)) <= moveTo_task_.tube_tolerances[0]) || \
         ((std::fabs(current_error_(0)) >  moveTo_task_.tube_tolerances[0]) && \
          (count < NUM_OF_CONSTRAINTS_ - 1)) 
       )
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return control_status::START_TO_CRUISE;
    }
    else
    {
        double speed = 0.0;
        switch (motion_profile_)
        {
            case m_profile::STEP:
                speed = motion_profile::negative_step_function(std::fabs(current_error_(0)), 
                                                               moveTo_task_.tube_speed, 
                                                               0.25, 0.4, 0.2);
                break;

            case m_profile::S_CURVE:
                speed = motion_profile::s_curve_function(std::fabs(current_error_(0)), 
                                                         0.05, moveTo_task_.tube_speed, 5.0);
                break;

            default:
                speed = moveTo_task_.tube_speed;
                break;
        }

        if (sign(current_error_(0)) == -1) desired_state.frame_velocity[END_EFF_].vel(0) = -1 * speed;      
        else desired_state.frame_velocity[END_EFF_].vel(0) = speed;              
    }
   
    return control_status::CRUISE_THROUGH_TUBE;
}

int finite_state_machine::update_full_pose_task(state_specification &desired_state)
{
    if (goal_reached_ || contact_detected_) return control_status::STOP_ROBOT;

    if(total_control_time_sec_ > full_pose_task_.time_limit) 
    {
        #ifndef NDEBUG
            if(!time_limit_reached_) printf("Time limit reached\n");
        #endif

        time_limit_reached_ = true;
        return control_status::STOP_CONTROL;
    }

    if(contact_detected(full_pose_task_.contact_threshold_linear, 
                        full_pose_task_.contact_threshold_angular))
    {
        #ifndef NDEBUG       
            printf("Contact occurred\n");
        #endif
        contact_detected_ = true;
        return control_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= full_pose_task_.goal_area[i]) count++;
    }
    
    if(count == NUM_OF_CONSTRAINTS_) 
    {
        #ifndef NDEBUG       
            printf("Goal area reached\n");
        #endif

        goal_reached_ = true;
        return control_status::STOP_ROBOT;
    }
    else goal_reached_ = false;

    return control_status::NOMINAL;
}

int finite_state_machine::update_motion_task_status(const state_specification &robot_state,
                                                    state_specification &desired_state,
                                                    const KDL::Twist &current_error,
                                                    const KDL::Wrench &ext_force,
                                                    const double time_passed_sec,
                                                    const int tube_section_count)
{
    robot_state_            = robot_state;
    desired_state_          = desired_state;
    current_error_          = current_error;
    ext_wrench_             = ext_force;
    total_control_time_sec_ = time_passed_sec;

    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            return update_moveConstrained_follow_path_task(desired_state, tube_section_count);
            break;

        case task_model::moveTo_follow_path:
            return update_moveTo_follow_path_task(desired_state, tube_section_count);
            break;

        case task_model::moveTo:
            return update_moveTo_task(desired_state);
            break;
        
        case task_model::full_pose:
            return update_full_pose_task(desired_state);
            break;
            
        default:
            printf("Unsupported task model\n");
            return control_status::STOP_CONTROL;
            break;
    }
}

int finite_state_machine::update_force_task_status(const KDL::Wrench &desired_force,
                                                   const KDL::Wrench &ext_force)
{
    if (!contact_secured(desired_force, ext_force)) return control_status::APPROACH;
    return control_status::CRUISE;
}

bool finite_state_machine::contact_secured(const KDL::Wrench &desired_force,
                                           const KDL::Wrench &ext_force)
{
    for (int i = 3; i < 5; i++)
    {
        double error = desired_force(i) - ext_force(i);
        if (std::fabs(error) > moveConstrained_follow_path_task_.tube_tolerances[i]) return false;
    }

    // std::cout << desired_force(2) << " " << desired_force(3) << " " << desired_force(4)<< std::endl;
    // std::cout << ext_force(2) << " " << ext_force(3) << " " << ext_force(4)<< std::endl;
    return true;
}

bool finite_state_machine::force_goal_maintained(const KDL::Wrench &desired_force,
                                                 const KDL::Wrench &ext_force)
{
    for (int i = 2; i < 5; i++)
    {
        double error = desired_force(i) - ext_force(i);
        if (std::fabs(error) > moveConstrained_follow_path_task_.tube_tolerances[i]) return false;
    }

    // std::cout << desired_force(2) << " " << desired_force(3) << " " << desired_force(4)<< std::endl;
    // std::cout << ext_force(2) << " " << ext_force(3) << " " << ext_force(4)<< std::endl;
    return true;
}

bool finite_state_machine::contact_detected(const double linear_force_threshold, 
                                            const double angular_force_threshold)
{

    if (desired_task_model_ == task_model::moveConstrained_follow_path)
    {
        for (int i = 0; i < 2; i++)
        {
            if (std::fabs(ext_wrench_(i)) > linear_force_threshold) return true;
        }
    }

    else 
    {
        for (int i = 0; i < 3; i++)
        {
            if (std::fabs(ext_wrench_(i)) > linear_force_threshold) return true;
        }

        for (int i = 3; i < 6; i++)
        {
            if (std::fabs(ext_wrench_(i)) > angular_force_threshold) return true;
        }
    }

    return false;    
}

int finite_state_machine::sign(double x)
{
    if (x > 0.0) return 1;
    else if (x < 0.0) return -1;
    else return 0;
}