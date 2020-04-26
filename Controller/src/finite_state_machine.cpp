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
    motion_profile_(m_profile::CONSTANT), loop_period_count_(0),
    compensator_trigger_count_(0), total_control_time_sec_(0.0),
    previous_task_time_(0.0), total_contact_time_(0.0),
    goal_reached_(false), time_limit_reached_(false), contact_detected_(false),
    contact_alignment_performed_(false), write_compensation_time_to_file_(false), 
    filtered_bias_(Eigen::VectorXd::Zero(6)), compensation_parameters_(Eigen::VectorXd::Zero(12)),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    desired_state_(robot_state_), 
    variance_gain_(100, 6), variance_bias_(100, 6), slope_bias_(100, 6),
    current_error_(KDL::Twist::Zero()), ext_wrench_(KDL::Wrench::Zero())
{
}

int finite_state_machine::initialize_with_moveConstrained_follow_path(const moveConstrained_follow_path_task &task,
                                                                      const int motion_profile)
{
    desired_task_model_               = task_model::moveConstrained_follow_path;
    moveConstrained_follow_path_task_ = task;
    motion_profile_                   = motion_profile;

    // log_file_ext_force_.open("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/ext_force_data.txt");
    // assert(log_file_ext_force_.is_open());

    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveTo_follow_path(const moveTo_follow_path_task &task,
                                                             const int motion_profile)
{
    desired_task_model_      = task_model::moveTo_follow_path;
    moveTo_follow_path_task_ = task;
    motion_profile_          = motion_profile;

    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveTo(const moveTo_task &task,
                                                 const int motion_profile)
{
    desired_task_model_ = task_model::moveTo;
    moveTo_task_        = task;
    motion_profile_     = motion_profile;

    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveGuarded(const moveGuarded_task &task,
                                                      const int motion_profile)
{
    desired_task_model_ = task_model::moveGuarded;
    moveGuarded_task_   = task;
    motion_profile_     = motion_profile;

    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_moveTo_weight_compensation(const moveTo_weight_compensation_task &task,
                                                                     const int motion_profile,
                                                                     const Eigen::VectorXd &compensation_parameters)
{
    desired_task_model_              = task_model::moveTo_weight_compensation;
    moveTo_weight_compensation_task_ = task;
    motion_profile_                  = motion_profile;
    compensation_parameters_         = compensation_parameters;

    log_file_compensation_.open("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/compensation_data.txt");
    assert(log_file_compensation_.is_open());
    log_file_compensation_ << compensation_parameters_(5) << " ";
    log_file_compensation_ << compensation_parameters_(6) << " ";
    log_file_compensation_ << compensation_parameters_(7) << std::endl;
    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_full_pose(const full_pose_task &task,
                                                    const int motion_profile)
{
    desired_task_model_ = task_model::full_pose;
    full_pose_task_     = task;
    motion_profile_     = motion_profile;

    return task_status::NOMINAL;
}

int finite_state_machine::initialize_with_gravity_compensation(const gravity_compensation_task &task)
{
    desired_task_model_        = task_model::gravity_compensation;
    gravity_compensation_task_ = task;

    return task_status::NOMINAL;
}

int finite_state_machine::update_moveConstrained_follow_path_task(state_specification &desired_state,
                                                                  const int tube_section_count)
{
    if (total_control_time_sec_ > moveConstrained_follow_path_task_.time_limit) 
    {
        #ifndef NDEBUG       
            printf("Time limit reached\n");
        #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (goal_reached_) return task_status::STOP_ROBOT;

    bool final_section_reached = false;
    if ((unsigned)tube_section_count == moveConstrained_follow_path_task_.tf_poses.size() - 1) final_section_reached = true;
    
    // Check if the current pose of the robot satisfies 2D tolerances
    int count = 0;
    for (int i = 0; i < 2; i++)
    {
        if (std::fabs(current_error_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[i]) count++;
    }

    // Check if the robot has reached the final goal area
    if (count == 2 && final_section_reached) 
    {   
        #ifndef NDEBUG       
            printf("Whole path covered\n");
        #endif

        goal_reached_ = true;
        return task_status::STOP_ROBOT;
    }

    /*
     * The robot is out of y area?
     * If yes command zero X linear velocity, to keep it in that x area.
     * Else go with initially commanded tube speed.
    */
    if (std::fabs(current_error_(1)) <= moveConstrained_follow_path_task_.tube_tolerances[1])
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
        if ((sign(current_error_(0)) == -1) && final_section_reached)
        {
            speed = -1 * speed;
            // #ifndef NDEBUG       
            //     printf("Motion reversed.\n");
            // #endif
        }
        desired_state.frame_velocity[END_EFF_].vel(0) = speed;      

        // Robot has crossed some tube section? If yes, switch to next one.
        if ((current_error_(0) < moveConstrained_follow_path_task_.tube_tolerances[0]) && !final_section_reached)
        {
            return task_status::CHANGE_TUBE_SECTION;
        }
        return task_status::CRUISE_THROUGH_TUBE;        
    }
    else
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::START_TO_CRUISE;
    }
}


int finite_state_machine::update_moveTo_follow_path_task(state_specification &desired_state,
                                                         const int tube_section_count)
{
    if (total_control_time_sec_ > moveTo_follow_path_task_.time_limit) 
    {
        // #ifndef NDEBUG       
            if (!time_limit_reached_) printf("Time limit reached\n");
        // #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (goal_reached_ || contact_detected_) return task_status::STOP_ROBOT;

    if (contact_detected(moveTo_follow_path_task_.contact_threshold_linear, 
                         moveTo_follow_path_task_.contact_threshold_angular))
    {
        // #ifndef NDEBUG       
            printf("Contact occurred\n");
        // #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return task_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    bool final_section_reached = false;
    if ((unsigned)tube_section_count == moveTo_follow_path_task_.tf_poses.size() - 1) final_section_reached = true;
    
    // Check if the current pose of the robot satisfies all 6D tolerances
    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveTo_follow_path_task_.tube_tolerances[i]) count++;
    }

    // Check if the robot has reached end of the tube path
    if (count == NUM_OF_CONSTRAINTS_ && final_section_reached) 
    {   
        // #ifndef NDEBUG       
            printf("Whole path covered\n");
        // #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::STOP_ROBOT;
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
            return task_status::CHANGE_TUBE_SECTION;
        }
        return task_status::CRUISE_THROUGH_TUBE;        
    }
    
    else
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::START_TO_CRUISE;
    }
}

int finite_state_machine::update_moveTo_weight_compensation_task(state_specification &desired_state)
{
    if (total_control_time_sec_ > moveTo_weight_compensation_task_.time_limit) 
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;

        // #ifndef NDEBUG       
            if (!time_limit_reached_) printf("Time limit reached\n");
        // #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (goal_reached_ || contact_detected_) return task_status::STOP_ROBOT;
    
    if (contact_detected(moveTo_weight_compensation_task_.contact_threshold_linear, 
                         moveTo_weight_compensation_task_.contact_threshold_angular))
    {
        // #ifndef NDEBUG       
            printf("Contact occurred\n");
        // #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return task_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveTo_weight_compensation_task_.tube_tolerances[i]) count++;
    }
    
    if (count == NUM_OF_CONSTRAINTS_) 
    {
        // #ifndef NDEBUG       
            printf("Goal area reached\n");
        // #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::STOP_ROBOT;
    }
    goal_reached_ = false;

    /*
     * The robot has reached goal x area?
     * If yes, command zero X linear velocity to keep it in that area, until all DOFs gets back into tube.
     * Else go with initially commanded tube speed.
    */
    if ( (std::fabs(current_error_(0)) <= moveTo_weight_compensation_task_.tube_tolerances[0]) || \
         ((std::fabs(current_error_(0)) >  moveTo_weight_compensation_task_.tube_tolerances[0]) && \
          (count < NUM_OF_CONSTRAINTS_ - 1)) 
       )
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::START_TO_CRUISE;
    }
    else
    {
        double speed = 0.0;
        switch (motion_profile_)
        {
            case m_profile::STEP:
                speed = motion_profile::negative_step_function(std::fabs(current_error_(0)), 
                                                               moveTo_weight_compensation_task_.tube_speed, 
                                                               0.25, 0.4, 0.2);
                break;

            case m_profile::S_CURVE:
                speed = motion_profile::s_curve_function(std::fabs(current_error_(0)), 
                                                         0.05, moveTo_weight_compensation_task_.tube_speed, 5.0);
                break;

            default:
                speed = moveTo_weight_compensation_task_.tube_speed;
                break;
        }

        if (sign(current_error_(0)) == -1) desired_state.frame_velocity[END_EFF_].vel(0) = -1 * speed;      
        else desired_state.frame_velocity[END_EFF_].vel(0) = speed;              
    }
   
    return task_status::CRUISE_THROUGH_TUBE;
}


int finite_state_machine::update_moveTo_task(state_specification &desired_state)
{
    if (total_control_time_sec_ > moveTo_task_.time_limit) 
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;

        // #ifndef NDEBUG
            if (!time_limit_reached_) printf("Time limit reached\n");
        // #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (goal_reached_ || contact_detected_) return task_status::STOP_ROBOT;

    if (contact_detected(moveTo_task_.contact_threshold_linear, 
                         moveTo_task_.contact_threshold_angular))
    {
        // #ifndef NDEBUG
            printf("Contact occurred\n");
        // #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return task_status::STOP_ROBOT;
    }
    contact_detected_ = false;

    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= moveTo_task_.tube_tolerances[i]) count++;
    }
    
    if (count == NUM_OF_CONSTRAINTS_) 
    {
        // #ifndef NDEBUG       
            printf("Goal area reached\n");
        // #endif

        goal_reached_ = true;
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        return task_status::STOP_ROBOT;
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
        return task_status::START_TO_CRUISE;
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
   
    return task_status::CRUISE_THROUGH_TUBE;
}


int finite_state_machine::update_moveGuarded_task(state_specification &desired_state)
{
    if (total_control_time_sec_ > moveGuarded_task_.time_limit) 
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;

        // #ifndef NDEBUG       
            if (!time_limit_reached_) printf("Time limit reached\n");
        // #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (contact_detected_) return task_status::STOP_ROBOT;
    
    if (contact_detected(moveGuarded_task_.contact_threshold_linear, 
                         moveGuarded_task_.contact_threshold_angular))
    {
        // #ifndef NDEBUG       
            printf("Contact occurred\n");
        // #endif

        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        contact_detected_ = true;
        return task_status::STOP_ROBOT;
    }

    // Check if robot is inside the tube
    for (int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) > moveGuarded_task_.tube_tolerances[i])
        {
            desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
            return task_status::START_TO_CRUISE;
        }
    }
    
    // TODO: Add motion profile here
    desired_state.frame_velocity[END_EFF_].vel(0) = moveGuarded_task_.tube_speed;              
    return task_status::CRUISE_THROUGH_TUBE;
}

int finite_state_machine::update_full_pose_task(state_specification &desired_state)
{
    if (total_control_time_sec_ > full_pose_task_.time_limit) 
    {
        // #ifndef NDEBUG
            if (!time_limit_reached_) printf("Time limit reached\n");
        // #endif

        time_limit_reached_ = true;
        return task_status::STOP_CONTROL;
    }

    if (goal_reached_ || contact_detected_) return task_status::STOP_ROBOT;

    if (contact_detected(full_pose_task_.contact_threshold_linear, 
                         full_pose_task_.contact_threshold_angular))
    {
        // #ifndef NDEBUG       
            printf("Contact occurred\n");
        // #endif
        contact_detected_ = true;
        return task_status::STOP_ROBOT;
    }

    int count = 0;
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if (std::fabs(current_error_(i)) <= full_pose_task_.goal_area[i]) count++;
    }
    
    if (count == NUM_OF_CONSTRAINTS_) 
    {
        printf("Goal area reached\n");

        goal_reached_ = true;
        return task_status::STOP_ROBOT;
    }

    return task_status::NOMINAL;
}

int finite_state_machine::update_gravity_compensation_task()
{
    if (total_control_time_sec_ > gravity_compensation_task_.time_limit) 
    {
        if (!time_limit_reached_)
        {
            printf("Time limit reached\n");
            time_limit_reached_ = true;
        }

        return task_status::STOP_CONTROL;
    }

    return task_status::NOMINAL;
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
    total_control_time_sec_ = time_passed_sec;

    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            return update_moveConstrained_follow_path_task(desired_state, tube_section_count);
            break;

        case task_model::moveTo_follow_path:
            ext_wrench_ = ext_force;
            return update_moveTo_follow_path_task(desired_state, tube_section_count);
            break;

        case task_model::moveTo:
            ext_wrench_ = ext_force;
            return update_moveTo_task(desired_state);
            break;

        case task_model::moveGuarded:
            ext_wrench_ = ext_force;
            return update_moveGuarded_task(desired_state);
            break;

        case task_model::moveTo_weight_compensation:
            ext_wrench_ = ext_force;
            return update_moveTo_weight_compensation_task(desired_state);
            break;

        case task_model::full_pose:
            ext_wrench_ = ext_force;
            return update_full_pose_task(desired_state);
            break;

         case task_model::gravity_compensation:
            ext_wrench_ = ext_force;
            return update_gravity_compensation_task();
            break;

        default:
            printf("Unsupported task model\n");
            return task_status::STOP_CONTROL;
            break;
    }
}

int finite_state_machine::update_weight_compensation_task_status(const int loop_iteration_count,
                                                                 const Eigen::VectorXd &bias_signal,
                                                                 const Eigen::VectorXd &gain_signal,
                                                                 Eigen::VectorXd &filtered_bias)
{
    log_compensation_data(loop_iteration_count, bias_signal, gain_signal);
    low_pass_filter(bias_signal, 0.99);
    variance_bias_.update(bias_signal);
    variance_gain_.update(gain_signal);
    slope_bias_.update(filtered_bias_);

    // Check for X linear axis
    if (compensator_trigger_count_ < compensation_parameters_(9))
    {
        if ( (variance_bias_.get_variance(0)      <= compensation_parameters_(5)) && \
             (variance_gain_.get_variance(0)      <= compensation_parameters_(6)) && \
             (std::fabs(slope_bias_.get_slope(0)) <= compensation_parameters_(7)) )
        {
            loop_period_count_++;
        }
        else 
        {
            loop_period_count_ = 0;
            return 0;
        }
        
        double compensation_error = compensation_parameters_(0) - filtered_bias_(0);
        if (std::fabs(compensation_error) <= compensation_parameters_(4)) compensation_error = 0.0;

        if ((loop_period_count_ >= compensation_parameters_(8)) && (compensation_error != 0.0))
        {
            printf("Triger X %d: %f, %f, %f \n", compensator_trigger_count_ + 1, filtered_bias_(0), filtered_bias_(1), filtered_bias_(2));
            filtered_bias(0)   = filtered_bias_(0);
            loop_period_count_ = 0;
            write_compensation_time_to_file_ = true;
            compensator_trigger_count_++;
            return 1;
        }
        else if ((compensation_error == 0.0) && (loop_period_count_ > 2 * compensation_parameters_(8)))
        {
            printf("X Estimation completed\n\n");
            loop_period_count_ = 0;
            compensator_trigger_count_ = compensation_parameters_(9);
            return 0;
        }
    }

    // if (compensator_trigger_count_ >= compensation_parameters_(9)) return -1;

    // Check for Y linear axis
    else if (compensator_trigger_count_ >= compensation_parameters_(9) && \
             compensator_trigger_count_  < (compensation_parameters_(9) + compensation_parameters_(10)))
    {
        if ( (variance_bias_.get_variance(1)      <= compensation_parameters_(5)) && \
             (variance_gain_.get_variance(1)      <= compensation_parameters_(6)) && \
             (std::fabs(slope_bias_.get_slope(1)) <= compensation_parameters_(7)) )
        {
            loop_period_count_++;
        }
        else 
        {
            loop_period_count_ = 0;
            return 0;
        }
        
        double compensation_error = compensation_parameters_(1) - filtered_bias_(1);
        if (std::fabs(compensation_error) <= compensation_parameters_(4)) compensation_error = 0.0;

        if ((loop_period_count_ >= compensation_parameters_(8)) && (compensation_error != 0.0))
        {
            printf("Triger Y %d: %f, %f, %f \n", compensator_trigger_count_ + 1, filtered_bias_(0), filtered_bias_(1), filtered_bias_(2));
            filtered_bias(1)   = filtered_bias_(1);
            loop_period_count_ = 0;
            write_compensation_time_to_file_ = true;
            compensator_trigger_count_++;
            return 2;
        }
        else if ((compensation_error == 0.0) && (loop_period_count_ > 2 * compensation_parameters_(8)))
        {
            printf("Y Estimation completed\n\n");
            loop_period_count_ = 0;
            compensator_trigger_count_ = compensation_parameters_(9) + compensation_parameters_(10);
            return 0;
        }
    }

    // Check for Z linear axis
    else if (compensator_trigger_count_ >= (compensation_parameters_(9) + compensation_parameters_(10)) && \
             compensator_trigger_count_  < (compensation_parameters_(9) + compensation_parameters_(10) + compensation_parameters_(11)))
    {
        if ( (variance_bias_.get_variance(2)      <= compensation_parameters_(5)) && \
             (variance_gain_.get_variance(2)      <= compensation_parameters_(6)) && \
             (std::fabs(slope_bias_.get_slope(2)) <= compensation_parameters_(7)) )
        {
            loop_period_count_++;
        }
        else 
        {
            loop_period_count_ = 0;
            return 0;
        }
        
        double compensation_error = compensation_parameters_(2) - filtered_bias_(2);
        if (std::fabs(compensation_error) <= compensation_parameters_(4)) compensation_error = 0.0;

        if ((loop_period_count_ >= compensation_parameters_(8)) && (compensation_error != 0.0))
        {
            printf("Triger Z %d: %f, %f, %f \n", compensator_trigger_count_ + 1, filtered_bias_(0), filtered_bias_(1), filtered_bias_(2));
            filtered_bias(2)   = filtered_bias_(2);
            loop_period_count_ = 0;
            write_compensation_time_to_file_ = true;
            compensator_trigger_count_++;
            return 3;
        }
        else if ((compensation_error == 0.0) && (loop_period_count_ > 2 * compensation_parameters_(8)))
        {
            printf("Z Estimation completed\n\n");
            loop_period_count_ = 0;
            compensator_trigger_count_ = compensation_parameters_(9) + compensation_parameters_(10) + compensation_parameters_(11);
            return 0;
        }
    }

    return 0;
}

int finite_state_machine::update_force_task_status(const KDL::Wrench &desired_force,
                                                   const KDL::Wrench &ext_force,
                                                   const double current_task_time,
                                                   const double time_threshold)
{
    // First filter the measurements. Data is too noisy.
    low_pass_filter(ext_force, 0.70);

    // for (int i = 0; i < 6; i++) 
    //     log_file_ext_force_ << ext_wrench_(i) << " ";
    // log_file_ext_force_ << std::endl;
    
    // Cruise control mode
    if (contact_alignment_performed_)
    {
        if (!contact_detected(moveConstrained_follow_path_task_.contact_threshold_linear, 
                              moveConstrained_follow_path_task_.contact_threshold_angular))
        {
            total_contact_time_ += current_task_time - previous_task_time_;
        }
        else total_contact_time_ = 0.0;

        previous_task_time_ = current_task_time;

        if (total_contact_time_ >= time_threshold)
        {
            #ifndef NDEBUG       
                printf("Contact Lost!\n");
            #endif
            return task_status::STOP_ROBOT;
        } 
        return task_status::CRUISE;
    }

    // Approach control mode
    else
    {
        if (!contact_alignment_secured(desired_force, ext_wrench_)) total_contact_time_ = 0.0;
        else total_contact_time_ += current_task_time - previous_task_time_;

        previous_task_time_ = current_task_time;
        if (total_contact_time_ >= 0.010) 
        {
            total_contact_time_ = 0.0;
            contact_alignment_performed_ = true;
            return task_status::CRUISE;
        }
        return task_status::APPROACH;
    }
}

bool finite_state_machine::contact_alignment_secured(const KDL::Wrench &desired_force,
                                                     const KDL::Wrench &ext_force)
{
    if (std::fabs(ext_force(2)) < 0.016 || std::fabs(ext_force(2)) > 1.0) return false;
    for (int i = 3; i < 5; i++)
    {
        if (std::fabs(ext_force(i)) > 0.0029) return false;
    }

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

    return true;
}

bool finite_state_machine::contact_detected(const double linear_force_threshold, 
                                            const double angular_force_threshold)
{
    for (int i = 0; i < 3; i++)
    {
        if (std::fabs(ext_wrench_(i)) > linear_force_threshold) return true;
    }

    for (int i = 3; i < 6; i++)
    {
        if (std::fabs(ext_wrench_(i)) > angular_force_threshold) return true;
    }

    return false;    
}

void finite_state_machine::low_pass_filter(const KDL::Wrench &ext_force,
                                           const double alpha)
{
    for (int i = 0; i < 6; i++)
        ext_wrench_(i) = alpha * ext_wrench_(i) + (1 - alpha) * ext_force(i);
}

void finite_state_machine::low_pass_filter(const Eigen::VectorXd &signal,
                                           const double alpha)
{
    for (int i = 0; i < 6; i++)
        filtered_bias_(i) = alpha * filtered_bias_(i) + (1 - alpha) * signal(i);
}

int finite_state_machine::sign(double x)
{
    if (x > 0.0) return 1;
    else if (x < 0.0) return -1;
    else return 0;
}

void finite_state_machine::log_compensation_data(const int loop_iteration_count,
                                                 const Eigen::VectorXd &bias_signal,
                                                 const Eigen::VectorXd &gain_signal)
{
    log_file_compensation_ << bias_signal.transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_compensation_ << gain_signal.transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_compensation_ << filtered_bias_.transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_compensation_ << variance_bias_.get_variance().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_compensation_ << variance_gain_.get_variance().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_compensation_ << slope_bias_.get_slope().transpose().format(dynamics_parameter::WRITE_FORMAT);
    if (write_compensation_time_to_file_) 
    {
        log_file_compensation_ << loop_iteration_count - 1 << std::endl;
        write_compensation_time_to_file_ = false;
    }
    else log_file_compensation_ << 0.0 << std::endl;
}
