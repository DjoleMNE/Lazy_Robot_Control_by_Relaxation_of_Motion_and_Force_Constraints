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

#include <dynamics_controller.hpp>
#define SECOND 1000000 // 1sec = 1 000 000 us
const double MIN_NORM = 1e-10;

dynamics_controller::dynamics_controller(robot_mediator *robot_driver,
                                         const int rate_hz):
    RATE_HZ_(rate_hz),
    // Time period defined in microseconds: 1s = 1 000 000us
    DT_MICRO_(SECOND / RATE_HZ_),  DT_SEC_(1.0 / static_cast<double>(RATE_HZ_)),
    loop_start_time_(), loop_end_time_(), //Not sure if required to init
    total_time_sec_(0.0),  robot_chain_(robot_driver->get_robot_model()),
    NUM_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUM_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    END_EFF_(NUM_OF_SEGMENTS_ - 1),
    CTRL_DIM_(NUM_OF_CONSTRAINTS_, false), fsm_result_(control_status::NOMINAL),
    previous_control_status_(fsm_result_), tube_section_count_(0),
    JOINT_TORQUE_LIMITS_(robot_driver->get_joint_torque_limits()),
    current_error_twist_(KDL::Twist::Zero()),
    abag_error_vector_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    predicted_error_twist_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    horizon_amplitude_(1.0),
    abag_command_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    max_command_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    cart_force_command_(NUM_OF_SEGMENTS_), ext_wrench_(KDL::Wrench::Zero()),
    hd_solver_(robot_chain_, robot_driver->get_joint_inertia(), 
               robot_driver->get_joint_torque_limits(),
               robot_driver->get_root_acceleration(), NUM_OF_CONSTRAINTS_),
    fk_vereshchagin_(robot_chain_),
    safety_control_(robot_driver, true), 
    fsm_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    abag_(abag_parameter::DIMENSIONS, abag_parameter::USE_ERROR_SIGN),
    predictor_(robot_chain_),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    desired_state_(robot_state_),
    predicted_state_(robot_state_)
{
    assert(("Robot is not initialized", robot_driver->is_initialized()));
    // KDL Solver constraint  
    assert(NUM_OF_JOINTS_ == NUM_OF_SEGMENTS_);

    // Control loop frequency must be higher than or equal to 1 Hz
    assert(("Selected frequency is too low", 1 <= RATE_HZ_));
    // Control loop frequency must be lower than or equal to 1000 Hz
    assert(("Selected frequency is too high", RATE_HZ_<= 10000));
    
    // Set default command interface to stop motion mode and initialize it as not safe
    desired_control_mode_.interface = control_mode::STOP_MOTION;
    desired_control_mode_.is_safe = false;

    desired_task_inteface_ = dynamics_interface::CART_FORCE;
    desired_task_model_    = task_model::full_pose;

    KDL::SetToZero(cart_force_command_[END_EFF_]);

    // Setting parameters of the ABAG Controller
    abag_.set_error_alpha(abag_parameter::ERROR_ALPHA);    
    abag_.set_bias_threshold(abag_parameter::BIAS_THRESHOLD);
    abag_.set_bias_step(abag_parameter::BIAS_STEP);
    abag_.set_gain_threshold(abag_parameter::GAIN_THRESHOLD);
    abag_.set_gain_step(abag_parameter::GAIN_STEP);
}

//Print information about controller settings
void dynamics_controller::print_settings_info()
{   
    #ifdef NDEBUG
        printf("The program is build in RELEASE mode.\n");
    #endif
    #ifndef NDEBUG
        printf("The program is build in DEBUG mode.\n");
    #endif
    
    printf("Selected controller settings:\n");
    printf("Control Loop Frequency: %d Hz\n", RATE_HZ_);

    printf("Joint Interface: ");
    switch(desired_control_mode_.interface) 
    {
        case control_mode::STOP_MOTION:
            printf("STOP JOINTS \n Stopping the robot!\n");
            break;

        case control_mode::POSITION:
            printf("Joint Position \n");
            break;

        case control_mode::VELOCITY:
            printf("Joint Velocity \n");
            break;

        case control_mode::TORQUE:
            printf("Joint Torque \n");
            break;
    }

    printf("Dynamics Interface: ");
    switch(desired_task_inteface_) 
    {
        case dynamics_interface::CART_ACCELERATION:
            printf("Cartesian EndEffector Acceleration Interface\n");
            break;

        case dynamics_interface::CART_FORCE:
            printf("Cartesian Force Interface\n");
            break;

        case dynamics_interface::FF_JOINT_TORQUE:
            printf("FeedForward Joint Torque Interface\n");
            break;

        default:
            printf("Stopping the robot!\n");
            break;
    }

    printf("Task Model: ");
    switch(desired_task_model_) 
    {
        case task_model::full_pose:
            printf("Control Full 6D Pose\n");
            break;

        case task_model::moveGuarded:
            printf("moveGuarded\n");
            break;

        case task_model::moveTo:
            printf("moveTo\n");
            break;

        case task_model::moveTo_follow_path:
            printf("moveTo_follow_path\n");
            break;

        case task_model::moveConstrained_follow_path:
            printf("moveConstrained_follow_path\n");
            break;

        default:
            printf("Stopping the robot!\n");
            break;
    }

    std::cout<< "\nInitial joint state: "<< std::endl;
    std::cout<< "Joint positions: "<< robot_state_.q << std::endl;
    std::cout<< "Joint velocities:"<< robot_state_.qd << "\n" << std::endl;

    std::cout<< "Initial Cartesian state: "<< std::endl;
    std::cout<< "End-effector position: "<< robot_state_.frame_pose[END_EFF_].p << std::endl;
    std::cout<< "End-effector orientation: \n"<< robot_state_.frame_pose[END_EFF_].M << std::endl;
    std::cout<< "End-effector velocity:"<< robot_state_.frame_velocity[END_EFF_] << "\n" << std::endl;
}


int dynamics_controller::check_control_status()
{
    switch (fsm_result_)
    {
        case control_status::NOMINAL:
            if(previous_control_status_ != control_status::NOMINAL) printf("Control status changed to NOMINAL\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;
        
        case control_status::START_TO_CRUISE:
            if(previous_control_status_ != control_status::START_TO_CRUISE) printf("Control status changed to START_TO_CRUISE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CRUISE_TO_STOP:
            if(previous_control_status_ != control_status::CRUISE_TO_STOP) printf("Control status changed to CRUISE_TO_STOP\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CRUISE_THROUGH_TUBE:
            if(previous_control_status_ != control_status::CRUISE_THROUGH_TUBE) printf("Control status changed to CRUISE_THROUGH_TUBE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;
        
        case control_status::CRUISE:
            if(previous_control_status_ != control_status::CRUISE) printf("Control status changed to CRUISE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CHANGE_TUBE_SECTION:
            if(previous_control_status_ != control_status::CHANGE_TUBE_SECTION) printf("Control status changed to CHANGE_TUBE_SECTION\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::APPROACH:
            if(previous_control_status_ != control_status::APPROACH) printf("Control status changed to APPROACH\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::STOP_ROBOT:
            if(previous_control_status_ != control_status::STOP_ROBOT) printf("Control status changed to STOP_ROBOT\n");
            // stop_robot_motion();
            previous_control_status_ = fsm_result_;
            return 1;
            break;
        
        default:
            printf("Stop control!\n");
            return -1;
            break;
    }
}

/* 
    If it is working on the real robot get sensor data from the driver 
    or if simulation is on, replace current state with 
    integrated joint velocities and positions.
*/
void dynamics_controller::update_current_state()
{
    // Get joint angles and velocities
    safety_control_.get_current_state(robot_state_);

    // Get Cart poses and velocities
    int fk_solver_result = fk_vereshchagin_.JntToCart(robot_state_.q, 
                                                      robot_state_.qd, 
                                                      robot_state_.frame_pose, 
                                                      robot_state_.frame_velocity);
    if(fk_solver_result != 0) 
        printf("Warning: FK solver returned an error! %d \n", fk_solver_result);

    // Print Current robot state in Debug mode
#ifndef NDEBUG
        // std::cout << "\nCurrent Joint state:          " << std::endl;
        // std::cout << "Joint angle:    " << robot_state_.q << std::endl;
        // std::cout << "Joint velocity: " << robot_state_.qd << std::endl;
        
        // std::cout << "\nCurrent Cartesian state:                 " << std::endl;
        // std::cout << "End-effector Position:   " 
        //           << robot_state_.frame_pose[END_EFF_].p  << std::endl;
        // std::cout << "End-effector Velocity:                \n" 
        //           << robot_state_.frame_velocity[END_EFF_] << std::endl;
#endif 

#ifdef NDEBUG
        // std::cout << "End-effector Velocity:   \n" 
        //           << robot_state_.frame_velocity[END_EFF_] << std::endl;
#endif
}

// Update current dynamics intefaces using desired robot state specifications 
void dynamics_controller::update_dynamics_interfaces()
{ 
    robot_state_.ee_unit_constraint_force = desired_state_.ee_unit_constraint_force;
    robot_state_.ee_acceleration_energy   = desired_state_.ee_acceleration_energy;
    robot_state_.feedforward_torque       = desired_state_.feedforward_torque;
    robot_state_.external_force           = desired_state_.external_force;
}

// Write control data to a file
void dynamics_controller::write_to_file()
{   
    for(int i = 0; i < 3; i++) 
        log_file_cart_ << robot_state_.frame_pose[END_EFF_].p(i) << " ";
    for(int i = 3; i < 6; i++) 
        log_file_cart_ << 0.0 << " ";
    log_file_cart_ << robot_state_.frame_velocity[END_EFF_](0) << " "; 
    log_file_cart_ << robot_state_.frame_velocity[END_EFF_](5) << " ";
    for (int i = 2; i < 5; i++)
        log_file_cart_ << ext_wrench_(i) << " ";
    log_file_cart_ << std::endl;

    for(int i = 0; i < 3; i++) 
        log_file_cart_ << desired_state_.frame_pose[END_EFF_].p(i) << " ";
    for(int i = 3; i < 6; i++) 
        log_file_cart_ << 0.0 << " ";        
    log_file_cart_ << desired_state_.frame_velocity[END_EFF_](0) << " ";
    log_file_cart_ << desired_state_.frame_velocity[END_EFF_](5) << " ";
    for (int i = 2; i < 5; i++)
        log_file_cart_ << desired_state_.external_force[END_EFF_](i) << " ";
    log_file_cart_ << std::endl;

    log_file_cart_ << predicted_error_twist_(0) << " ";
    for(int i = 1; i < 6; i++) 
        log_file_cart_ << abag_error_vector_(i) << " ";
    log_file_cart_ << abag_error_vector_(0) << std::endl;

    log_file_cart_ << abag_.get_error().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_bias().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_gain().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_command().transpose().format(dynamics_parameter::WRITE_FORMAT);

    log_file_joint_ << robot_state_.control_torque.data.transpose().format(dynamics_parameter::WRITE_FORMAT);
}

// Set all values of desired state to 0 - public method
void dynamics_controller::reset_desired_state()
{
    reset_state(desired_state_);
}

// Set all values of selected state to 0 - Private method
void dynamics_controller::reset_state(state_specification &state)
{
    desired_state_.reset_values();
}

//Send 0 joints velocities to the robot driver
void dynamics_controller::stop_robot_motion()
{   
    safety_control_.stop_robot_motion();
}

void dynamics_controller::define_moveConstrained_follow_path_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector< std::vector<double> > &tube_path_points,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double tube_force,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                std::vector< std::vector<double> > &task_frame_poses)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_poses.size()     == tube_path_points.size() - 1);
    assert(tube_path_points[0].size()  == 3);
    assert(task_frame_poses[0].size()  == 12);

    moveConstrained_follow_path_task_.tf_poses   = std::vector<KDL::Frame>(tube_path_points.size() - 1);
    moveConstrained_follow_path_task_.goal_poses = moveConstrained_follow_path_task_.tf_poses;

    CTRL_DIM_ = constraint_direction;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    std::vector<double> task_frame_pose(12, 0.0);

    for (int i = 0; i < tube_path_points.size() - 1; i++)
    {
        KDL::Vector tube_start_position = KDL::Vector(tube_path_points[i    ][0], tube_path_points[i    ][1], tube_path_points[i    ][2]);
        KDL::Vector tf_position         = KDL::Vector(tube_path_points[i + 1][0], tube_path_points[i + 1][1], tube_path_points[i + 1][2]);
        KDL::Vector x_task              = tf_position - tube_start_position;
        x_task.Normalize();

        KDL::Vector cross_product = x_world * x_task;
        double cosine             = dot(x_world, x_task);
        double sine               = cross_product.Norm();
        double angle              = atan2(sine, cosine);

        task_frame_pose[0] = tf_position[0]; 
        task_frame_pose[1] = tf_position[1]; 
        task_frame_pose[2] = tf_position[2];

        KDL::Rotation tf_orientation;
        if(cosine < (-1 + 1e-6))
        {
            tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
            task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
        }
        else if(sine < 1e-6)
        {
            tf_orientation = KDL::Rotation::Identity();
            task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
        }
        else
        {
            tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
            task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
            task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
            task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
        }

        task_frame_poses[i]                                 = task_frame_pose;
        moveConstrained_follow_path_task_.tf_poses[i]       = KDL::Frame(tf_orientation, tf_position);
        moveConstrained_follow_path_task_.goal_poses[i]     = KDL::Frame::Identity();
    }

    moveConstrained_follow_path_task_.tube_path_points          = tube_path_points;
    moveConstrained_follow_path_task_.tube_tolerances           = tube_tolerances;
    moveConstrained_follow_path_task_.tube_speed                = tube_speed;
    moveConstrained_follow_path_task_.tube_force                = tube_force;
    moveConstrained_follow_path_task_.contact_threshold_linear  = contact_threshold_linear;
    moveConstrained_follow_path_task_.contact_threshold_angular = contact_threshold_angular;
    moveConstrained_follow_path_task_.time_limit                = task_time_limit_sec;

    desired_state_.frame_pose[END_EFF_]    = moveConstrained_follow_path_task_.goal_poses[0];
    desired_task_model_                    = task_model::moveConstrained_follow_path;
}


void dynamics_controller::define_moveTo_follow_path_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector< std::vector<double> > &tube_path_points,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                std::vector< std::vector<double> > &task_frame_poses)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_poses.size()     == tube_path_points.size() - 1);
    assert(tube_path_points[0].size()  == 3);
    assert(task_frame_poses[0].size()  == 12);

    moveTo_follow_path_task_.tf_poses   = std::vector<KDL::Frame>(tube_path_points.size() - 1);
    moveTo_follow_path_task_.goal_poses = moveTo_follow_path_task_.tf_poses;

    CTRL_DIM_ = constraint_direction;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    std::vector<double> task_frame_pose(12, 0.0);

    for (int i = 0; i < tube_path_points.size() - 1; i++)
    {
        KDL::Vector tube_start_position = KDL::Vector(tube_path_points[i][0], tube_path_points[i][1], tube_path_points[i][2]);
        KDL::Vector tf_position         = KDL::Vector(tube_path_points[i + 1][0], tube_path_points[i + 1][1], tube_path_points[i + 1][2]);
        KDL::Vector x_task              = tf_position - tube_start_position;
        x_task.Normalize();

        KDL::Vector cross_product = x_world * x_task;
        double cosine             = dot(x_world, x_task);
        double sine               = cross_product.Norm();
        double angle              = atan2(sine, cosine);

        task_frame_pose[0] = tf_position[0]; 
        task_frame_pose[1] = tf_position[1]; 
        task_frame_pose[2] = tf_position[2];

        KDL::Rotation tf_orientation;
        if(cosine < (-1 + 1e-6))
        {
            tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
            task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
        }
        else if(sine < 1e-6)
        {
            tf_orientation = KDL::Rotation::Identity();
            task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
        }
        else
        {
            tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
            task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
            task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
            task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
        }

        task_frame_poses[i]                        = task_frame_pose;
        moveTo_follow_path_task_.tf_poses[i]       = KDL::Frame(tf_orientation, tf_position);
        moveTo_follow_path_task_.goal_poses[i]     = KDL::Frame::Identity();
    }

    moveTo_follow_path_task_.tube_path_points          = tube_path_points;
    moveTo_follow_path_task_.tube_tolerances           = tube_tolerances;
    moveTo_follow_path_task_.tube_speed                = tube_speed;
    moveTo_follow_path_task_.contact_threshold_linear  = contact_threshold_linear;
    moveTo_follow_path_task_.contact_threshold_angular = contact_threshold_angular;
    moveTo_follow_path_task_.time_limit                = task_time_limit_sec;

    desired_state_.frame_pose[END_EFF_]    = moveTo_follow_path_task_.goal_poses[0];
    desired_task_model_                    = task_model::moveTo_follow_path;
}


void dynamics_controller::define_moveTo_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &tube_start_position,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                std::vector<double> &task_frame_pose)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_pose.size()      == 12);
    assert(tube_start_position.size()  == 3);

    CTRL_DIM_ = constraint_direction;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    KDL::Vector tf_position = KDL::Vector(task_frame_pose[0], task_frame_pose[1], task_frame_pose[2]);
    KDL::Vector x_task = tf_position - KDL::Vector(tube_start_position[0], tube_start_position[1], tube_start_position[2]);
    x_task.Normalize();

    KDL::Vector cross_product = x_world * x_task;
    double cosine             = dot(x_world, x_task);
    double sine               = cross_product.Norm();
    double angle              = atan2(sine, cosine);

    KDL::Rotation tf_orientation;
    if(cosine < (-1 + 1e-6))
    {
        tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
        task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
        task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
        task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
    }
    else if(sine < 1e-6)
    {
        tf_orientation = KDL::Rotation::Identity();
        task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
        task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
        task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
    }
    else
    {
        tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
        task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
        task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
        task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
    }

    moveTo_task_.tf_pose                   = KDL::Frame(tf_orientation, tf_position);
    moveTo_task_.goal_pose                 = KDL::Frame::Identity();
    moveTo_task_.tube_start_position       = tube_start_position;
    moveTo_task_.tube_tolerances           = tube_tolerances;
    moveTo_task_.tube_speed                = tube_speed;
    moveTo_task_.contact_threshold_linear  = contact_threshold_linear;
    moveTo_task_.contact_threshold_angular = contact_threshold_angular;
    moveTo_task_.time_limit                = task_time_limit_sec;

    desired_state_.frame_pose[END_EFF_]    = moveTo_task_.goal_pose;
    desired_task_model_                    = task_model::moveTo;
}

void dynamics_controller::define_desired_ee_pose(
                            const std::vector<bool> &constraint_direction,
                            const std::vector<double> &cartesian_pose,
                            const double contact_threshold_linear,
                            const double contact_threshold_angular,
                            const double task_time_limit_sec)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(cartesian_pose.size()       == NUM_OF_CONSTRAINTS_ * 2);
    
    CTRL_DIM_ = constraint_direction;
    
    desired_state_.frame_pose[END_EFF_].p(0) = cartesian_pose[0];
    desired_state_.frame_pose[END_EFF_].p(1) = cartesian_pose[1];
    desired_state_.frame_pose[END_EFF_].p(2) = cartesian_pose[2];

    desired_state_.frame_pose[END_EFF_].M = \
        KDL::Rotation(cartesian_pose[3], cartesian_pose[4], cartesian_pose[5],
                      cartesian_pose[6], cartesian_pose[7], cartesian_pose[8],
                      cartesian_pose[9], cartesian_pose[10], cartesian_pose[11]);
    
    full_pose_task_.tf_pose                   = KDL::Frame::Identity();
    full_pose_task_.goal_pose                 = desired_state_.frame_pose[END_EFF_];
    full_pose_task_.goal_area                 = std::vector<double>(6, 0.01);
    full_pose_task_.contact_threshold_linear  = contact_threshold_linear;
    full_pose_task_.contact_threshold_angular = contact_threshold_angular;
    full_pose_task_.time_limit                = task_time_limit_sec;
    desired_task_model_                       = task_model::full_pose;
}

// Define Cartesian Acceleration task on the end-effector - Public Method
void dynamics_controller::define_ee_acc_constraint(
                            const std::vector<bool> &constraint_direction,
                            const std::vector<double> &cartesian_acceleration)
{    
    //Call private method for this state
    set_ee_acc_constraints(desired_state_, 
                           constraint_direction, 
                           cartesian_acceleration);
}

// Define Cartesian Acceleration task on the end-effector - Private Method
void dynamics_controller::set_ee_acc_constraints(
                                state_specification &state,
                                const std::vector<bool> &constraint_direction, 
                                const std::vector<double> &cartesian_acceleration)
{    
    assert(constraint_direction.size()   == NUM_OF_CONSTRAINTS_);
    assert(cartesian_acceleration.size() == NUM_OF_CONSTRAINTS_);

    // Set directions in which constraint force should work. Alpha in the solver 
    KDL::Twist unit_force_x_l(
        KDL::Vector((constraint_direction[0] ? 1.0 : 0.0), 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(0, unit_force_x_l);

    KDL::Twist unit_force_y_l(
            KDL::Vector(0.0, (constraint_direction[1] ? 1.0 : 0.0), 0.0),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(1, unit_force_y_l);

    KDL::Twist unit_force_z_l(
            KDL::Vector(0.0, 0.0, (constraint_direction[2] ? 1.0 : 0.0)),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(2, unit_force_z_l);

    KDL::Twist unit_force_x_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector((constraint_direction[3] ? 1.0 : 0.0), 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(3, unit_force_x_a);

    KDL::Twist unit_force_y_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, (constraint_direction[4] ? 1.0 : 0.0), 0.0));
    state.ee_unit_constraint_force.setColumn(4, unit_force_y_a);

    KDL::Twist unit_force_z_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, 0.0, (constraint_direction[5] ? 1.0 : 0.0)));
    state.ee_unit_constraint_force.setColumn(5, unit_force_z_a);

    // Set desired acceleration on the end-effector. Beta in the solver
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        state.ee_acceleration_energy(i) = cartesian_acceleration[i];
}

// Define External force task - Public Method
void dynamics_controller::define_ee_external_force(const std::vector<double> &external_force)
{
    //Call private method for this state
    set_external_forces(desired_state_, external_force);
}

// Define External force task - Private Method
void dynamics_controller::set_external_forces(state_specification &state, 
                                              const std::vector<double> &external_force)
{
    //For now it is only updating forces on the end-effector
    //TODO: add forces on other segments as well
    assert(external_force.size() == NUM_OF_CONSTRAINTS_);

    state.external_force[END_EFF_] = KDL::Wrench (KDL::Vector(external_force[0],
                                                              external_force[1],
                                                              external_force[2]),
                                                  KDL::Vector(external_force[3],
                                                              external_force[4],
                                                              external_force[5]));
}

// Define FeedForward joint torques task - Public Method
void dynamics_controller::define_feedforward_torque(const std::vector<double> &ff_torque)
{
    //Call private method for this state
    set_feedforward_torque(desired_state_, ff_torque);
}

// Define FeedForward joint torques task - Private Method
void dynamics_controller::set_feedforward_torque(state_specification &state, 
                                                 const std::vector<double> &ff_torque)
{
    assert(ff_torque.size() == NUM_OF_JOINTS_);

    for (int i = 0; i < NUM_OF_JOINTS_; i++) state.feedforward_torque(i) = ff_torque[i];
}

//Make sure that the control loop runs exactly with specified frequency
int dynamics_controller::enforce_loop_frequency()
{
    loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);

    if(loop_interval_ < std::chrono::microseconds(DT_MICRO_))
    {   
        //Loop is sufficiently fast
        // clock_nanosleep((DT_MICRO_ - loop_interval_.count()));
        while(loop_interval_.count() < DT_MICRO_){
            loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);
        }
        return 0;
    } else return -1; //Loop is too slow
}

/*
    Apply joint commands using safe control interface.
    If the computed commands are not safe, exit the program.
*/
int dynamics_controller::apply_joint_control_commands()
{ 
    /* 
        Safety controller checks if the commands are over the limits.
        If false: use desired control mode
        Else: stop the robot motion 
    */
    int safe_control_mode = safety_control_.set_control_commands(robot_state_, 
                                                                 DT_SEC_, 
                                                                 desired_control_mode_.interface,
                                                                 integration_method::SYMPLECTIC_EULER);
   
    // Check if the safety controller has changed the control mode
    // Save the current decision if desired control mode is safe or not.
    desired_control_mode_.is_safe =\
        (desired_control_mode_.interface == safe_control_mode)? true : false; 

    // Notify if the safety controller has changed the control mode
    switch(safe_control_mode) {
        case control_mode::TORQUE:
            assert(desired_control_mode_.is_safe);
            return 0;

        case control_mode::VELOCITY:
            if(!desired_control_mode_.is_safe) 
                printf("WARNING: Control switched to velocity mode \n");
            return 0;

        case control_mode::POSITION:
            if(!desired_control_mode_.is_safe) 
                printf("WARNING: Control switched to position mode \n");
            return 0;

        default: 
            stop_robot_motion();
            // printf("WARNING: Computed commands are not safe. Stopping the robot!\n");
            return -1;
    }
}

/*  
    Predict future robot Cartesian states given the current Cartesian state.
    I.e. Integrate Cartesian variables.
*/
void dynamics_controller::make_predictions(const double dt_sec, const int num_steps)
{
    predictor_.integrate_cartesian_space(robot_state_, 
                                         predicted_state_, 
                                         dt_sec, num_steps);
}


KDL::Twist dynamics_controller::finite_displacement_twist(const state_specification &state_a, 
                                                          const state_specification &state_b)
{
    /**
     * Difference between two poses: Decoupled calculation.
     * See "Modern Robotics" Book, 2017, sections 9.2.1 and 11.3.3.
    */

    // The default constructor initialises to Zero via the constructor of Vector!
    KDL::Twist twist;

    /**
     * This error part represents a linear motion necessary to go from 
     * predicted to desired position (positive direction of translation).
    */
    twist.vel = state_a.frame_pose[END_EFF_].p - state_b.frame_pose[END_EFF_].p;

    /**
     * Describes rotation required to align R_p with R_d.
     * It represents relative rotation from predicted state to 
     * desired state, expressed in the BASE frame!
     * Source: Luh et al. "Resolved-acceleration control of 
     * mechanical manipulators".
    */
    KDL::Rotation relative_rot_matrix = state_a.frame_pose[END_EFF_].M * \
                                        state_b.frame_pose[END_EFF_].M.Inverse();

    // Error calculation for angular part, i.e. logarithmic map on SO(3).
    twist.rot = geometry::log_map_so3(relative_rot_matrix);

    return twist;
}

double dynamics_controller::kinetic_energy(const KDL::Twist &twist,
                                           const int segment_index)
{
    return 0.5 * dot(twist, robot_chain_.getSegment(segment_index).getInertia() * twist);
}


/**
 * Compute the control error for tube deviations and velocity setpoint.
 * Error function for moveTo_follow_path task.
*/
void dynamics_controller::compute_moveTo_follow_path_task_error()
{
    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]     = moveTo_follow_path_task_.tf_poses[tube_section_count_].Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_] = moveTo_follow_path_task_.tf_poses[tube_section_count_].M.Inverse() * robot_state_.frame_velocity[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update(robot_state_, desired_state_, current_error_twist_, 
                              ext_wrench_, total_time_sec_, tube_section_count_);
    
    abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_].vel(0) - robot_state_.frame_velocity[END_EFF_].vel(0);
    
    // Check for tube on velocity
    if ((desired_state_.frame_velocity[END_EFF_].vel(0) != 0.0) && \
        (std::fabs(abag_error_vector_(0)) <= moveTo_follow_path_task_.tube_tolerances[6]))
    {
        abag_error_vector_(0) = 0.0;
    }

    // Other parts of the ABAG error are position errors
    for(int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if ( std::fabs(predicted_error_twist_(i)) <= moveTo_follow_path_task_.tube_tolerances[i] ) abag_error_vector_(i) = 0.0;
        else abag_error_vector_(i) = predicted_error_twist_(i);        
    }
}


/**
 * Compute the control error for tube deviations and velocity setpoint.
 * Error function for moveTo task.
*/
void dynamics_controller::compute_moveTo_task_error()
{
    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]     = moveTo_task_.tf_pose.Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_] = moveTo_task_.tf_pose.M.Inverse() * robot_state_.frame_velocity[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update(robot_state_, desired_state_, current_error_twist_, 
                              ext_wrench_, total_time_sec_, tube_section_count_);

    abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_].vel(0) - robot_state_.frame_velocity[END_EFF_].vel(0);

    // Check for tube on velocity
    if ((desired_state_.frame_velocity[END_EFF_].vel(0) != 0.0) && \
        (std::fabs(abag_error_vector_(0)) <= moveTo_task_.tube_tolerances[6]))
    {
        abag_error_vector_(0) = 0.0;
    }

    // Other parts of the ABAG error are position errors
    for (int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if ( std::fabs(predicted_error_twist_(i)) <= moveTo_task_.tube_tolerances[i] ) abag_error_vector_(i) = 0.0;
        else abag_error_vector_(i) = predicted_error_twist_(i);        
    }
}

/**
 * Compute the error between desired Cartesian state and predicted (integrated) Cartesian state.
 * Error function for full_pose task
*/
void dynamics_controller::compute_full_pose_task_error()
{
    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );
    
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_            = fsm_.update(robot_state_, desired_state_, current_error_twist_, 
                                         ext_wrench_, total_time_sec_, tube_section_count_);
    abag_error_vector_     = predicted_error_twist_;
}

/**
 * Compute the general control error. 
 * Internally an error function will be called for each selected task
*/
void dynamics_controller::compute_control_error()
{    
    switch (desired_task_model_)
    {
        case task_model::moveTo_follow_path:
            compute_moveTo_follow_path_task_error();
            break;

        case task_model::moveTo:
            compute_moveTo_task_error();
            break;

        case task_model::full_pose:
            compute_full_pose_task_error(); 
            break;

        default:
            assert(("Unsupported task model", false));
            break;
    }
}

//Change the reference frame of external (virtual) forces, from task frame to base frame
void dynamics_controller::transform_force_driver()
{
    if (desired_task_model_ == task_model::moveTo_follow_path)
    {
        cart_force_command_[END_EFF_] = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * cart_force_command_[END_EFF_];
    }
    else cart_force_command_[END_EFF_] = moveTo_task_.tf_pose.M * cart_force_command_[END_EFF_];
}

//Change the reference frame of constraint forces, from task frame to base frame
void dynamics_controller::transform_motion_driver()
{
    KDL::Wrench wrench_column;
    KDL::Twist twist_column;
    KDL::Jacobian alpha = robot_state_.ee_unit_constraint_force;

    //Change the reference frame of constraint forces, from task frame to base frame
    for (int c = 0; c < NUM_OF_CONSTRAINTS_; c++)
    {
        // Change Data Type of constraint forces to fit General KDL type
        wrench_column = KDL::Wrench(KDL::Vector(alpha(0, c), alpha(1, c), alpha(2, c)),
                                    KDL::Vector(alpha(3, c), alpha(4, c), alpha(5, c)));
        
        if (desired_task_model_ == task_model::moveTo_follow_path)
        {
            wrench_column = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * wrench_column;
        }
        else wrench_column = moveTo_task_.tf_pose.M * wrench_column;

        // Change Data Type to fit Vereshchagin
        twist_column = KDL::Twist(wrench_column.force, wrench_column.torque);
        robot_state_.ee_unit_constraint_force.setColumn(c, twist_column);
    }
}

void dynamics_controller::compute_cart_control_commands()
{   
    abag_command_ = abag_.update_state(abag_error_vector_).transpose();

    switch (desired_task_inteface_)
    {
        case dynamics_interface::CART_FORCE:
            // Set virtual forces computed by the ABAG controller
            for(int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
                cart_force_command_[END_EFF_](i) = CTRL_DIM_[i]? abag_command_(i) * max_command_(i) : 0.0;
            
            if(desired_task_model_ == task_model::moveTo || \
               desired_task_model_ == task_model::moveTo_follow_path)
            {
                transform_force_driver();

                if(fsm_result_ == control_status::CHANGE_TUBE_SECTION) tube_section_count_++;
                if(tube_section_count_ > moveTo_follow_path_task_.tf_poses.size() - 1)
                {
                    tube_section_count_ = moveTo_follow_path_task_.tf_poses.size() - 1;
                }
            } 

            break;

        case dynamics_interface::CART_ACCELERATION:
            // Set Cartesian Acceleration Constraints on the End-Effector
            set_ee_acc_constraints(robot_state_,
                                   std::vector<bool>{CTRL_DIM_[0], CTRL_DIM_[1], CTRL_DIM_[2], // Linear
                                                     CTRL_DIM_[3], CTRL_DIM_[4], CTRL_DIM_[5]}, // Angular
                                   std::vector<double>{abag_command_(0) * max_command_(0), // Linear
                                                       abag_command_(1) * max_command_(1), // Linear
                                                       abag_command_(2) * max_command_(2), // Linear
                                                       abag_command_(3) * max_command_(3), // Angular
                                                       abag_command_(4) * max_command_(4), // Angular
                                                       abag_command_(5) * max_command_(5)}); // Angular

            if(desired_task_model_ == task_model::moveTo || \
               desired_task_model_ == task_model::moveTo_follow_path)
            {
                //Change the reference frame of constraint forces, from task frame to base frame
                transform_motion_driver();

                // Change the reference frame of External Forces from the task frame to the base frame
                if(use_mixed_driver_)
                {
                    // Use external force interace for only X linear direction, to control tube speed
                    KDL::SetToZero(cart_force_command_[END_EFF_]);
                    cart_force_command_[END_EFF_](0) = abag_command_(0) * max_command_(0);
                    transform_force_driver();
                } 
                
                if(fsm_result_ == control_status::CHANGE_TUBE_SECTION) tube_section_count_++;
                if(tube_section_count_ > moveTo_follow_path_task_.tf_poses.size() - 1)
                {
                    tube_section_count_ = moveTo_follow_path_task_.tf_poses.size() - 1;
                }
            }

            break;

        default:
            assert(("Unsupported interface!", false));
            break;
    }

#ifndef NDEBUG
    // std::cout << "ABAG Commands:         "<< abag_command_.transpose() << std::endl;
    // std::cout << "Virtual Force Command: " << cart_force_command_[END_EFF_] << std::endl;
    // printf("\n");
#endif
}

//Calculate robot dynamics - Resolve motion and forces using the Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    int hd_solver_result = hd_solver_.CartToJnt(robot_state_.q,
                                                robot_state_.qd,
                                                robot_state_.qdd,
                                                robot_state_.ee_unit_constraint_force,
                                                robot_state_.ee_acceleration_energy,
                                                robot_state_.external_force,
                                                cart_force_command_,
                                                robot_state_.feedforward_torque);

    if(hd_solver_result != 0) return hd_solver_result;

    hd_solver_.get_control_torque(robot_state_.control_torque);

    // hd_solver_.get_transformed_link_acceleration(robot_state_.frame_acceleration);
    
    // Print computed state in Debug mode
#ifndef NDEBUG
        // std::cout << "\nComputed Cartesian state:" << std::endl;

        // std::cout << "Frame ACC" << '\n';
        // for (size_t i = 0; i < NUM_OF_SEGMENTS_ + 1; i++)
        //     std::cout << robot_state_.frame_acceleration[i] << '\n';

        // std::cout << "End-effector Position:   " 
        //       << robot_state_.frame_pose[END_EFF_].p  << std::endl;

        // std::cout << "\nComputed Joint state:          " << std::endl;
        // std::cout << "Joint torque:  " << robot_state_.control_torque << std::endl;
        // std::cout << "Joint acc:     " << robot_state_.qdd << std::endl;
#endif 

#ifdef NDEBUG
        // std::cout << "Joint torque:  " << robot_state_.control_torque << std::endl;
#endif

    return hd_solver_result;
}

void dynamics_controller::set_parameters(const double horizon_amplitude,
                                         const int abag_error_type,
                                         const Eigen::VectorXd &max_command,
                                         const Eigen::VectorXd &error_alpha, 
                                         const Eigen::VectorXd &bias_threshold, 
                                         const Eigen::VectorXd &bias_step, 
                                         const Eigen::VectorXd &gain_threshold, 
                                         const Eigen::VectorXd &gain_step,
                                         const Eigen::VectorXd &min_bias_sat,
                                         const Eigen::VectorXd &min_command_sat)
{
    //First check input dimensions
    assert(max_command.size()    == NUM_OF_CONSTRAINTS_); 
    assert(error_alpha.size()    == NUM_OF_CONSTRAINTS_); 
    assert(bias_threshold.size() == NUM_OF_CONSTRAINTS_); 
    assert(bias_step.size()      == NUM_OF_CONSTRAINTS_); 
    assert(gain_threshold.size() == NUM_OF_CONSTRAINTS_); 
    assert(gain_step.size()      == NUM_OF_CONSTRAINTS_); 

    this->horizon_amplitude_  = horizon_amplitude;
    this->max_command_        = max_command;
    
    // Setting parameters of the ABAG Controller
    abag_.set_error_alpha(error_alpha);    
    abag_.set_bias_threshold(bias_threshold);
    abag_.set_bias_step(bias_step);
    abag_.set_gain_threshold(gain_threshold);
    abag_.set_gain_step(gain_step);
    abag_.set_min_bias_sat_limit(min_bias_sat);
    abag_.set_min_command_sat_limit(min_command_sat);
    abag_.set_error_type(abag_error_type);
}

int dynamics_controller::initialize(const int desired_control_mode, 
                                    const int desired_task_inteface,
                                    const bool use_mixed_driver,
                                    const bool store_control_data,
                                    const int motion_profile)
{
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;

    //Exit the program if the "Stop Motion" mode is selected
    assert(desired_control_mode_.interface != control_mode::STOP_MOTION); 

    desired_task_inteface_ = desired_task_inteface;

    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            fsm_.initialize_with_moveConstrained_follow_path(moveConstrained_follow_path_task_, 
                                                             motion_profile);
            break;

        case task_model::moveTo_follow_path:
            fsm_.initialize_with_moveTo_follow_path(moveTo_follow_path_task_, 
                                                    motion_profile);
            break;

        case task_model::moveTo:
            fsm_.initialize_with_moveTo(moveTo_task_, motion_profile);
            break;
        
        case task_model::full_pose:
            fsm_.initialize_with_full_pose(full_pose_task_, motion_profile);
            break;
            
        default:
            printf("Unsupported task model\n");
            return -1;
            break;
    }

    // First make sure that the robot is not moving
    // stop_robot_motion();

    // Update current constraints, external forces, and feedforward torques
    update_dynamics_interfaces();
    store_control_data_ = store_control_data;

    if (store_control_data_) 
    {
        log_file_cart_.open(dynamics_parameter::LOG_FILE_CART_PATH);
        assert(log_file_cart_.is_open());

        for(int i = 0; i < NUM_OF_CONSTRAINTS_ + 1; i++)
        {
            if (desired_task_model_ == task_model::moveTo_follow_path)
            {
                log_file_cart_ << moveTo_follow_path_task_.tube_tolerances[i] << " ";
            }
            else log_file_cart_ << moveTo_task_.tube_tolerances[i] << " ";
        }
        log_file_cart_ << std::endl;      

        log_file_joint_.open(dynamics_parameter::LOG_FILE_JOINT_PATH);
        assert(log_file_joint_.is_open());

        log_file_predictions_.open(dynamics_parameter::LOG_FILE_PREDICTIONS_PATH);
        assert(log_file_predictions_.is_open());

        log_file_transformed_.open(dynamics_parameter::LOG_FILE_TRANSFORMED_PATH);
        assert(log_file_transformed_.is_open());

        for(int i = 0; i < NUM_OF_JOINTS_; i++) 
            log_file_joint_ << JOINT_TORQUE_LIMITS_[i] << " ";
        log_file_joint_ << std::endl;
    }

    print_settings_info();
    return 0;
}

//Main control loop
int dynamics_controller::control(const int desired_control_mode,
                                 const bool store_control_data)
{   
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;
    
    //Exit the program if the "Stop Motion" mode is selected
    if(desired_control_mode_.interface == control_mode::STOP_MOTION){
        std::cout << "Stop Motion mode selected. Exiting the program" << std::endl;
        return -1;
    } 
    
    // First make sure that the robot is not moving
    stop_robot_motion();

    /* 
        Get sensor data from the robot driver or if simulation is on, 
        replace current state with the integrated joint velocities and positions.
        Additionally, update dynamics intefaces.
    */
    update_current_state();  update_dynamics_interfaces();

    //Print information about controller settings
    print_settings_info();

    if (store_control_data) 
    {
        log_file_cart_.open(dynamics_parameter::LOG_FILE_CART_PATH);
        if (!log_file_cart_.is_open()) {
            printf("Unable to open the file!\n");
            return -1;
        }

        log_file_joint_.open(dynamics_parameter::LOG_FILE_JOINT_PATH);
        if (!log_file_joint_.is_open()) {
            printf("Unable to open the file!\n");
            return -1;
        }

        log_file_predictions_.open(dynamics_parameter::LOG_FILE_PREDICTIONS_PATH);
        if (!log_file_predictions_.is_open()) {
            printf("Unable to open the file!\n");
            return -1;
        }

        log_file_transformed_.open(dynamics_parameter::LOG_FILE_TRANSFORMED_PATH);
        if (!log_file_transformed_.is_open()) {
            printf("Unable to open the file!\n");
            return -1;
        }

        for(int i = 0; i < NUM_OF_JOINTS_; i++) 
            log_file_joint_ << JOINT_TORQUE_LIMITS_[i] << " ";
        log_file_joint_ << std::endl;
    }

    double loop_time = 0.0;
    int loop_count = 0;

    std::cout << "Control Loop Started"<< std::endl;
    while(1)
    {   
        loop_count++; 
        // printf("Loop Count: %d \n", loop_count);

        // Save current time point
        loop_start_time_ = std::chrono::steady_clock::now();

        //Get current robot state from the joint sensors: velocities and angles
        update_current_state();

        compute_control_error();
        
        compute_cart_control_commands();
        if (store_control_data) write_to_file();        

        // Calculate robot dynamics using the Vereshchagin HD solver
        if(evaluate_dynamics() != 0)
        {
            stop_robot_motion();
            if (store_control_data) 
            {
                log_file_cart_.close();
                log_file_joint_.close();
                log_file_predictions_.close();
                log_file_transformed_.close();
            }
            printf("WARNING: Dynamics Solver returned error. Stopping the robot!");
            return -1;
        }
        // if(loop_count == 1) return 0;

        // Apply joint commands using safe control interface.
        if(apply_joint_control_commands() != 0){
            if (store_control_data) 
            {
                log_file_cart_.close();
                log_file_joint_.close();
                log_file_predictions_.close();
                log_file_transformed_.close();
            }
            return -1;
        } 

        // Make sure that the loop is always running with the same frequency
        if(!enforce_loop_frequency() == 0)
            printf("WARNING: Control loop runs too slow \n");

        // loop_time += std::chrono::duration<double, std::micro>\
        //             (std::chrono::steady_clock::now() -\
        //                                          loop_start_time_).count();
        // if(loop_count == 40) {
        //     std::cout << loop_time / 40.0 <<std::endl;
        //     return 0;
        // }
    }

    if (store_control_data) 
    {
        log_file_cart_.close();
        log_file_joint_.close();
        log_file_predictions_.close();
        log_file_transformed_.close();
    }
    return 0;
}


/**
 * Perform single step of the control loop, given current robot joint state
 * Required for RTT's updateHook method
*/
int dynamics_controller::step(const KDL::JntArray &q_input,
                              const KDL::JntArray &qd_input,
                              const KDL::Wrench &ext_force,
                              Eigen::VectorXd &tau_output,
                              const double time_passed_sec)
{
    robot_state_.q  = q_input;
    robot_state_.qd = qd_input;
    ext_wrench_     = ext_force;
    total_time_sec_ = time_passed_sec;

    // Get Cart poses and velocities
    int fk_solver_result = fk_vereshchagin_.JntToCart(robot_state_.q, 
                                                      robot_state_.qd, 
                                                      robot_state_.frame_pose, 
                                                      robot_state_.frame_velocity);
    if(fk_solver_result != 0)
    {
        deinitialize();
        printf("Warning: FK solver returned an error! %d \n", fk_solver_result);
        return -1;
    }

    // Print Current robot state in Debug mode
    #ifndef NDEBUG
        // std::cout << "\nCurrent Joint state:          " << std::endl;
        // std::cout << "Joint angle:    " << robot_state_.q << std::endl;
        // std::cout << "Joint velocity: " << robot_state_.qd << std::endl;
        
        // std::cout << "\nCurrent Cartesian state:                 " << std::endl;
        // std::cout << "End-effector Position:   " 
        //           << robot_state_.frame_pose[END_EFF_].p  << std::endl;
        // std::cout << "End-effector Orientation:   \n" 
        //           << robot_state_.frame_pose[END_EFF_].M  << std::endl;
        // std::cout << "End-effector Velocity:                \n" 
        //           << robot_state_.frame_velocity[END_EFF_] << std::endl;
    #endif 

    // static int steps = 0;

    // if (steps % 1 == 0)
    // {
    //     compute_control_error();
    //     compute_cart_control_commands();
    // }

    compute_control_error();

    if (check_control_status() == -1) return -1;

    compute_cart_control_commands();

    // steps++;

    if (store_control_data_) write_to_file();   

    // Calculate robot dynamics using the Vereshchagin HD solver
    if(evaluate_dynamics() != 0)
    {
        deinitialize();
        printf("WARNING: Dynamics Solver returned error. Stopping the robot!\n");
        return -1;
    }

    // apply_joint_control_commands();
    // Apply joint commands using safe control interface.
    // if(apply_joint_control_commands() != 0)
    // {
    //     deinitialize();
    //     return -1;
    // } 
    tau_output = robot_state_.control_torque.data;

    return 0;
}

void dynamics_controller::deinitialize()
{
    // First make sure that the robot is not moving
    stop_robot_motion();
    if (store_control_data_) 
    {
        log_file_cart_.close();
        log_file_joint_.close();
        log_file_predictions_.close();
        log_file_transformed_.close();
    }
}