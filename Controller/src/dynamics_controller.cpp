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
const double MIN_NORM = 1e-3;

dynamics_controller::dynamics_controller(robot_mediator *robot_driver,
                                         const int rate_hz,
                                         const bool compensate_gravity):
    RATE_HZ_(rate_hz),
    // Time period defined in microseconds: 1s = 1 000 000us
    DT_MICRO_(SECOND / RATE_HZ_),  DT_SEC_(1.0 / static_cast<double>(RATE_HZ_)),
    store_control_data_(true), desired_dynamics_interface_(dynamics_interface::CART_ACCELERATION), 
    desired_task_model_(task_model::full_pose),
    loop_start_time_(), loop_end_time_(), //Not sure if required to init
    total_time_sec_(0.0), loop_iteration_count_(0), feedforward_loop_count_(0),
    robot_chain_(robot_driver->get_robot_model()),
    NUM_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUM_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    END_EFF_(NUM_OF_SEGMENTS_ - 1), ROBOT_ID_(robot_driver->get_robot_ID()),
    CTRL_DIM_(NUM_OF_CONSTRAINTS_, false), POS_TUBE_DIM_(NUM_OF_CONSTRAINTS_, false),
    MOTION_CTRL_DIM_(NUM_OF_CONSTRAINTS_, false), FORCE_CTRL_DIM_(NUM_OF_CONSTRAINTS_, false),
    fsm_result_(control_status::NOMINAL), fsm_force_task_result_(control_status::APPROACH),
    previous_control_status_(fsm_result_), tube_section_count_(0), 
    transform_drivers_(false), transform_force_drivers_(false),
    apply_feedforward_force_(false), compute_null_space_command_(false),
    write_contact_time_to_file_(false), compensate_unknown_weight_(false),
    compensate_gravity_(compensate_gravity),
    JOINT_TORQUE_LIMITS_(robot_driver->get_joint_torque_limits()),
    current_error_twist_(KDL::Twist::Zero()),
    abag_error_vector_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    null_space_abag_error_(Eigen::VectorXd::Zero(1)),
    predicted_error_twist_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    compensation_error_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    horizon_amplitude_(1.0), null_space_abag_command_(0.0), null_space_angle_(0.0),
    abag_command_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    max_command_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    compensation_parameters_(Eigen::VectorXd::Constant(7, 0.0)),
    force_task_parameters_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    min_sat_limits_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    filtered_bias_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    cart_force_command_(NUM_OF_SEGMENTS_, KDL::Wrench::Zero()), 
    zero_wrenches_(NUM_OF_SEGMENTS_, KDL::Wrench::Zero()),
    ext_wrench_(KDL::Wrench::Zero()), ext_wrench_base_(KDL::Wrench::Zero()),
    compensated_weight_(KDL::Wrench::Zero()),
    zero_joint_array_(NUM_OF_JOINTS_), gravity_torque_(NUM_OF_JOINTS_),
    hd_solver_(robot_chain_, robot_driver->get_joint_inertia(), 
               robot_driver->get_joint_torque_limits(), !compensate_gravity_,
               compensate_gravity_? KDL::Twist::Zero() : robot_driver->get_root_acceleration(), 
               NUM_OF_CONSTRAINTS_),
    id_solver_(robot_chain_, KDL::Vector(0.0, 0.0, -9.81289), robot_driver->get_joint_inertia(), 
               robot_driver->get_joint_torque_limits(), false),
    fk_vereshchagin_(robot_chain_),
    safety_control_(robot_driver, true), 
    fsm_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    abag_(abag_parameter::DIMENSIONS, abag_parameter::USE_ERROR_SIGN),
    abag_null_space_(1, abag_parameter::USE_ERROR_SIGN),
    predictor_(robot_chain_),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    robot_state_base_(robot_state_), desired_state_(robot_state_),
    desired_state_base_(robot_state_), predicted_state_(robot_state_)
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

    // Setting parameters of the ABAG Controller
    abag_.set_error_alpha(abag_parameter::ERROR_ALPHA);    
    abag_.set_bias_threshold(abag_parameter::BIAS_THRESHOLD);
    abag_.set_bias_step(abag_parameter::BIAS_STEP);
    abag_.set_gain_threshold(abag_parameter::GAIN_THRESHOLD);
    abag_.set_gain_step(abag_parameter::GAIN_STEP);

    abag_null_space_.set_error_alpha(abag_parameter::NULL_SPACE_ERROR_ALPHA, 0);    
    abag_null_space_.set_bias_threshold(abag_parameter::NULL_SPACE_BIAS_THRESHOLD, 0);
    abag_null_space_.set_bias_step(abag_parameter::NULL_SPACE_BIAS_STEP, 0);
    abag_null_space_.set_gain_threshold(abag_parameter::NULL_SPACE_GAIN_THRESHOLD, 0);
    abag_null_space_.set_gain_step(abag_parameter::NULL_SPACE_GAIN_STEP, 0);
    abag_null_space_.set_min_bias_sat_limit((Eigen::VectorXd(1) << -1.0).finished());
    abag_null_space_.set_min_command_sat_limit((Eigen::VectorXd(1) << -1.0).finished());
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
    switch(desired_dynamics_interface_) 
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

        case task_model::moveTo_weight_compensation:
            printf("moveTo_weight_compensation\n");
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


int dynamics_controller::check_fsm_status()
{
    switch (fsm_result_)
    {
        case control_status::NOMINAL:
            if (previous_control_status_ != control_status::NOMINAL) printf("Control status changed to NOMINAL\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;
        
        case control_status::START_TO_CRUISE:
            if (previous_control_status_ != control_status::START_TO_CRUISE) printf("Control status changed to START_TO_CRUISE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CRUISE_TO_STOP:
            if (previous_control_status_ != control_status::CRUISE_TO_STOP) printf("Control status changed to CRUISE_TO_STOP\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CRUISE_THROUGH_TUBE:
            if (previous_control_status_ != control_status::CRUISE_THROUGH_TUBE) printf("Control status changed to CRUISE_THROUGH_TUBE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;
        
        case control_status::CRUISE:
            if (previous_control_status_ != control_status::CRUISE) printf("Control status changed to CRUISE\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::CHANGE_TUBE_SECTION:
            if (previous_control_status_ != control_status::CHANGE_TUBE_SECTION) printf("Control status changed to CHANGE_TUBE_SECTION\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::APPROACH:
            if (previous_control_status_ != control_status::APPROACH) printf("Control status changed to APPROACH\n");
            previous_control_status_ = fsm_result_;
            return 0;
            break;

        case control_status::STOP_ROBOT:
            if (previous_control_status_ != control_status::STOP_ROBOT) printf("Control status changed to STOP_ROBOT\n");
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
int dynamics_controller::update_current_state()
{
    // Get joint angles and velocities
    safety_control_.get_current_state(robot_state_);

    // Get Cart poses and velocities
    int fk_solver_result = fk_vereshchagin_.JntToCart(robot_state_.q, 
                                                      robot_state_.qd, 
                                                      robot_state_.frame_pose, 
                                                      robot_state_.frame_velocity);
    // Print Current robot state in Debug mode
// #ifndef NDEBUG
        // std::cout << "\nCurrent Joint state:          " << std::endl;
        // std::cout << "Joint angle:    " << robot_state_.q << std::endl;
        // std::cout << "Joint velocity: " << robot_state_.qd << std::endl;
        
        // std::cout << "\nCurrent Cartesian state:                 " << std::endl;
        // std::cout << "End-effector Position:   " 
        //           << robot_state_.frame_pose[END_EFF_].p  << std::endl;
        // std::cout << "End-effector Velocity:                \n" 
        //           << robot_state_.frame_velocity[END_EFF_] << std::endl;
// #endif 

// #ifdef NDEBUG
        // std::cout << "End-effector Velocity:   \n" 
        //           << robot_state_.frame_velocity[END_EFF_] << std::endl;
// #endif
    return fk_solver_result;
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
    // Write measured state
    for (int i = 0; i < 3; i++) 
        log_file_cart_ << robot_state_.frame_pose[END_EFF_].p(i) << " ";
    for (int i = 3; i < 6; i++) 
        log_file_cart_ << 0.0 << " ";
    log_file_cart_ << robot_state_.frame_velocity[END_EFF_](0) << " "; 
    log_file_cart_ << robot_state_.frame_velocity[END_EFF_](5) << " ";
    for (int i = 2; i < 5; i++)
        log_file_cart_ << ext_wrench_(i) << " ";
    if (write_contact_time_to_file_) log_file_cart_ << loop_iteration_count_;
    else log_file_cart_ << 0.0;
    log_file_cart_ << std::endl;

    // Write desired state
    for (int i = 0; i < 3; i++) 
        log_file_cart_ << desired_state_.frame_pose[END_EFF_].p(i) << " ";
    for (int i = 3; i < 6; i++) 
        log_file_cart_ << 0.0 << " ";
    log_file_cart_ << desired_state_.frame_velocity[END_EFF_](0) << " ";
    log_file_cart_ << desired_state_.frame_velocity[END_EFF_](5) << " ";
    for (int i = 2; i < 5; i++)
        log_file_cart_ << desired_state_.external_force[END_EFF_](i) << " ";
    log_file_cart_ << std::endl;

    // Write control error
    log_file_cart_ << predicted_error_twist_(0) << " ";
    for (int i = 1; i < 6; i++) 
        log_file_cart_ << abag_error_vector_(i) << " ";
    log_file_cart_ << abag_error_vector_(0) << std::endl;

    // Write ABAG state
    log_file_cart_ << abag_.get_error().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_bias().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_gain().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_cart_ << abag_.get_command().transpose().format(dynamics_parameter::WRITE_FORMAT);

    // Log data in the base frame
    if (desired_task_model_ != task_model::full_pose)
    {
        // Write measured state in base frame
        for (int i = 0; i < 3; i++) 
            log_file_cart_base_ << robot_state_base_.frame_pose[END_EFF_].p(i) << " ";

        for (int i = 0; i < 6; i++)
            log_file_cart_base_ << ext_wrench_base_(i) << " ";

        if (write_contact_time_to_file_) log_file_cart_base_ << loop_iteration_count_ << " ";
        else log_file_cart_base_ << 0.0 << " ";

        if (compensate_unknown_weight_)
        {
            for (int i = 0; i < 3; i++)
                log_file_cart_base_ << robot_state_.external_force[END_EFF_].force(i) << " ";
        }
        log_file_cart_base_ << std::endl;

        // Write desired state in base frame
        for (int i = 0; i < 3; i++) 
            log_file_cart_base_ << desired_state_base_.frame_pose[END_EFF_].p(i) << " ";

        for (int i = 0; i < 6; i++)
            log_file_cart_base_ << desired_state_base_.external_force[END_EFF_](i) << " ";

        if (write_contact_time_to_file_) log_file_cart_base_ << loop_iteration_count_ << " ";
        else log_file_cart_base_ << 0.0 << " ";

        if (compensate_unknown_weight_)
        {
            for (int i = 0; i < 3; i++)
                log_file_cart_base_ << 0.0 << " ";
        }
        log_file_cart_base_ << std::endl;
    }

    // Write joint space state
    log_file_joint_ << robot_state_.control_torque.data.transpose().format(dynamics_parameter::WRITE_FORMAT);

    // Write null-space control state
    log_file_null_space_ << RAD_TO_DEG(null_space_angle_) << " "; // Measured state
    if (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_null_space_ << 0.0 << " ";  // desired state for LWR 4
    else log_file_null_space_ << 94.0 << " ";  // desired state for YouBot
    log_file_null_space_ << RAD_TO_DEG(null_space_abag_error_(0)) << " " << abag_null_space_.get_error()(0) << " "; // Raw and filtered error
    log_file_null_space_ << abag_null_space_.get_bias()(0)    << " " << abag_null_space_.get_gain()(0) << " ";
    log_file_null_space_ << abag_null_space_.get_command()(0) << " ";
    if (write_contact_time_to_file_) 
    {
        log_file_null_space_ << loop_iteration_count_;
        write_contact_time_to_file_ = false;
    }
    else log_file_null_space_ << 0.0;
    log_file_null_space_ << std::endl;
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

    CTRL_DIM_           = constraint_direction;
    POS_TUBE_DIM_[0]    = CTRL_DIM_[0]; POS_TUBE_DIM_[1]    = CTRL_DIM_[1]; 
    MOTION_CTRL_DIM_[0] = CTRL_DIM_[0]; MOTION_CTRL_DIM_[1] = CTRL_DIM_[1]; MOTION_CTRL_DIM_[5] = CTRL_DIM_[5];
    FORCE_CTRL_DIM_[3]  = CTRL_DIM_[3]; FORCE_CTRL_DIM_[4]  = CTRL_DIM_[4];

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    std::vector<double> task_frame_pose(12, 0.0);
    KDL::Rotation task_orientation = KDL::Rotation::EulerZYZ(0.0, 0.0, -M_PI/4);
    for (int i = 0; i < tube_path_points.size() - 1; i++)
    {
        KDL::Vector tube_start_position = KDL::Vector(tube_path_points[i    ][0], tube_path_points[i    ][1], tube_path_points[i    ][2]);
        KDL::Vector tf_position         = KDL::Vector(tube_path_points[i + 1][0], tube_path_points[i + 1][1], tube_path_points[i + 1][2]);

        tube_start_position = task_orientation * tube_start_position;
        tf_position         = task_orientation * tf_position;

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

        task_frame_poses[i]                             = task_frame_pose;
        moveConstrained_follow_path_task_.tf_poses[i]   = KDL::Frame(tf_orientation, tf_position);
        moveConstrained_follow_path_task_.goal_poses[i] = KDL::Frame::Identity();
    }

    moveConstrained_follow_path_task_.tube_path_points             = tube_path_points;
    moveConstrained_follow_path_task_.tube_tolerances              = tube_tolerances;
    moveConstrained_follow_path_task_.tube_speed                   = tube_speed;
    moveConstrained_follow_path_task_.tube_force                   = tube_force;
    moveConstrained_follow_path_task_.contact_threshold_linear     = contact_threshold_linear;
    moveConstrained_follow_path_task_.contact_threshold_angular    = contact_threshold_angular;
    moveConstrained_follow_path_task_.time_limit                   = task_time_limit_sec;
    moveConstrained_follow_path_task_.tf_force                     = KDL::Rotation::Identity();
    moveConstrained_follow_path_task_.null_space_plane_orientation = KDL::Rotation::Identity();
    moveConstrained_follow_path_task_.null_space_force_direction   = KDL::Vector::Zero();

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveConstrained_follow_path_task_.null_space_tolerance = moveConstrained_follow_path_task_.tube_tolerances[5];

    desired_state_.frame_pose[END_EFF_]        = moveConstrained_follow_path_task_.goal_poses[0];
    desired_task_model_                        = task_model::moveConstrained_follow_path;
    desired_state_.frame_velocity[END_EFF_](0) = tube_speed;
    desired_state_.frame_velocity[END_EFF_](5) = 0.0;
    desired_state_.external_force[END_EFF_](2) = tube_force;
    desired_state_.external_force[END_EFF_](3) = 0.0;
    desired_state_.external_force[END_EFF_](4) = 0.0;
    compute_null_space_command_ = true;
    transform_force_drivers_    = true;
    compensate_unknown_weight_  = false;
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
    MOTION_CTRL_DIM_ = CTRL_DIM_;

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
    
    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveTo_follow_path_task_.null_space_force_direction = KDL::Vector::Zero();
    moveTo_follow_path_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveTo_follow_path_task_.goal_poses[0];
    desired_task_model_                    = task_model::moveTo_follow_path;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    if (ROBOT_ID_ == robot_id::YOUBOT) compute_null_space_command_ = true;
    else                               compute_null_space_command_ = false;
    compensate_unknown_weight_  = false;
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
    MOTION_CTRL_DIM_ = CTRL_DIM_;

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

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveTo_task_.null_space_force_direction = KDL::Vector::Zero();
    moveTo_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveTo_task_.goal_pose;
    desired_task_model_                    = task_model::moveTo;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    if (ROBOT_ID_ == robot_id::YOUBOT) compute_null_space_command_ = true;
    else                               compute_null_space_command_ = false;
    compensate_unknown_weight_  = false;
}

void dynamics_controller::define_moveTo_weight_compensation_task(
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
    MOTION_CTRL_DIM_ = CTRL_DIM_;

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

    moveTo_weight_compensation_task_.tf_pose                   = KDL::Frame(tf_orientation, tf_position);
    moveTo_weight_compensation_task_.goal_pose                 = KDL::Frame::Identity();
    moveTo_weight_compensation_task_.tube_start_position       = tube_start_position;
    moveTo_weight_compensation_task_.tube_tolerances           = tube_tolerances;
    moveTo_weight_compensation_task_.tube_speed                = tube_speed;
    moveTo_weight_compensation_task_.contact_threshold_linear  = contact_threshold_linear;
    moveTo_weight_compensation_task_.contact_threshold_angular = contact_threshold_angular;
    moveTo_weight_compensation_task_.time_limit                = task_time_limit_sec;

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveTo_weight_compensation_task_.null_space_force_direction = KDL::Vector::Zero();
    moveTo_weight_compensation_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveTo_weight_compensation_task_.goal_pose;
    desired_task_model_                    = task_model::moveTo_weight_compensation;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    compute_null_space_command_ = false;
    compensate_unknown_weight_  = true;
}

void dynamics_controller::define_moveGuarded_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &tube_start_position,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                std::vector<double> &tube_end_position)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(tube_end_position.size()    == 12);
    assert(tube_start_position.size()  == 3);

    CTRL_DIM_ = constraint_direction;
    MOTION_CTRL_DIM_ = CTRL_DIM_;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    KDL::Vector tf_position = KDL::Vector(tube_end_position[0], tube_end_position[1], tube_end_position[2]);
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
        tube_end_position[3] = -1.0; tube_end_position[4]  =  0.0; tube_end_position[5]  = 0.0;
        tube_end_position[6] =  0.0; tube_end_position[7]  = -1.0; tube_end_position[8]  = 0.0;
        tube_end_position[9] =  0.0; tube_end_position[10] =  0.0; tube_end_position[11] = 1.0;
    }
    else if(sine < 1e-6)
    {
        tf_orientation = KDL::Rotation::Identity();
        tube_end_position[3] = 1.0; tube_end_position[4]  = 0.0; tube_end_position[5]  = 0.0;
        tube_end_position[6] = 0.0; tube_end_position[7]  = 1.0; tube_end_position[8]  = 0.0;
        tube_end_position[9] = 0.0; tube_end_position[10] = 0.0; tube_end_position[11] = 1.0;
    }
    else
    {
        tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
        tube_end_position[3] = tf_orientation.data[0]; tube_end_position[4]  = tf_orientation.data[1]; tube_end_position[5]  = tf_orientation.data[2];
        tube_end_position[6] = tf_orientation.data[3]; tube_end_position[7]  = tf_orientation.data[4]; tube_end_position[8]  = tf_orientation.data[5];
        tube_end_position[9] = tf_orientation.data[6]; tube_end_position[10] = tf_orientation.data[7]; tube_end_position[11] = tf_orientation.data[8];
    }

    moveGuarded_task_.tf_pose                   = KDL::Frame(tf_orientation, tf_position);
    moveGuarded_task_.goal_pose                 = KDL::Frame::Identity();
    moveGuarded_task_.tube_start_position       = tube_start_position;
    moveGuarded_task_.tube_end_position         = tube_end_position;
    moveGuarded_task_.tube_tolerances           = tube_tolerances;
    moveGuarded_task_.tube_speed                = tube_speed;
    moveGuarded_task_.contact_threshold_linear  = contact_threshold_linear;
    moveGuarded_task_.contact_threshold_angular = contact_threshold_angular;
    moveGuarded_task_.time_limit                = task_time_limit_sec;

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveGuarded_task_.null_space_force_direction = KDL::Vector::Zero();
    moveGuarded_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveGuarded_task_.goal_pose;
    desired_task_model_                    = task_model::moveGuarded;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    if (ROBOT_ID_ == robot_id::YOUBOT) compute_null_space_command_ = true;
    else                               compute_null_space_command_ = false;
    compensate_unknown_weight_  = false;
}


void dynamics_controller::define_desired_ee_pose(
                            const std::vector<bool> &constraint_direction,
                            const std::vector<double> &cartesian_pose,
                            const double contact_threshold_linear,
                            const double contact_threshold_angular,
                            const double task_time_limit_sec,
                            const double null_space_tolerance)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(cartesian_pose.size()       == NUM_OF_CONSTRAINTS_ * 2);
    
    CTRL_DIM_ = constraint_direction;
    MOTION_CTRL_DIM_ = CTRL_DIM_;

    desired_state_.frame_pose[END_EFF_].p(0) = cartesian_pose[0];
    desired_state_.frame_pose[END_EFF_].p(1) = cartesian_pose[1];
    desired_state_.frame_pose[END_EFF_].p(2) = cartesian_pose[2];

    desired_state_.frame_pose[END_EFF_].M = KDL::Rotation(cartesian_pose[3], cartesian_pose[4], cartesian_pose[5],
                                                          cartesian_pose[6], cartesian_pose[7], cartesian_pose[8],
                                                          cartesian_pose[9], cartesian_pose[10], cartesian_pose[11]);
    
    full_pose_task_.tf_pose                   = KDL::Frame::Identity();
    full_pose_task_.goal_pose                 = desired_state_.frame_pose[END_EFF_];
    full_pose_task_.goal_area                 = std::vector<double>(6, 0.01);
    full_pose_task_.contact_threshold_linear  = contact_threshold_linear;
    full_pose_task_.contact_threshold_angular = contact_threshold_angular;
    full_pose_task_.time_limit                = task_time_limit_sec;
  
    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    full_pose_task_.null_space_force_direction = KDL::Vector::Zero();
    full_pose_task_.null_space_tolerance       = null_space_tolerance;

    desired_task_model_                       = task_model::full_pose;
    transform_drivers_          = false;
    transform_force_drivers_    = false;
    if (ROBOT_ID_ == robot_id::YOUBOT) compute_null_space_command_ = true;
    else                               compute_null_space_command_ = false;
    compensate_unknown_weight_  = false;
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
    loop_interval_= std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_);

    if (loop_interval_ < std::chrono::microseconds(DT_MICRO_)) // Loop is sufficiently fast
    {
        while (loop_interval_.count() < (DT_MICRO_ - 1.0))
        {
            loop_interval_= std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_);
        }

        return 0;
    } 
    else return -1; //Loop is too slow
}

/*
    Apply joint commands using safe control interface.
    If the computed commands are not safe, exit the program.
*/
int dynamics_controller::apply_joint_control_commands()
{ 
    /* 
        Safety controller checks if the commands are over the limits.
        If yes: stop the robot motion 
        Else: use desired control mode
    */
    int safe_control_mode = safety_control_.set_control_commands(robot_state_, 
                                                                 DT_SEC_, 
                                                                 desired_control_mode_.interface,
                                                                 integration_method::SYMPLECTIC_EULER);

    // Check if the safety controller has changed the control mode.
    // Save the current decision: if desired control mode is safe or not.
    desired_control_mode_.is_safe = (desired_control_mode_.interface == safe_control_mode)? true : false; 

    // Notify if the safety controller has changed the control mode
    switch (safe_control_mode) 
    {
        case control_mode::TORQUE:
            if (!desired_control_mode_.is_safe) 
            {
                desired_control_mode_.interface = control_mode::STOP_MOTION;
                return -1;
            }
            return 0;

        case control_mode::VELOCITY:
            if (!desired_control_mode_.is_safe) printf("WARNING: Control switched to velocity mode \n");
            return 0;

        case control_mode::POSITION:
            if (!desired_control_mode_.is_safe) printf("WARNING: Control switched to position mode \n");
            return 0;

        default:
            desired_control_mode_.interface = control_mode::STOP_MOTION;
            return -1;
    }
}

/*  
    Predict future robot Cartesian states given the current Cartesian state.
    I.e. Integrate Cartesian variables.
*/
void dynamics_controller::make_predictions(const double dt_sec, const int num_steps)
{
    predictor_.integrate_cartesian_space(robot_state_, predicted_state_, dt_sec, num_steps);
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
     * predicted/measured to desired position (positive direction of translation).
    */
    twist.vel = state_a.frame_pose[END_EFF_].p - state_b.frame_pose[END_EFF_].p;

    /**
     * Describes rotation required to align R_m/p with R_d.
     * It represents relative rotation from measured/predicted state to 
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

void dynamics_controller::compute_moveConstrained_null_space_task_error()
{
    // X-axis of the null-space plane
    KDL::Vector plane_x(robot_state_.frame_pose[END_EFF_].p(0), robot_state_.frame_pose[END_EFF_].p(1), 0.0);
    double norm = plane_x.Normalize();
    
    if (norm < MIN_NORM)
    {
        null_space_abag_error_(0) = 0.0;
        moveConstrained_follow_path_task_.null_space_force_direction = KDL::Vector::Zero();
        printf("Null space norm too small 1 \n");
    }
    else 
    {
        // Calculate error: angle from the plane
        KDL::Vector plane_y(0.0, 0.0, 1.0);
        moveConstrained_follow_path_task_.null_space_plane_orientation = KDL::Rotation(plane_x, plane_y, plane_x * plane_y);
        KDL::Vector r_direction = moveConstrained_follow_path_task_.null_space_plane_orientation.Inverse() * robot_state_.frame_pose[3].p;
        r_direction.Normalize();
        null_space_angle_ = std::atan2(r_direction(2), r_direction(1)); // Angle between Y and R_yz

        // Unit of tolerance is degree
        if (std::fabs(null_space_angle_) <= DEG_TO_RAD(moveConstrained_follow_path_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
        else null_space_abag_error_(0) = null_space_angle_;

        // Calculate control/force direction: Cart force for null-space motion
        // plane_x = moveConstrained_follow_path_task_.null_space_plane_orientation.Inverse() * robot_state_.frame_pose[END_EFF_].p;
        // plane_x.Normalize();
        // KDL::Vector control_direction = r_direction * plane_x;

        KDL::Vector control_direction = r_direction * KDL::Vector(1.0, 0.0, 0.0);

        control_direction(0) = 0.0;
        norm = control_direction.Normalize();
        if (norm < MIN_NORM) 
        {
            null_space_abag_error_(0) = 0.0;
            moveConstrained_follow_path_task_.null_space_force_direction = KDL::Vector::Zero();
            // printf("Null space norm too small 2 \n");
        }
        else moveConstrained_follow_path_task_.null_space_force_direction = control_direction; // Transform from plane frame to base frame
    }
}

/**
 * Compute the control error for position velocity and force tube deviations.
 * Error function for moveConstrained_follow_path task.
*/
void dynamics_controller::compute_moveConstrained_follow_path_task_error()
{
    // Saturate linear force measurements in Z (sensor frame) direction
    if (ext_wrench_(2) < 0.0) ext_wrench_(2) = 0.0;

    // Reset the flags
    for (int i = 0; i < 6; i++)
    {
        MOTION_CTRL_DIM_[i] = false;
        FORCE_CTRL_DIM_[i]  = false;
    }

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveConstrained_null_space_task_error();

    // End-effector/Sensor Frame is the task frame for force DOFs
    moveConstrained_follow_path_task_.tf_force = robot_state_.frame_pose[END_EFF_].M;
    ext_wrench_base_ = moveConstrained_follow_path_task_.tf_force * ext_wrench_;
    desired_state_base_.external_force[END_EFF_] = moveConstrained_follow_path_task_.tf_force * desired_state_.external_force[END_EFF_];

    // Force-task FSM has priority over motion-task FSM
    if (previous_control_status_ != control_status::STOP_ROBOT)
    {
        // This function expects external wrench values to be expressed w.r.t. sensor frame
        fsm_force_task_result_ = fsm_.update_force_task_status(desired_state_.external_force[END_EFF_], 
                                                               ext_wrench_,
                                                               total_time_sec_,
                                                               0.014);
    }

    switch (fsm_force_task_result_)
    {
        case control_status::APPROACH:
            // Set ABAG parameters for linear Z axis velocity control 
            if (previous_control_status_ == control_status::NOMINAL)
            {
                // Parameters for Velocity controlled DOF
                abag_.set_error_alpha(   abag_parameter::ERROR_ALPHA(2),    2);
                abag_.set_bias_threshold(abag_parameter::BIAS_THRESHOLD(2), 2);
                abag_.set_bias_step(     abag_parameter::BIAS_STEP(2),      2);
                abag_.set_gain_threshold(abag_parameter::GAIN_THRESHOLD(2), 2);
                abag_.set_gain_step(     abag_parameter::GAIN_STEP(2),      2);
                abag_.set_min_bias_sat_limit(   Eigen::VectorXd::Constant(6, -1.0));
                abag_.set_min_command_sat_limit(Eigen::VectorXd::Constant(6, -1.0));
                max_command_(2) = 60.0;
            }

            // Linear DOFs: Control Linear X,Y,Z Motion w.r.t. base frame
            for (int i = 0; i < 2; i++)
            {
                abag_error_vector_(i) = 0.0 - robot_state_.frame_velocity[END_EFF_](i);
                if (std::fabs(abag_error_vector_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[6]) abag_error_vector_(i) = 0.0;
                MOTION_CTRL_DIM_[i] = true;
            }
            abag_error_vector_(2) = -0.02 - robot_state_.frame_velocity[END_EFF_](2);
            if (std::fabs(abag_error_vector_(2)) <= moveConstrained_follow_path_task_.tube_tolerances[6]) abag_error_vector_(2) = 0.0;
            MOTION_CTRL_DIM_[2] = true;

            for (int i = 3; i < 5; i++)
            {
                abag_error_vector_(i) = 0.0 - ext_wrench_(i);
                if (std::fabs(abag_error_vector_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[i]) abag_error_vector_(i) = 0.0;
                FORCE_CTRL_DIM_[i]  = true;       
            }

            // Set null-space error tolerance; small null-space oscillations are desired in this mode
            // moveConstrained_follow_path_task_.null_space_tolerance = 15.0;
            
            // Motion error is computed in the base-frame
            transform_drivers_ = false;
            fsm_result_ = control_status::APPROACH;
            break;

        case control_status::CRUISE:
            // Set ABAG parameters for linear Z axis force control 
            if (previous_control_status_ == control_status::APPROACH)
            {
                abag_.reset_state(0);
                abag_.reset_state(1);
                abag_.reset_state(2);
                
                // Parameters for linear force controlled DOF
                abag_.set_error_alpha(   force_task_parameters_(0), 2);
                abag_.set_bias_threshold(force_task_parameters_(1), 2);
                abag_.set_bias_step(     force_task_parameters_(2), 2);
                abag_.set_gain_threshold(force_task_parameters_(3), 2);
                abag_.set_gain_step(     force_task_parameters_(4), 2);
                abag_.set_min_bias_sat_limit(   min_sat_limits_);
                abag_.set_min_command_sat_limit(min_sat_limits_);
                max_command_(2) = force_task_parameters_(5);
                apply_feedforward_force_ = true;

                // Visualize time flag in control graphs, for this control mode switching
                write_contact_time_to_file_ = true;
            }

            // Update tube section count 
            if (previous_control_status_ == control_status::CHANGE_TUBE_SECTION) tube_section_count_++;
            if (tube_section_count_ > moveConstrained_follow_path_task_.tf_poses.size() - 1) tube_section_count_ = moveConstrained_follow_path_task_.tf_poses.size() - 1;

            // Make prediction while the state is expressed in the base frame
            make_predictions(horizon_amplitude_, 1);

            //Change the reference frame of the robot motion state, from base frame to task frame
            robot_state_.frame_pose[END_EFF_]        = moveConstrained_follow_path_task_.tf_poses[tube_section_count_].Inverse()   * robot_state_.frame_pose[END_EFF_];
            robot_state_.frame_velocity[END_EFF_]    = moveConstrained_follow_path_task_.tf_poses[tube_section_count_].M.Inverse() * robot_state_.frame_velocity[END_EFF_];
            predicted_state_.frame_pose[END_EFF_]    = moveConstrained_follow_path_task_.tf_poses[tube_section_count_].Inverse()   * predicted_state_.frame_pose[END_EFF_];
            desired_state_base_.frame_pose[END_EFF_] = moveConstrained_follow_path_task_.tf_poses[tube_section_count_]             * desired_state_.frame_pose[END_EFF_];

            current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);
            predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

            for (int i = 0; i < 2; i++)
                current_error_twist_(i) = POS_TUBE_DIM_[i]? current_error_twist_(i) : 0.0;

            // fsm_result_ = control_status::CRUISE_THROUGH_TUBE;
            fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
                                                         ext_wrench_, total_time_sec_, tube_section_count_);

            // for (int i = 0; i < 2; i++)
            // {
            //     abag_error_vector_(i) = desired_state_.frame_velocity[END_EFF_](i) - robot_state_.frame_velocity[END_EFF_](i);
            //     if (std::fabs(abag_error_vector_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[6]) abag_error_vector_(i) = 0.0;
            //     MOTION_CTRL_DIM_[i] = true;
            // }

            // Check for tube on velocity X-linear
            abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_](0) - robot_state_.frame_velocity[END_EFF_](0);
            if ((desired_state_.frame_velocity[END_EFF_](0) != 0.0) && \
                (std::fabs(abag_error_vector_(0)) <= moveConstrained_follow_path_task_.tube_tolerances[6])) abag_error_vector_(0) = 0.0;
            MOTION_CTRL_DIM_[0] = true;

            // Check for tube on position Y-linear
            abag_error_vector_(1) = predicted_error_twist_(1);
            if (std::fabs(abag_error_vector_(1)) <= moveConstrained_follow_path_task_.tube_tolerances[1]) abag_error_vector_(1) = 0.0;
            MOTION_CTRL_DIM_[1] = true;

            // Check for tube on force: Z linear, sensor frame
            abag_error_vector_(2) = desired_state_.external_force[END_EFF_](2) - ext_wrench_(2);
            if (std::fabs(abag_error_vector_(2)) <= moveConstrained_follow_path_task_.tube_tolerances[2]) abag_error_vector_(2) = 0.0;
            FORCE_CTRL_DIM_[2] = true;


            // Check for tube on angular force: X & Y-angular, sensor frame
            // for (int i = 3; i < 5; i++)
            // {
            //     abag_error_vector_(i) = 0.0 - ext_wrench_(i);
            //     if (std::fabs(abag_error_vector_(i)) <= moveConstrained_follow_path_task_.tube_tolerances[i]) abag_error_vector_(i) = 0.0;
            //     FORCE_CTRL_DIM_[i]  = true;       
            // }

            // Check for tube on angular force: X angular, sensor frame
            abag_error_vector_(3) = std::atan2(-ext_wrench_(1), ext_wrench_(2));
            if (std::fabs(abag_error_vector_(3)) <= moveConstrained_follow_path_task_.tube_tolerances[3]) abag_error_vector_(3) = 0.0;
            FORCE_CTRL_DIM_[3]  = true;

            // Check for tube on angular force: Y-angular, sensor frame
            abag_error_vector_(4) = std::atan2(-ext_wrench_(0), ext_wrench_(2));
            if (std::fabs(abag_error_vector_(4)) <= moveConstrained_follow_path_task_.tube_tolerances[4]) abag_error_vector_(4) = 0.0;
            FORCE_CTRL_DIM_[4]  = true;


            // Check for tube on angular velocity: Z-angular
            // abag_error_vector_(5) = 0.0 - robot_state_.frame_velocity[END_EFF_](5);
            // if (std::fabs(abag_error_vector_(5)) <= moveConstrained_follow_path_task_.tube_tolerances[7]) abag_error_vector_(5) = 0.0;
            // MOTION_CTRL_DIM_[5] = true;

            // Set null-space error tolerance; small null-space oscillations are desired in this mode
            // moveConstrained_follow_path_task_.null_space_tolerance = 25.0;

            // Motion error is, in this case, expressed in the task frame
            transform_drivers_ = true;

            // Feedforward force for ensuring stable contact alignment once control mode switch occurs
            if (apply_feedforward_force_)
            {
                feedforward_loop_count_++;
                if (feedforward_loop_count_ > 20) apply_feedforward_force_ = false;
                else abag_error_vector_(2) = 1.0; // ABAG does not care about raw error magnitude
            }

            if (fsm_result_ == control_status::STOP_ROBOT) fsm_force_task_result_ = control_status::STOP_ROBOT;
            else break;

        default:
            if (previous_control_status_ != control_status::STOP_ROBOT) write_contact_time_to_file_ = true;
            fsm_result_ = control_status::STOP_ROBOT;
            
            // Reset the flags
            for (int i = 0; i < 6; i++)
            {
                MOTION_CTRL_DIM_[i] = false;
                FORCE_CTRL_DIM_[i]  = false;
            }
            compute_null_space_command_ = false;
            return;
    }
}

void dynamics_controller::compute_moveToGuarded_null_space_task_error()
{
    // Youbot (URDF model) joint 2, 3 and 4 offset angles for candle pose are 1.13446, -2.54818 and 1.78896
    null_space_angle_ = (robot_state_.q(1) - 1.13446) + (robot_state_.q(2) + 2.54818) + (robot_state_.q(3) - 1.78896);

    // Desired orientation of the link is to be aligned with the table
    //_Relative_ desired angle from Z (base) axis. Anti-Clock wise direction.
    null_space_abag_error_(0) = DEG_TO_RAD(94.0) - null_space_angle_;

    // Unit of this tube tolerance is degree
    switch (desired_task_model_)
    {
        case task_model::moveTo_follow_path:
            if (std::fabs(null_space_abag_error_(0)) <= DEG_TO_RAD(moveTo_follow_path_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
            break;

        case task_model::moveTo_weight_compensation:
            if (std::fabs(null_space_abag_error_(0)) <= DEG_TO_RAD(moveTo_weight_compensation_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
            break;

        case task_model::moveGuarded:
            if (std::fabs(null_space_abag_error_(0)) <= DEG_TO_RAD(moveGuarded_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
            break;

        case task_model::moveTo:
            if (std::fabs(null_space_abag_error_(0)) <= DEG_TO_RAD(moveTo_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
            break;

        default:
            if (std::fabs(null_space_abag_error_(0)) <= DEG_TO_RAD(full_pose_task_.null_space_tolerance)) null_space_abag_error_(0) = 0.0;
            break;
    }
}

/**
 * Compute the control error for position and velocity tube deviations.
 * Error function for moveTo_follow_path task.
*/
void dynamics_controller::compute_moveTo_follow_path_task_error()
{
    if (previous_control_status_ == control_status::CHANGE_TUBE_SECTION) tube_section_count_++;
    if (tube_section_count_ > moveTo_follow_path_task_.tf_poses.size() - 1) tube_section_count_ = moveTo_follow_path_task_.tf_poses.size() - 1;

    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]        = moveTo_follow_path_task_.tf_poses[tube_section_count_].Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_]    = moveTo_follow_path_task_.tf_poses[tube_section_count_].M.Inverse() * robot_state_.frame_velocity[END_EFF_];
    desired_state_base_.frame_pose[END_EFF_] = moveTo_follow_path_task_.tf_poses[tube_section_count_]             * desired_state_.frame_pose[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
                                                 ext_wrench_, total_time_sec_, tube_section_count_);
    
    abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_].vel(0) - robot_state_.frame_velocity[END_EFF_].vel(0);
    
    // Check for tube on velocity
    if ((desired_state_.frame_velocity[END_EFF_].vel(0) != 0.0) && \
        (std::fabs(abag_error_vector_(0)) <= moveTo_follow_path_task_.tube_tolerances[6])) abag_error_vector_(0) = 0.0;

    // Other parts of the ABAG error are position errors
    for(int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if ( std::fabs(predicted_error_twist_(i)) <= moveTo_follow_path_task_.tube_tolerances[i] ) abag_error_vector_(i) = 0.0;
        else abag_error_vector_(i) = predicted_error_twist_(i);        
    }

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveToGuarded_null_space_task_error();
}


/**
 * Compute the control error for tube deviations and velocity setpoint.
 * Error function for moveTo task.
*/
void dynamics_controller::compute_moveTo_task_error()
{
    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]        = moveTo_task_.tf_pose.Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_]    = moveTo_task_.tf_pose.M.Inverse() * robot_state_.frame_velocity[END_EFF_];
    desired_state_base_.frame_pose[END_EFF_] = moveTo_task_.tf_pose             * desired_state_.frame_pose[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
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

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveToGuarded_null_space_task_error();
}

/**
 * Compute the control error for tube deviations and velocity setpoint.
 * Error function for moveTo weight_compensation task.
*/
void dynamics_controller::compute_moveTo_weight_compensation_task_error()
{
    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]        = moveTo_weight_compensation_task_.tf_pose.Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_]    = moveTo_weight_compensation_task_.tf_pose.M.Inverse() * robot_state_.frame_velocity[END_EFF_];
    desired_state_base_.frame_pose[END_EFF_] = moveTo_weight_compensation_task_.tf_pose             * desired_state_.frame_pose[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
                                                 ext_wrench_, total_time_sec_, tube_section_count_);

    abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_].vel(0) - robot_state_.frame_velocity[END_EFF_].vel(0);

    // Check for tube on velocity
    if ((desired_state_.frame_velocity[END_EFF_].vel(0) != 0.0) && \
        (std::fabs(abag_error_vector_(0)) <= moveTo_weight_compensation_task_.tube_tolerances[6]))
    {
        abag_error_vector_(0) = 0.0;
    }

    // Other parts of the ABAG error are position errors
    for (int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if ( std::fabs(predicted_error_twist_(i)) <= moveTo_weight_compensation_task_.tube_tolerances[i] ) abag_error_vector_(i) = 0.0;
        else abag_error_vector_(i) = predicted_error_twist_(i);        
    }

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveToGuarded_null_space_task_error();
}


/**
 * Compute the control error for tube deviations and velocity setpoint.
 * Error function for moveGuarded task.
*/
void dynamics_controller::compute_moveGuarded_task_error()
{
    //Change the reference frame of the robot state, from base frame to task frame
    robot_state_.frame_pose[END_EFF_]        = moveGuarded_task_.tf_pose.Inverse()   * robot_state_.frame_pose[END_EFF_];
    robot_state_.frame_velocity[END_EFF_]    = moveGuarded_task_.tf_pose.M.Inverse() * robot_state_.frame_velocity[END_EFF_];
    desired_state_base_.frame_pose[END_EFF_] = moveGuarded_task_.tf_pose             * desired_state_.frame_pose[END_EFF_];

    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);

    make_predictions(horizon_amplitude_, 1);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );

    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
                                                 ext_wrench_, total_time_sec_, tube_section_count_);

    abag_error_vector_(0) = desired_state_.frame_velocity[END_EFF_].vel(0) - robot_state_.frame_velocity[END_EFF_].vel(0);

    // Check for tube on velocity
    if ((fsm_result_ == control_status::CRUISE_THROUGH_TUBE) && \
        (std::fabs(abag_error_vector_(0)) <= moveGuarded_task_.tube_tolerances[6])) abag_error_vector_(0) = 0.0;

    // Other parts of the ABAG error are position errors
    for (int i = 1; i < NUM_OF_CONSTRAINTS_; i++)
    {
        if ( std::fabs(predicted_error_twist_(i)) <= moveGuarded_task_.tube_tolerances[i] ) abag_error_vector_(i) = 0.0;
        else abag_error_vector_(i) = predicted_error_twist_(i);        
    }

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveToGuarded_null_space_task_error();
}

/**
 * Compute the error between desired Cartesian state and predicted (integrated) Cartesian state.
 * Error function for full_pose task
*/
void dynamics_controller::compute_full_pose_task_error()
{
    make_predictions(horizon_amplitude_, 1);
    current_error_twist_   = finite_displacement_twist(desired_state_, robot_state_);
    predicted_error_twist_ = conversions::kdl_twist_to_eigen( finite_displacement_twist(desired_state_, predicted_state_) );
    
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        current_error_twist_(i) = CTRL_DIM_[i]? current_error_twist_(i) : 0.0;

    fsm_result_            = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, 
                                                            ext_wrench_, total_time_sec_, tube_section_count_);
    abag_error_vector_     = predicted_error_twist_;

    // Additional Cartesian force to keep residual part of the robot in a good configuration
    if (compute_null_space_command_) compute_moveToGuarded_null_space_task_error();
}

/**
 * Compute the general control error. 
 * Internally an error function will be called for each selected task
*/
void dynamics_controller::compute_control_error()
{    
    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            compute_moveConstrained_follow_path_task_error();
            break;

        case task_model::moveTo_follow_path:
            compute_moveTo_follow_path_task_error();
            break;

        case task_model::moveTo:
            compute_moveTo_task_error();
            break;

        case task_model::moveGuarded:
            compute_moveGuarded_task_error();
            break;

        case task_model::moveTo_weight_compensation:
            compute_moveTo_weight_compensation_task_error();
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
    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            cart_force_command_[END_EFF_] = moveConstrained_follow_path_task_.tf_force * cart_force_command_[END_EFF_];
            break;
        
        case task_model::moveTo_follow_path:
            cart_force_command_[END_EFF_] = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * cart_force_command_[END_EFF_];
            break;

        case task_model::moveTo_weight_compensation:
            cart_force_command_[END_EFF_] = moveTo_weight_compensation_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;

        case task_model::moveGuarded:
            cart_force_command_[END_EFF_] = moveGuarded_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;

        default:
            cart_force_command_[END_EFF_] = moveTo_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;
    }
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
        
        switch (desired_task_model_)
        {
            case task_model::moveConstrained_follow_path:
                wrench_column = moveConstrained_follow_path_task_.tf_poses[tube_section_count_].M * wrench_column;
                break;

            case task_model::moveTo_follow_path:
                wrench_column = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * wrench_column;
                break;

            case task_model::moveTo_weight_compensation:
                wrench_column = moveTo_weight_compensation_task_.tf_pose.M * wrench_column;
                break;

            case task_model::moveGuarded:
                wrench_column = moveGuarded_task_.tf_pose.M * wrench_column;
                break;

            default:
                wrench_column = moveTo_task_.tf_pose.M * wrench_column;
                break;
        }

        // Change Data Type to fit Vereshchagin
        twist_column = KDL::Twist(wrench_column.force, wrench_column.torque);
        robot_state_.ee_unit_constraint_force.setColumn(c, twist_column);
    }
}

void dynamics_controller::compute_null_space_control_commands()
{
    // First reset force magnitudes for the null space task control
    KDL::SetToZero(cart_force_command_[3].force);

    // Null space control commands to align third segment with arm's plane
    if (compute_null_space_command_)
    {
        // Used for moveConstrained task execution with LWR 4 robot
        if (desired_task_model_ == task_model::moveConstrained_follow_path)
        {
            // Compute null-space control command
            null_space_abag_command_     = abag_null_space_.update_state(null_space_abag_error_)(0) * 250.0;

            // Span the command over unit force vector space
            cart_force_command_[3].force = moveConstrained_follow_path_task_.null_space_force_direction * null_space_abag_command_;

            // Transform the Cartesian force from arm's plane-frame to base frame
            cart_force_command_[3].force = moveConstrained_follow_path_task_.null_space_plane_orientation * cart_force_command_[3].force;
        }
        // Used for moveTo (follow_path) / moveGuarded task execution with youBot
        else
        {
            // Compute null-space control command
            null_space_abag_command_ = abag_null_space_.update_state(null_space_abag_error_)(0) * 5.0;
            
            // Feed-Forward Torque is applied in order to keep end-effector link aligned with table plane
            robot_state_.feedforward_torque(3) = null_space_abag_command_;
        }
    }
}

void dynamics_controller::compute_cart_control_commands()
{   
    abag_command_ = abag_.update_state(abag_error_vector_).transpose();
    KDL::SetToZero(cart_force_command_[END_EFF_]);

    switch (desired_dynamics_interface_)
    {
        case dynamics_interface::CART_FORCE:
            // Set virtual forces computed by the ABAG controller
            for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
                cart_force_command_[END_EFF_](i) = CTRL_DIM_[i]? abag_command_(i) * max_command_(i) : 0.0;

            if (transform_drivers_) transform_force_driver();
            break;

        case dynamics_interface::CART_ACCELERATION:
            // Set Cartesian Acceleration Constraints on the End-Effector
            set_ee_acc_constraints(robot_state_,
                                   std::vector<bool>{MOTION_CTRL_DIM_[0], MOTION_CTRL_DIM_[1], MOTION_CTRL_DIM_[2], // Linear
                                                     MOTION_CTRL_DIM_[3], MOTION_CTRL_DIM_[4], MOTION_CTRL_DIM_[5]}, // Angular
                                   std::vector<double>{abag_command_(0) * max_command_(0), // Linear
                                                       abag_command_(1) * max_command_(1), // Linear
                                                       abag_command_(2) * max_command_(2), // Linear
                                                       abag_command_(3) * max_command_(3), // Angular
                                                       abag_command_(4) * max_command_(4), // Angular
                                                       abag_command_(5) * max_command_(5)}); // Angular

            // Change the reference frame of constraint forces, from task frame to base frame
            if (transform_drivers_) transform_motion_driver();

            // Use external force interface only for force commands in task specs
            if (desired_task_model_ == task_model::moveConstrained_follow_path)
            {
                for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
                    cart_force_command_[END_EFF_](i) = FORCE_CTRL_DIM_[i]? abag_command_(i) * max_command_(i) : 0.0;
            }

            // Change the reference frame of External Forces, from the task frame to the base frame
            if (transform_force_drivers_) transform_force_driver();
            if (apply_feedforward_force_) cart_force_command_[END_EFF_](2) = - max_command_(2)/3;
            break;

        default:
            assert(("Unsupported interface!", false));
            break;
    }

    compute_null_space_control_commands();
}

void dynamics_controller::compute_weight_compensation_control_commands()
{
    // Values expressed in the task frames
    int compensation_status = fsm_.update_weight_compensation_task_status(loop_iteration_count_,
                                                                          abag_.get_bias(), 
                                                                          abag_.get_gain(), 
                                                                          filtered_bias_);
    // Update error and control command for linear X axis
    if (compensation_status == 1) 
    {
        // Values expressed in the task frame: offset - current bias
        compensation_error_(0) = compensation_parameters_(2) - filtered_bias_(0);
        if (std::fabs(compensation_error_(0)) <= compensation_parameters_(1)) compensation_error_(0) = 0.0;

        // Force in task frame = error in procentage * max command * proportional gain
        compensated_weight_.force = compensated_weight_.force + KDL::Vector(compensation_error_(0) * max_command_(0) * compensation_parameters_(0), 0.0, 0.0);
        
        #ifndef NDEBUG
            printf("X Force: %f\n", compensated_weight_.force(0));
        #endif

        // Transform external force from task frame to the base frame
        robot_state_.external_force[END_EFF_].force = moveTo_weight_compensation_task_.tf_pose.M * compensated_weight_.force;
    }
    
    // Update error and control command for linear Z axis
    else if (compensation_status == 2) 
    {
        // Values expressed in the task frame: offset - current bias
        compensation_error_(2) = compensation_parameters_(2) - filtered_bias_(2);
        if (std::fabs(compensation_error_(2)) <= compensation_parameters_(1)) compensation_error_(2) = 0.0;

        // compensation_error_(2) = compensation_parameters_(2) + filtered_bias_(2);
        // if (compensation_error_(2) >= -compensation_parameters_(1)) compensation_error_(2) = 0.0;

        // Force in task frame = error in procentage * max command * proportional gain
        compensated_weight_.force = compensated_weight_.force + KDL::Vector(0.0, 0.0, compensation_error_(2) * max_command_(2) * compensation_parameters_(0) * 0.8);
        // compensated_weight_.force = compensated_weight_.force + KDL::Vector(compensation_error_(2) * max_command_(2) * compensation_parameters_(0)*0.9, 0.0, 0.0);
        
        #ifndef NDEBUG
            printf("Z Force: %f\n", compensated_weight_.force(2));
        #endif

        // Transform external force from task frame to the base frame
        robot_state_.external_force[END_EFF_].force = moveTo_weight_compensation_task_.tf_pose.M * compensated_weight_.force;
    }
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

    if (hd_solver_result != 0) return hd_solver_result;

    hd_solver_.get_control_torque(robot_state_.control_torque);

    // hd_solver_.get_transformed_link_acceleration(robot_state_.frame_acceleration);
    
    // Print computed state in Debug mode
// #ifndef NDEBUG
        // std::cout << "\nComputed Cartesian state:" << std::endl;

        // std::cout << "Frame ACC" << '\n';
        // for (size_t i = 0; i < NUM_OF_SEGMENTS_ + 1; i++)
        //     std::cout << robot_state_.frame_acceleration[i] << '\n';

        // std::cout << "End-effector Position:   " 
        //       << robot_state_.frame_pose[END_EFF_].p  << std::endl;

        // std::cout << "\nComputed Joint state:          " << std::endl;
        // std::cout << "Joint torque:  " << robot_state_.control_torque << std::endl;
        // std::cout << "Joint acc:     " << robot_state_.qdd << std::endl;
// #endif 

// #ifdef NDEBUG
        // std::cout << "Joint torque:  " << robot_state_.control_torque << std::endl;
// #endif

    return hd_solver_result;
}

int dynamics_controller::compute_gravity_compensation_control_commands()
{
    int id_solver_result = id_solver_.CartToJnt(robot_state_.q, zero_joint_array_, zero_joint_array_, 
                                                zero_wrenches_, gravity_torque_);
    if (id_solver_result != 0) return id_solver_result;
    
    robot_state_.control_torque.data = robot_state_.control_torque.data + gravity_torque_.data;
    
    for (int j = 0; j < NUM_OF_JOINTS_; j++)
    {
        if      (robot_state_.control_torque(j) >=  JOINT_TORQUE_LIMITS_[j]) robot_state_.control_torque(j) =  JOINT_TORQUE_LIMITS_[j] - 0.001;
        else if (robot_state_.control_torque(j) <= -JOINT_TORQUE_LIMITS_[j]) robot_state_.control_torque(j) = -JOINT_TORQUE_LIMITS_[j] + 0.001;
    }

    return id_solver_result;
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
                                         const Eigen::VectorXd &min_command_sat,
                                         const Eigen::VectorXd &null_space_abag_parameters,
                                         const Eigen::VectorXd &compensation_parameters)
{
    //First check input dimensions
    assert(max_command.size()    == NUM_OF_CONSTRAINTS_); 
    assert(error_alpha.size()    == NUM_OF_CONSTRAINTS_); 
    assert(bias_threshold.size() == NUM_OF_CONSTRAINTS_); 
    assert(bias_step.size()      == NUM_OF_CONSTRAINTS_); 
    assert(gain_threshold.size() == NUM_OF_CONSTRAINTS_); 
    assert(gain_step.size()      == NUM_OF_CONSTRAINTS_); 

    this->horizon_amplitude_        = horizon_amplitude;
    this->max_command_              = max_command;

    // Setting parameters of the force controller DOF
    this->force_task_parameters_(0) = error_alpha(2);
    this->force_task_parameters_(1) = bias_threshold(2);
    this->force_task_parameters_(2) = bias_step(2);
    this->force_task_parameters_(3) = gain_threshold(2);
    this->force_task_parameters_(4) = gain_step(2);
    this->force_task_parameters_(5) = max_command(2);
    this->min_sat_limits_ = min_command_sat;

    // Setting parameters of the ABAG Controller
    abag_.set_error_alpha(error_alpha);    
    abag_.set_bias_threshold(bias_threshold);
    abag_.set_bias_step(bias_step);
    abag_.set_gain_threshold(gain_threshold);
    abag_.set_gain_step(gain_step);
    abag_.set_min_bias_sat_limit(min_bias_sat);
    abag_.set_min_command_sat_limit(min_command_sat);
    abag_.set_error_type(abag_error_type);

    abag_null_space_.set_error_alpha(   null_space_abag_parameters(0), 0);
    abag_null_space_.set_bias_threshold(null_space_abag_parameters(1), 0);
    abag_null_space_.set_bias_step(     null_space_abag_parameters(2), 0);
    abag_null_space_.set_gain_threshold(null_space_abag_parameters(3), 0);
    abag_null_space_.set_gain_step(     null_space_abag_parameters(4), 0);

    this->compensation_parameters_ = compensation_parameters;
}

int dynamics_controller::initialize(const int desired_control_mode, 
                                    const int desired_dynamics_interface,
                                    const bool store_control_data,
                                    const int motion_profile)
{
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;

    //Exit the program if the "Stop Motion" mode is selected
    assert(desired_control_mode_.interface != control_mode::STOP_MOTION); 

    desired_dynamics_interface_ = desired_dynamics_interface;

    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            fsm_result_ = fsm_.initialize_with_moveConstrained_follow_path(moveConstrained_follow_path_task_, 
                                                                           motion_profile);
            break;

        case task_model::moveTo_follow_path:
            fsm_result_ = fsm_.initialize_with_moveTo_follow_path(moveTo_follow_path_task_, 
                                                                  motion_profile);
            break;

        case task_model::moveTo:
            fsm_result_ = fsm_.initialize_with_moveTo(moveTo_task_, motion_profile);
            break;

        case task_model::moveGuarded:
            fsm_result_ = fsm_.initialize_with_moveGuarded(moveGuarded_task_, motion_profile);
            break;

        case task_model::moveTo_weight_compensation:
            fsm_result_ = fsm_.initialize_with_moveTo_weight_compensation(moveTo_weight_compensation_task_, 
                                                                          motion_profile,
                                                                          compensation_parameters_);
            break;

        case task_model::full_pose:
            fsm_result_ = fsm_.initialize_with_full_pose(full_pose_task_, motion_profile);
            break;
            
        default:
            printf("Unsupported task model\n");
            return -1;
            break;
    }

    store_control_data_ = store_control_data;

    if (store_control_data_) 
    {
        log_file_cart_.open(dynamics_parameter::LOG_FILE_CART_PATH);
        assert(log_file_cart_.is_open());

        for (int i = 0; i < NUM_OF_CONSTRAINTS_ + 2; i++)
        {
            if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_cart_ << moveConstrained_follow_path_task_.tube_tolerances[i] << " ";            
            else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_cart_ << moveTo_follow_path_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_cart_ << moveTo_weight_compensation_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveGuarded)                 log_file_cart_ << moveGuarded_task_.tube_tolerances[i] << " ";
            else                                                                     log_file_cart_ << moveTo_task_.tube_tolerances[i] << " ";
        }
        log_file_cart_ << std::endl;

        for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
            log_file_cart_ << max_command_(i) << " ";
        log_file_cart_ << std::endl;      

        log_file_cart_base_.open(dynamics_parameter::LOG_FILE_CART_BASE_PATH);
        assert(log_file_cart_base_.is_open());

        for (int i = 0; i < NUM_OF_CONSTRAINTS_ + 2; i++)
        {
            if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_cart_base_ << moveConstrained_follow_path_task_.tube_tolerances[i] << " ";            
            else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_cart_base_ << moveTo_follow_path_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_cart_base_ << moveTo_weight_compensation_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveGuarded)                 log_file_cart_base_ << moveGuarded_task_.tube_tolerances[i] << " ";
            else                                                                     log_file_cart_base_ << moveTo_task_.tube_tolerances[i] << " ";
        }
        log_file_cart_base_ << std::endl;

        log_file_predictions_.open(dynamics_parameter::LOG_FILE_PREDICTIONS_PATH);
        assert(log_file_predictions_.is_open());

        log_file_null_space_.open(dynamics_parameter::LOG_FILE_NULL_SPACE_PATH);
        assert(log_file_null_space_.is_open());
        if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_null_space_ << moveConstrained_follow_path_task_.null_space_tolerance << std::endl;           
        else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_null_space_ << moveTo_follow_path_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_null_space_ << moveTo_weight_compensation_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveGuarded)                 log_file_null_space_ << moveGuarded_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveTo)                      log_file_null_space_ << moveTo_task_.null_space_tolerance << std::endl;
        else                                                                     log_file_null_space_ << full_pose_task_.null_space_tolerance << std::endl;

        log_file_joint_.open(dynamics_parameter::LOG_FILE_JOINT_PATH);
        assert(log_file_joint_.is_open());
        for(int i = 0; i < NUM_OF_JOINTS_; i++) 
            log_file_joint_ << JOINT_TORQUE_LIMITS_[i] << " ";
        log_file_joint_ << std::endl;
    }

    KDL::SetToZero(robot_state_.feedforward_torque);

    // First make sure that the robot is not moving
    stop_robot_motion();

    /* 
        Get sensor data from the robot driver or if simulation is on, 
        replace current state with the integrated joint velocities and positions.
    */
    int state_update_result = update_current_state();  
    if (state_update_result != 0)
    {
        deinitialize();
        printf("Warning: FK solver returned an error! %d \n", state_update_result);
        return -1;
    }

    //Print information about controller settings
    print_settings_info();    
    return 0;
}

// Main control loop
int dynamics_controller::control()
{   
    double loop_time = 0.0;
    int ctrl_status  = 0;
    printf("Control Loop Started\n");

    while (1)
    {
        // Save current time point
        loop_iteration_count_++;
        loop_start_time_ = std::chrono::steady_clock::now();

        //Get current robot state from the joint sensors: velocities and angles
        ctrl_status = update_current_state();  
        if (ctrl_status != 0)
        {
            deinitialize();
            printf("Warning: FK solver returned an error! %d \n", ctrl_status);
            return -1;
        }
        // Save the state expressed in base frame
        robot_state_base_.frame_pose = robot_state_.frame_pose;

        compute_control_error();

        ctrl_status = check_fsm_status();
        if (ctrl_status != 0) 
        {
            deinitialize();
            if (ctrl_status == -1) return -1;
            else
            {
                printf("Control changed to Stop-Robot mode.\n");
                return 1;
            }
        }

        compute_cart_control_commands();
        if (compensate_unknown_weight_) compute_weight_compensation_control_commands();

        // Evaluate robot dynamics using the Vereshchagin HD solver
        ctrl_status = evaluate_dynamics();
        if (ctrl_status != 0)
        {
            deinitialize();
            printf("WARNING: Hybrid Dynamics Solver returned error: %d. Stopping the robot!", ctrl_status);
            return -1;
        }

        // Compute necessary torques for compensating gravity, using the RNE ID solver
        if (compensate_gravity_) 
        {
            ctrl_status = compute_gravity_compensation_control_commands();
            if (ctrl_status != 0)
            {
                deinitialize();
                printf("WARNING: Inverse Dynamics Solver returned error: %d. Stopping the robot!", ctrl_status);
                return -1;
            }
        }

        if (store_control_data_) write_to_file();
        // return 0;

        // Apply joint commands using safe control interface.
        if (apply_joint_control_commands() != 0)
        {
            deinitialize();
            printf("WARNING: Computed commands are not safe. Stopping the robot!\n");
            return -1;
        }

        // Make sure that the loop is always running with the same frequency
        if (enforce_loop_frequency() != 0) printf("WARNING: Control loop runs too slow \n");

        // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_).count();
        // if (loop_iteration_count_ == 50) 
        // {
        //     std::cout << loop_time / 50.0 <<std::endl;
        //     return 0;
        // }
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
                              const double time_passed_sec,
                              const int loop_iteration)
{
    robot_state_.q  = q_input;
    robot_state_.qd = qd_input;
    ext_wrench_     = ext_force;
    total_time_sec_ = time_passed_sec;
    loop_iteration_count_ = loop_iteration;

    // Get Cart poses and velocities
    int fk_solver_result = fk_vereshchagin_.JntToCart(robot_state_.q, 
                                                      robot_state_.qd, 
                                                      robot_state_.frame_pose, 
                                                      robot_state_.frame_velocity);
    if (fk_solver_result != 0)
    {
        deinitialize();
        printf("Warning: FK solver returned an error! %d \n", fk_solver_result);
        return -1;
    }
    // Save the state expressed in base frame
    robot_state_base_.frame_pose = robot_state_.frame_pose;

    compute_control_error();

    if (check_fsm_status() == -1) return -1;

    compute_cart_control_commands();
    if (compensate_unknown_weight_) compute_weight_compensation_control_commands();

    // Evaluate robot dynamics using the Vereshchagin HD solver
    if (evaluate_dynamics() != 0)
    {
        deinitialize();
        printf("WARNING: Hybrid Dynamics Solver returned error. Stopping the robot!\n");
        return -1;
    }

    // Compute necessary torques for compensating gravity, using the RNE ID solver
    if (compensate_gravity_) 
    {
        if (compute_gravity_compensation_control_commands() != 0)
        {
            deinitialize();
            printf("WARNING: Inverse Dynamics Solver returned error. Stopping the robot!\n");
            return -1;
        }
    }

    if (store_control_data_) write_to_file();

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
        log_file_cart_base_.close();
        log_file_joint_.close();
        log_file_predictions_.close();
        log_file_null_space_.close();
    }

    #ifndef NDEBUG
        printf("Number of iterations: %d\n", loop_iteration_count_);
    #endif
}