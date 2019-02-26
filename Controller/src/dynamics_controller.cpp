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
#define SECOND 1000000

dynamics_controller::dynamics_controller(youbot_mediator &robot_driver,
                                         const int rate_hz):
    RATE_HZ_(rate_hz),
    // Time period defined in microseconds: 1s = 1 000 000us
    DT_MICRO_(SECOND / RATE_HZ_),  DT_SEC_(1.0 / static_cast<double>(RATE_HZ_)),
    loop_start_time_(), loop_end_time_(), //Not sure if required to init
    robot_chain_(robot_driver.get_robot_model()),
    NUM_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUM_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    END_EFF_(NUM_OF_SEGMENTS_ - 1),
    MAX_FORCE_(dynamics_parameter::MAX_FORCE),
    error_vector_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    abag_command_(Eigen::VectorXd::Zero(abag_parameter::DIMENSIONS)),
    hd_solver_(robot_chain_, robot_driver.get_joint_inertia(),
               robot_driver.get_root_acceleration(), NUM_OF_CONSTRAINTS_),
    fk_vereshchagin_(robot_chain_),
    safety_control_(robot_driver, true), 
    abag_(abag_parameter::DIMENSIONS, abag_parameter::USE_ERROR_MAGNITUDE),
    predictor_(robot_chain_),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_,
                 NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    robot_commands_(robot_state_),
    desired_state_(robot_state_),
    predicted_state_(robot_state_)
{
    assert(("Robot is not initialized", robot_driver.is_initialized()));
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
}

//Print information about controller settings
void dynamics_controller::print_settings_info()
{   
    #ifdef NDEBUG
        std::cout << "The program is build in RELEASE mode." << std::endl;
    #endif
    #ifndef NDEBUG
        std::cout << "The program is build in DEBUG mode." << std::endl;
    #endif
    
    std::cout << "Selected controller settings:" << std::endl;
    std::cout << "Control Loop Frequency: " << RATE_HZ_ << " Hz" << std::endl;
    std::cout << "Control Mode: ";

    switch(desired_control_mode_.interface) 
    {
        case control_mode::STOP_MOTION:
            std::cout << "STOP MOTION \n" << "Stopping the robot!" << std::endl;
            break;

        case control_mode::POSITION:
            std::cout << "Joint Position Control" << std::endl;
            break;

        case control_mode::VELOCITY:
            std::cout << "Joint Velocity Control" << std::endl;
            break;

        case control_mode::TORQUE:
            std::cout << "Joint Torque Control" << std::endl;
            break;
    }

    /* 
        Get sensor data from the robot driver or if simulation is on, 
        replace current state with the integrated joint velocities and positions
    */
    update_current_state();
    
    std::cout<<"\nInitial joint state: "<< std::endl;
    std::cout<< "Joint positions: "<< robot_state_.q << std::endl;
    std::cout<< "Joint velocities:"<< robot_state_.qd << "\n" << std::endl;

    std::cout<<"Initial Cartesian state: "<< std::endl;
    std::cout<< "End-effector position: "<< robot_state_.frame_pose[END_EFF_].p << std::endl;
    std::cout<< "End-effector orientation: \n"<< robot_state_.frame_pose[END_EFF_].M << std::endl;
    std::cout<< "End-effector velocity:"<< robot_state_.frame_velocity[END_EFF_] << "\n" << std::endl;
}

/* 
    If it is working on the real robot get sensor data from the driver 
    or if simulation is on, replace current state with 
    integrated joint velocities and positions.
*/
void dynamics_controller::update_current_state()
{
    safety_control_.get_current_state(robot_state_);
    
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

// Write control data to a file
void dynamics_controller::write_to_file()
{   
    for(int i = 0; i < 3; i++)
        log_file_ << robot_state_.frame_pose[END_EFF_].p(i) << " ";

    for(int i = 3; i < 6; i++) log_file_ << error_vector_(i) << " ";
    log_file_ << std::endl;

    for(int i = 0; i < 3; i++)
        log_file_ << desired_state_.frame_pose[END_EFF_].p(i) << " ";

    for(int i = 3; i < 6; i++) log_file_ << 0.0 << " ";
    log_file_ << std::endl;

    log_file_ << abag_.get_error().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_ << abag_.get_bias().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_ << abag_.get_gain().transpose().format(dynamics_parameter::WRITE_FORMAT);
    log_file_ << abag_.get_command().transpose().format(dynamics_parameter::WRITE_FORMAT);
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

void dynamics_controller::define_desired_ee_pose(
                            const std::vector<bool> &constraint_direction,
                            const std::vector<double> &cartesian_pose)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(cartesian_pose.size() == NUM_OF_CONSTRAINTS_ * 2);
    
    CTRL_DIM_ = constraint_direction;
    
    desired_state_.frame_pose[END_EFF_].p(0) = cartesian_pose[0];
    desired_state_.frame_pose[END_EFF_].p(1) = cartesian_pose[1];
    desired_state_.frame_pose[END_EFF_].p(2) = cartesian_pose[2];

    desired_state_.frame_pose[END_EFF_].M = \
        KDL::Rotation(cartesian_pose[3], cartesian_pose[4], cartesian_pose[5],
                      cartesian_pose[6], cartesian_pose[7], cartesian_pose[8],
                      cartesian_pose[9], cartesian_pose[10], cartesian_pose[11]);
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
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
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
            printf("WARNING: Computed commands are not safe. Stopping the robot!");
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

/**
 * Compute the error between current (measured) or desired Cartesian state 
 * and predicted (integrated) Cartesian state.
*/
void dynamics_controller::compute_control_error()
{
    make_predictions(1, 1);
    // make_predictions(0.1, 10);
    // make_predictions(0.0001, 10000);

    /**
     * This error part represents linear motion necessary to go from 
     * predicted to desired position (positive direction of translation).
    */
    error_vector_.head(3) = \
        conversions::kdl_vector_to_eigen(desired_state_.frame_pose[END_EFF_].p - \
                                         predicted_state_.frame_pose[END_EFF_].p);
    /**
     * Describes rotation required to align R_p with R_d.
     * It represents relative rotation from predicted state to 
     * desired state, expressed in the BASE frame!
     * Source: Luh et al. "Resolved-acceleration control of 
     * mechanical manipulators".
    */
    KDL::Rotation error_rot_matrix = desired_state_.frame_pose[END_EFF_].M * \
                                     predicted_state_.frame_pose[END_EFF_].M.Inverse();

    // Error calculation for angular part, i.e. logarithmic map on SO(3).
    error_vector_.tail(3) = \
        conversions::kdl_vector_to_eigen(geometry::log_map_so3(error_rot_matrix));

    #ifndef NDEBUG
        std::cout << "\nLinear Error: " << error_vector_.head(3).transpose() << "    Linear norm: " << error_vector_.head(3).norm() << std::endl;
        std::cout << "Angular Error: " << error_vector_.tail(3).transpose() << "         Angular norm: " << error_vector_.tail(3).norm() << std::endl;
    #endif
}

void dynamics_controller::compute_cart_control_commands()
{   
    abag_command_ = abag_.update_state(error_vector_).transpose();

#ifndef NDEBUG
    std::cout << "ABAG Commands: "<< abag_command_.transpose() << std::endl;
#endif

    // First reset old robot external forces to initial values
    robot_state_.external_force = desired_state_.external_force;

    // Add additional force computed by the ABAG controller
    robot_state_.external_force[END_EFF_].force(0)  =+ \
        CTRL_DIM_[0]? abag_command_(0) * MAX_FORCE_[0] : 0.0;
    
    robot_state_.external_force[END_EFF_].force(1)  =+ \
        CTRL_DIM_[1]? abag_command_(1) * MAX_FORCE_[1] : 0.0;
    
    robot_state_.external_force[END_EFF_].force(2)  =+ \
        CTRL_DIM_[2]? abag_command_(2) * MAX_FORCE_[2] : 0.0;  
    
    robot_state_.external_force[END_EFF_].torque(0) =+ \
        CTRL_DIM_[3]? abag_command_(3) * MAX_FORCE_[3] : 0.0;

    robot_state_.external_force[END_EFF_].torque(1) =+ \
        CTRL_DIM_[4]? abag_command_(4) * MAX_FORCE_[4] : 0.0;
    
    robot_state_.external_force[END_EFF_].torque(2) =+ \
        CTRL_DIM_[5]? abag_command_(5) * MAX_FORCE_[5] : 0.0;
}

// Update current dynamics intefaces using desired robot state specifications 
void dynamics_controller::update_dynamics_interfaces()
{ 
    robot_state_.ee_unit_constraint_force = desired_state_.ee_unit_constraint_force;
    robot_state_.ee_acceleration_energy   = desired_state_.ee_acceleration_energy;
    robot_state_.feedforward_torque       = desired_state_.feedforward_torque;
    robot_state_.external_force           = desired_state_.external_force;
}

//Calculate robot dynamics - Resolve the motion using the Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    int hd_solver_result = hd_solver_.CartToJnt(robot_state_.q,
                                                robot_state_.qd,
                                                robot_state_.qdd,
                                                robot_state_.ee_unit_constraint_force,
                                                robot_state_.ee_acceleration_energy,
                                                robot_state_.external_force,
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

    //Print information about controller settings
    print_settings_info();

    if (store_control_data) 
    {
        log_file_.open(dynamics_parameter::LOG_FILE_PATH);
        if (!log_file_.is_open()) {
            std::cout << "Unable to open the file"<< std::endl;
            return -1;
        }
    }

    double loop_time = 0.0;
    int loop_count = 0;

    update_dynamics_interfaces();
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
            if (store_control_data) log_file_.close();
            printf("WARNING: Dynamics Solver returned error. Stopping the robot!");
            return -1;
        }
        if(loop_count == 1) return 0;

        // Apply joint commands using safe control interface.
        if(apply_joint_control_commands() != 0){
            if (store_control_data) log_file_.close();
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

    if (store_control_data) log_file_.close();
    return 0;
}