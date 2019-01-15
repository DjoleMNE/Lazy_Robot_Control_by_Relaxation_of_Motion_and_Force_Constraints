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
    DT_MICRO_(SECOND / RATE_HZ_),
    DT_SEC_(1.0 / static_cast<double>(RATE_HZ_)), 
    solver_result_(0),
    safe_control_mode_(-1),
    // loop_start_time_(), loop_end_time_(), //Not sure if required to init
    robot_chain_(robot_driver.get_robot_model()),
    NUMBER_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    NUMBER_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUMBER_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUMBER_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    hd_solver_(robot_chain_, robot_driver.get_joint_inertia(),
               robot_driver.get_root_acceleration(), NUMBER_OF_CONSTRAINTS_),
    safety_control_(robot_driver, true), 
    abag_(abag_parameter::DIMENSIONS, abag_parameter::USE_ERROR_MAGNITUDE),
    predictor_(robot_chain_),
    robot_state_(NUMBER_OF_JOINTS_, NUMBER_OF_SEGMENTS_, 
                    NUMBER_OF_FRAMES_, NUMBER_OF_CONSTRAINTS_), 
    commands_(robot_state_), 
    desired_state_(robot_state_), 
    predicted_state_(robot_state_)
{
    assert(("Robot is not initialized", robot_driver.is_initialized));
    // KDL Solver constraint  
    assert(NUMBER_OF_JOINTS_ == NUMBER_OF_SEGMENTS_);

    // Control loop frequency must be higher than or equal to 1 Hz
    assert(("Desired frequency is too low", 1 <= RATE_HZ_));
    // Control loop frequency must be lower than or equal to 1000 Hz
    assert(("Desired frequency is too high", RATE_HZ_<= 10000));
    
    // Set default command interface to velocity mode and initialize it as safe
    desired_control_mode_.interface = control_mode::STOP_MOTION;
    desired_control_mode_.is_safe = false;

    // Setting parameters of the ABAG Controller
    abag_.set_error_alpha(abag_parameter::ERROR_ALPHA);    
    abag_.set_bias_threshold(abag_parameter::BIAS_THRESHOLD);
    abag_.set_bias_step(abag_parameter::BIAS_STEP);
    abag_.set_gain_threshold(abag_parameter::GAIN_THRESHOLD);
    abag_.set_gain_step(abag_parameter::GAIN_STEP);
    
    Eigen::VectorXd v1(abag_parameter::DIMENSIONS);
    v1 = Eigen::VectorXd::Constant(abag_parameter::DIMENSIONS, 1.0 / 0.10);
    
    Eigen::VectorXd v2(abag_parameter::DIMENSIONS);
    v2 = Eigen::VectorXd::Constant(abag_parameter::DIMENSIONS, 1.0 / 50.0);
    
    // std::cout << "Command: \n" << abag_.update_state(v1, v2).transpose() << std::endl;
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

// Define Cartesian Acceleration task on the end-effector - Public Method
void dynamics_controller::define_ee_constraint_task(
                            const std::vector<bool> constraint_direction,
                            const std::vector<double> cartesian_acceleration)
{    
    //Call private method for this state
    set_ee_constraints(desired_state_, 
                       constraint_direction, 
                       cartesian_acceleration);
}

// Define Cartesian Acceleration task on the end-effector - Private Method
void dynamics_controller::set_ee_constraints(state_specification &state,
                                const std::vector<bool> constraint_direction, 
                                const std::vector<double> cartesian_acceleration)
{    
    assert(constraint_direction.size() == NUMBER_OF_CONSTRAINTS_);
    assert(cartesian_acceleration.size() == NUMBER_OF_CONSTRAINTS_);

    // Set directions in which constraint force should work 
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

    // Set desired acceleration on the end-effector
    for (int i = 0; i < NUMBER_OF_CONSTRAINTS_; i++)
        state.ee_acceleration_energy(i) = cartesian_acceleration[i];
}

// Define External force task - Public Method
void dynamics_controller::define_ee_external_force_task(
                                    const std::vector<double> external_force)
{
    //Call private method for this state
    set_external_forces(desired_state_, external_force);
}

// Define External force task - Private Method
void dynamics_controller::set_external_forces(state_specification &state, 
                                    const std::vector<double> external_force)
{
    //For now it is only updating forces on the end-effector
    //TODO: add forces on other segments as well
    assert(external_force.size() == NUMBER_OF_CONSTRAINTS_);

    state.external_force[state.external_force.size() - 1] = \
                                KDL::Wrench (KDL::Vector(external_force[0],
                                                         external_force[1],
                                                         external_force[2]),
                                             KDL::Vector(external_force[3],
                                                         external_force[4],
                                                         external_force[5]));
}

// Define FeedForward joint torques task - Public Method
void dynamics_controller::define_feadforward_torque_task(
                                            const std::vector<double> ff_torque)
{
    //Call private method for this state
    set_feadforward_torque(desired_state_, ff_torque);
}

// Define FeedForward joint torques task - Private Method
void dynamics_controller::set_feadforward_torque(
                                            state_specification &state, 
                                            const std::vector<double> ff_torque)
{
    assert(ff_torque.size() == NUMBER_OF_JOINTS_);

    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
        state.feedforward_torque(i) = ff_torque[i];
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

//Send joint commands to the robot driver
int dynamics_controller::apply_control_commands()
{ 
    /* 
        Safety controller checks if the commands are over the limits.
        If no: use desired control mode
        Else: stop the robot motion 
    */
    safe_control_mode_ = \
        safety_control_.set_control_commands(robot_state_, DT_SEC_, 
                                             desired_control_mode_.interface,
                                             integration_method::SYMPLECTIC_EULER);
   
    // Check if the safety controller has changed the control mode
    // Save the current decision if desired control mode is safe or not.
    desired_control_mode_.is_safe =\
        (desired_control_mode_.interface == safe_control_mode_)? true : false; 

    // Notify if the safety controller has changed the control mode
    switch(safe_control_mode_) {
        case control_mode::TORQUE:
            assert(desired_control_mode_.is_safe);
            return 0;

        case control_mode::VELOCITY:
            if(!desired_control_mode_.is_safe) 
                std::cout << "WARNING: Control switched to velocity mode \n" << std::endl;
            return 0;

        case control_mode::POSITION:
            if(!desired_control_mode_.is_safe) 
                std::cout << "WARNING: Control switched to position mode \n" << std::endl;
            return 0;

        default: 
            stop_robot_motion();
            std::cout << "WARNING: Computed commands are not safe. " 
                      << "Stopping the robot!" << std::endl;
            return -1;
    }
}

/*  
    Predict future robot states (motion) based given the computed state
    from the solver. I.e. Integrate Cartesian variables.
*/
void dynamics_controller::make_predictions(const int prediction_method)
{
    // predictor_.integrate_cartesian_space(robot_state_, 
    //                                      predicted_state_, 
    //                                      DT_SEC_, 1, 
    //                                      prediction_method);
}

//Calculate robot dynamics - Resolve the motion using the Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    solver_result_= hd_solver_.CartToJnt(robot_state_.q,
                                         robot_state_.qd,
                                         robot_state_.qdd,
                                         robot_state_.ee_unit_constraint_force,
                                         robot_state_.ee_acceleration_energy,
                                         robot_state_.external_force,
                                         robot_state_.feedforward_torque);

    if(solver_result_ != 0) return solver_result_;

    hd_solver_.get_transformed_link_pose(robot_state_.frame_pose);
    hd_solver_.get_transformed_link_velocity(robot_state_.frame_velocity);
    hd_solver_.get_transformed_link_acceleration(robot_state_.frame_acceleration);
    hd_solver_.get_control_torque(robot_state_.control_torque);
    
    // Print Cartesian state in Debug mode
    #ifndef NDEBUG
        std::cout << "Cartesian state:" << std::endl;
        std::cout << "Current End-effector Position: " 
                << robot_state_.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
                << std::endl;
        std::cout << "Current End-effector Velocity: " 
            << robot_state_.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
            << std::endl;    
        std::cout << "Computed Frame ACC" << '\n';
        for (size_t i = 0; i < NUMBER_OF_SEGMENTS_ + 1; i++)
            std::cout << robot_state_.frame_acceleration[i] << '\n';
        std::cout << "\nJoint state:" << std::endl;
        std::cout << "Current Joint angle" << robot_state_.q << std::endl;
        std::cout << "Current Joint velocity" << robot_state_.qd << std::endl;
        std::cout << "Computed Joint torque" << robot_state_.control_torque << std::endl;
    #endif 

    return solver_result_;
}

// Update the desired robot state 
void dynamics_controller::update_task()
{ 
    /* 
        TODO: This component should update desired state in case of a user 
        changing task specification in parallel (while control loop in this 
        component is running). Maybe something like a callback function.
    */
    robot_state_.ee_unit_constraint_force = desired_state_.ee_unit_constraint_force;
    robot_state_.ee_acceleration_energy = desired_state_.ee_acceleration_energy;
    robot_state_.external_force = desired_state_.external_force;
    robot_state_.feedforward_torque = desired_state_.feedforward_torque;
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

        case control_mode::VELOCITY:
            std::cout << "Velocity Control" << std::endl;
            break;

        case control_mode::POSITION:
            std::cout << "Position Control" << std::endl;
            break;

        case control_mode::TORQUE:
            std::cout << "Torque Control" << std::endl;
            break;
    }
    
    std::cout<<"\nInitial joint state: "<< std::endl;
    std::cout<< "Joint velocities:"<< robot_state_.qd << std::endl;
    std::cout<< "Joint positions: "<< robot_state_.q <<"\n" << std::endl;
}

//Get current robot state from the joint sensors, velocities and angles
void dynamics_controller::update_current_state()
{
    safety_control_.get_current_state(robot_state_);
}

//Send 0 joints velocities to the robot driver
void dynamics_controller::stop_robot_motion()
{   
    safety_control_.stop_robot_motion();
}

//Main control loop
int dynamics_controller::control(const int desired_control_mode, 
                                 const int desired_task_interface)
{   
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;
    
    stop_robot_motion();

    /* 
        Get sensor data from the robot driver or if simulation is on, 
        replace current state with the integrated joint velocities and positions
    */
    update_current_state();

    //Print information about controller settings
    print_settings_info();

    //Exit the program if the "Stop Motion" mode is selected
    if(desired_control_mode_.interface == control_mode::STOP_MOTION){
        std::cout << "Stop Motion mode selected. Exiting the program" << std::endl;
        return -1;
    } 

    // double loop_time = 0.0;
    int count = 0;

    safe_control_mode_ = desired_control_mode_.interface;
    std::cout << "Control Loop Started"<< std::endl;
    while(1)
    {   
        // count++;
        // std::cout << "Loop Count: "<< count << std::endl;

        // Save current time point
        loop_start_time_ = std::chrono::steady_clock::now();

        // Check if the task specification is changed. Update accordingly.
        update_task();

        /* 
            Get sensor data from the robot driver or  if simulation is on, 
            replace current state with integrated joint velocities and positions
        */
        update_current_state();

        // Calculate robot dynamics using the Vereshchagin HD solver
        if(evaluate_dynamics() != 0)
        {
            stop_robot_motion();
            std::cerr << "WARNING: Dynamics Solver returned error. "
                      << "Stopping the robot!" << std::endl;
            return -1;
        } 

        // Currently not implemented. Method definition is empty.
        make_predictions(integration_method::SYMPLECTIC_EULER);

        // Apply joint commands using safe control interface.
        // If the computed commands are not safe, exit the program.
        if(apply_control_commands() != 0) return -1;

        // Make sure that the loop is always running with the same frequency
        if(!enforce_loop_frequency() == 0)
            std::cerr << "WARNING: Control loop runs too slow \n" << std::endl;

        // loop_time += std::chrono::duration<double, std::micro>\
        //             (std::chrono::steady_clock::now() -\
        //                                          loop_start_time_).count();
        // if(count == 1000) {
        //     std::cout << loop_time / 1000.0 <<std::endl;
        //     return 0;
        // }
        // if(count == 250) return 0;
    }
    return 0;
}