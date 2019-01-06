/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2018]

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

dynamics_controller::dynamics_controller(
                            youbot_mediator &robot_driver,
                            const KDL::Chain &chain,
                            const KDL::Twist &root_acc,
                            const std::vector<double> joint_position_limits_p,
                            const std::vector<double> joint_position_limits_n,
                            const std::vector<double> joint_velocity_limits,
                            const std::vector<double> joint_acceleration_limits,
                            const std::vector<double> joint_torque_limits,
                            const std::vector<double> rotor_inertia,
                            const int rate_hz):
        robot_chain_(chain),
        root_acc_(root_acc),
        NUMBER_OF_CONSTRAINTS_(6),
        NUMBER_OF_JOINTS_(chain.getNrOfJoints()),
        NUMBER_OF_SEGMENTS_(chain.getNrOfSegments()),
        NUMBER_OF_FRAMES_(chain.getNrOfSegments() + 1),
        hd_solver_(robot_chain_, root_acc_, NUMBER_OF_CONSTRAINTS_),
        robot_state_(chain.getNrOfJoints(), chain.getNrOfSegments(), 
                     chain.getNrOfSegments() + 1, NUMBER_OF_CONSTRAINTS_), 
        commands_(robot_state_), 
        desired_state_(robot_state_), 
        predicted_state_(robot_state_), 
        predictor_(robot_chain_),
        robot_driver_(robot_driver),
        safety_control_(robot_chain_, joint_position_limits_p,
                        joint_position_limits_n, joint_velocity_limits, 
                        joint_acceleration_limits, joint_torque_limits, true),
        //Resize and set vector's elements to zero 
        zero_joint_velocities_(chain.getNrOfJoints()),
        solver_result_(0),
        safe_control_mode_(-1),
        rate_hz_(rate_hz),
        // Time period defined in microseconds: 1s = 1 000 000us
        dt_micro_(SECOND / rate_hz),
        dt_sec_(1.0 / static_cast<double>(rate_hz)) 
{
    assert(NUMBER_OF_JOINTS_ == NUMBER_OF_SEGMENTS_);

    // Control loop frequency must be higher than or equal to 1 Hz
    assert(("Desired frequency is too low", 1 <= rate_hz_));
    // Control loop frequency must be lower than or equal to 1000 Hz
    assert(("Desired frequency is too high", rate_hz_<= 10000));

    // Set vector of joint rotor inertia: term "d" in the algorithm
    hd_solver_.set_rotor_inertia(rotor_inertia);
    
    // Set default command interface to velocity mode and initialize it as safe
    desired_control_mode_.interface = control_mode::stop_motion;
    desired_control_mode_.is_safe = false;
}

// Set all values of desired state to 0 - public method
void dynamics_controller::reset_desired_state(){
    reset_state(desired_state_);
}

// Set all values of selected state to 0 - Private method
void dynamics_controller::reset_state(state_specification &state){
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

//Calculate robot dynamics - Resolve its motion using Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    solver_result_= hd_solver_.CartToJnt(robot_state_.q,
                                         robot_state_.qd,
                                         robot_state_.qdd,
                                         robot_state_.ee_unit_constraint_force,
                                         robot_state_.ee_acceleration_energy,
                                         robot_state_.external_force,
                                         robot_state_.feedforward_torque);

    hd_solver_.get_transformed_link_pose(robot_state_.frame_pose);
    hd_solver_.get_transformed_link_velocity(robot_state_.frame_velocity);
    hd_solver_.get_transformed_link_acceleration(robot_state_.frame_acceleration);
    hd_solver_.get_control_torque(robot_state_.control_torque);
    
    // Print Cartesian state in Debug mode
    #if DEBUG == 0
        std::cout << "End-effector Position: " 
                << robot_state_.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
                << std::endl;
        std::cout << "End-effector Velocity: " 
            << robot_state_.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
            << std::endl;    
        std::cout << "Frame ACC" << '\n';
        for (size_t i = 0; i < NUMBER_OF_SEGMENTS_ + 1; i++)
            std::cout << robot_state_.frame_acceleration[i] << '\n';
        std::cout << std::endl;
    #endif

    return solver_result_;
}

//Make sure that the control loop runs exactly with specified frequency
int dynamics_controller::enforce_loop_frequency()
{
    loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);

    if(loop_interval_ < std::chrono::microseconds(dt_micro_))
    {
        while(loop_interval_.count() < dt_micro_)
            loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);
    } else return -1; 
    return 0;    
}

//Print information about controller settings
void dynamics_controller::print_settings_info()
{   std::cout << "Selected controller settings:" << std::endl;
    std::cout << "Control Loop Frequency: " << rate_hz_ << " Hz" << std::endl;
    std::cout << "Control Mode: ";

    switch(desired_control_mode_.interface) 
    {
        case control_mode::stop_motion:
            std::cout << "STOP MOTION \n" << "Stopping the robot!" << std::endl;
            break;

        case control_mode::velocity_control:
            std::cout << "Velocity Control" << std::endl;
            break;

        case control_mode::position_control:
            std::cout << "Position Control" << std::endl;
            break;

        case control_mode::torque_control:
            std::cout << "Torque Control" << std::endl;
            break;
    }
    
    std::cout<<"\nInitial joint state:"<< std::endl;
    std::cout<< "Joint velocities:"<< robot_state_.qd << std::endl;
    std::cout<< "Joint positions: "<< robot_state_.q << std::endl;
    std::cout<<"\n";
}

//Set velocities of arm's joints to 0 value
void dynamics_controller::stop_robot_motion()
{   
    // Send commands to the robot driver
    robot_driver_.set_joint_velocities(zero_joint_velocities_);
}

// Predict future robot states (motion) based given the current state
void dynamics_controller::make_predictions(const int prediction_method)
{
    predictor_.integrate_cartesian_space(robot_state_, 
                                         predicted_state_, 
                                         dt_sec_, 1, 
                                         prediction_method);
}

// Update the desired robot state 
void dynamics_controller::update_task()
{ 
    /* 
        TODO: This component should update desired state in case of a user 
        changing task specification in parallel (while control loop in this 
        component is running). Maybe something like a callback function.
    */
}

//Send joint commands to the robot driver
void dynamics_controller::apply_control_commands()
{ 
    //TODO - maybe not required
}

//Get current robot state from the joint sensors
void dynamics_controller::update_current_state(const bool simulation_environment)
{
    if(!simulation_environment){
        robot_driver_.get_joint_positions(robot_state_.q);
        robot_driver_.get_joint_velocities(robot_state_.qd);
    } else{
        robot_state_.qd = commands_.qd;
        robot_state_.q = commands_.q;
    }
}

//Main control loop
int dynamics_controller::control(const bool simulation_environment,
                                 const int desired_control_mode)
{   
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;

    //Print information about controller settings
    print_settings_info();

    if(!simulation_environment)
    {   
        // Check if the robot is initialied and connection established
        assert(("Robot is not initialized", robot_driver_.is_initialized));   

        // Make sure that robot is not moving at the start 
        // Command zero velocities to the robot driver
        stop_robot_motion();
    }

    //Exit the program if the "Stop Motion" mode is selected
    if(desired_control_mode_.interface == control_mode::stop_motion) return -1;

    safe_control_mode_ = desired_control_mode_.interface;
    std::cout << "Control Loop Started"<< std::endl;
    while(1)
    {   
        // Save current time point
        loop_start_time_ = std::chrono::steady_clock::now();

        // Check if the task specification is changed
        update_task();

        /* 
            Get sensor data from the robot driver or
            if simulation is on, replace current state with 
            the integrated joint velocities and positions
        */
        update_current_state(simulation_environment);


        // Calculate robot dynamics using Vereshchagin HD solver
        if(evaluate_dynamics() != 0){
            if(!simulation_environment) stop_robot_motion();
            std::cerr << "WARNING: Dynamics Solver returned error. "
                      << "Current commands are not safe. " 
                      << "Stopping the robot!" << std::endl;
            return -1;
        } 

        // Predict future robot states (motion) based given the current state
        // I.e. Integrate Cartesian variables
        // make_predictions(integration_method::symplectic_euler);

        /* 
            All calculations done - check if the commands are over the limits
            If yes: use desired control mode
            Else: use the control mode selected by the safety controller 
        */
        safe_control_mode_ = safety_control_.generate_commands(
                                        robot_state_, 
                                        commands_, dt_sec_, 
                                        desired_control_mode_.interface,
                                        integration_method::symplectic_euler);

        desired_control_mode_.is_safe =\
            (desired_control_mode_.interface == safe_control_mode_)? true : false; 

        switch(safe_control_mode_) {
            case control_mode::torque_control:
                assert(desired_control_mode_.is_safe);
                if (!simulation_environment)
                    robot_driver_.set_joint_torques(commands_.control_torque);
                break;

            case control_mode::velocity_control:
                if (!simulation_environment) 
                    robot_driver_.set_joint_velocities(commands_.qd);
                if(!desired_control_mode_.is_safe) 
                    cout << "WARNING: Control switched to velocity mode \n" << endl;
                break;

            case control_mode::position_control:
                if (!simulation_environment)
                    robot_driver_.set_joint_positions(commands_.q);
                if(!desired_control_mode_.is_safe) 
                    cout << "WARNING: Control switched to position mode \n" << endl;
                break;

            default:
                if (!simulation_environment) stop_robot_motion();
                cout << "WARNING: Current commands are not safe. " 
                     << "Stopping the robot!" << endl;
                return -1;             
        }

        // Make sure that the loop is always running with the same frequency
        if(!enforce_loop_frequency() == 0) 
            std::cerr << "WARNING: Control loop runs too slow \n" << std::endl;
    }
    return 0;
}