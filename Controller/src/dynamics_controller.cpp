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
                            const std::vector<double> joint_position_limits,
                            const std::vector<double> joint_velocity_limits,
                            const std::vector<double> joint_acceleration_limits,
                            const std::vector<double> joint_torque_limits,
                            const std::vector<double> youbot_joint_offsets,
                            const int rate_hz):
        robot_chain_(chain),
        root_acc_(root_acc),
        predictor_(robot_chain_),
        robot_driver_(robot_driver),
        NUMBER_OF_CONSTRAINTS_(6),
        NUMBER_OF_JOINTS_(chain.getNrOfJoints()),
        NUMBER_OF_SEGMENTS_(chain.getNrOfSegments()),
        NUMBER_OF_FRAMES_(NUMBER_OF_SEGMENTS_ + 1),
        hd_solver_(robot_chain_, root_acc_, NUMBER_OF_CONSTRAINTS_),
        robot_state_(NUMBER_OF_JOINTS_, NUMBER_OF_SEGMENTS_, 
                     NUMBER_OF_FRAMES_, NUMBER_OF_CONSTRAINTS_), 
        commands_(robot_state_), 
        desired_state_(robot_state_), 
        predicted_state_(robot_state_), 
        joint_position_limits_(joint_position_limits),
        joint_velocity_limits_(joint_velocity_limits),
        joint_acceleration_limits_(joint_acceleration_limits),
        joint_torque_limits_(joint_torque_limits),
        youbot_joint_offsets_(youbot_joint_offsets),
        rate_hz_(rate_hz),
        // Time period defined in microseconds: 1s = 1 000 000us
        dt_micro_(SECOND / rate_hz)
{
    assert(NUMBER_OF_JOINTS_ == NUMBER_OF_SEGMENTS_);
    assert(("Frequency is too low", 1.0 <= rate_hz_));
    assert(("Frequency is too high", rate_hz_<= 2000.0));
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
    set_external_forces(desired_state_, ff_torque);
}

// Define FeedForward joint torques task - Private Method
void dynamics_controller::set_feadforward_torque(
    state_specification &state, 
    const std::vector<double> ff_torque)
{
    assert(ff_torque.size() == NUMBER_OF_JOINTS_);

    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
        state.feedforward_torque(i) = ff_torque[0];
}

//Calculate robot dynamics - Resolve its motion using Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    return hd_solver_.CartToJnt(robot_state_.q,
                                robot_state_.qd,
                                robot_state_.qdd,
                                robot_state_.ee_unit_constraint_force,
                                robot_state_.ee_acceleration_energy,
                                robot_state_.external_force,
                                robot_state_.feedforward_torque);

    // hd_solver_.get_transformed_link_acceleration(motion_.frame_acceleration);
    // std::cout << "\n \n Frame ACC" << '\n';
    // for (size_t i = 0; i < number_of_segments + 1; i++)
    //     std::cout << motion_.frame_acceleration[i] << '\n';
    // std::cout << std::endl;
    // KDL::JntArray control_torque_Ver(number_of_joints);
    // hd_solver_.get_control_torque(control_torque_Ver);
    // std::cout << "\n" << "Joint torques: " << control_torque_Ver << '\n';
}

//Set velocities of arm's joints to 0 value
void dynamics_controller::stop_motion(const bool is_simulation_environment)
{ 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) commands_.qd(i) = 0;  
    if (!is_simulation_environment) robot_driver_.set_joint_velocities(commands_.qd);
}

void dynamics_controller::update_task()
{ 
    //TODO
}

void dynamics_controller::apply_commands(const bool custom_model_used)
{ 
    // std::cout << "\n Joint Vel::Commanded: ";
    // for (int i = 0; i < JOINTS; i++)
        // std::cout << commands_.qd(i) << " , ";
    // std::cout << "\n";
    if(custom_model_used) commands_.qd(4) = 0;
    // if(custom_model_used) commands_.qd(4) = -1 * commands_.qd(4);
    robot_driver_.set_joint_velocities(commands_.qd);
}

//Get current robot state from the joint sensors
void dynamics_controller::update_current_state(const bool custom_model_used)
{
    robot_driver_.get_joint_positions(robot_state_.q);
    robot_driver_.get_joint_velocities(robot_state_.qd);

    // std::cout << "\n" << "Joint Positions" << robot_state_.q << std::endl;
    // std::cout << "\n" <<"Joint Velocities"<< robot_state_.qd << std::endl;

    // Custom model's home state is not folded - it is candle
    if (custom_model_used){
        for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
            robot_state_.q(i) = robot_state_.q(i) + youbot_joint_offsets_[i];
        robot_state_.qd(4) = -1 * robot_state_.qd(4);
    }
}

int dynamics_controller::enforce_loop_frequency(){
    loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);

    if(loop_interval_ < std::chrono::microseconds(dt_micro_))
    {
        while(loop_interval_.count() < dt_micro_)
            loop_interval_= std::chrono::duration<double, std::micro>\
                    (std::chrono::steady_clock::now() - loop_start_time_);
    } else return -1;

    // loop_interval_= std::chrono::duration<double, std::micro>\
    //                 (std::chrono::steady_clock::now() - loop_start_time_);
    // std::cout << loop_interval_.count() << std::endl;   
    return 0;    
}

int dynamics_controller::control(const bool is_simulation_environment,
                                 const bool custom_model_used)
{
    stop_motion(is_simulation_environment);

    int solver_result;
    std::cout << "Control Loop Started"<< std::endl;

    while(1)
    {
        loop_start_time_ = std::chrono::steady_clock::now();

        update_task();

        if (!is_simulation_environment) update_current_state(custom_model_used);

        solver_result = evaluate_dynamics();
        assert(solver_result == 0);

        make_predictions();

        assert(("Loop too slow", enforce_loop_frequency() == 0));

        if (!is_simulation_environment) apply_commands(custom_model_used);
    }
    return 0;
}