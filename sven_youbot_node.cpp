#include <youbot/YouBotManipulator.hpp>
#include <solver_functions.h>
#include <solvers.h>
#include <robots.h>

#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    const int JOINTS = 5;
    const int MILLISECOND = 1000;
    double hw_to_dyn_offset[] = { -2.9496, -1.1344, 2.6354, -1.7890, -2.9234 };

    struct kinematic_chain *robot = &youbot_robot;
    struct solver_state state;
    setup_state(robot, &state);

    assert(JOINTS == robot->number_of_segments);

    youbot::YouBotManipulator arm("youbot-manipulator", "example/config/");

    arm.doJointCommutation();
    arm.calibrateManipulator();

    double candle[] = { 2.9496, 1.1344, -2.6354, 1.7890, 2.9234 };
    std::vector<youbot::JointAngleSetpoint> candle_set_point(JOINTS);
    for (int i = 0; i < JOINTS; i++) candle_set_point[i].angle = candle[i] * radian;
    arm.setJointData(candle_set_point);
    usleep(5000 * MILLISECOND);

    //double static_friction_jp[] = { 1.26, 0.956, 0.486, 0.3, 0.177 };
    //double static_friction[] = { 1.46, 0.956, 0.486, 0.2, 0.1 };
    double static_friction[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    while (true) {
        std::vector<youbot::JointSensedAngle> q(JOINTS);
        std::vector<youbot::JointSensedVelocity> qd(JOINTS);
        arm.getJointData(q);
        arm.getJointData(qd);

        state.fak[0].linear_acceleration.z = 9.81;
        for (int i = 0; i < JOINTS; i++) {
            state.q[i][0] = q[i].angle.value() + hw_to_dyn_offset[i];
            state.qd[i][0] = qd[i].angularVelocity.value();
            state.qdd[i][0] = 0.0;
        }

        //recursive_newton_euler_algorithm(robot, &state);
        solver_algorithm(robot, &state, &grav_comp);

        std::vector<youbot::JointTorqueSetpoint> tau(JOINTS);
        for (int i = 0; i < JOINTS; i++) {
            double t = -state.joint_control_force[i].coordinates[0];

            if (qd[i].angularVelocity.value() < 0.001) t -= 0.8 * static_friction[i];
            if (qd[i].angularVelocity.value() > 0.001) t += 0.8 * static_friction[i];

            tau[i] = t * newton_meter;

            //std::cout << state.joint_control_force[i].coordinates[0] << ", ";
            //std::cout << q[i].angle.value() << ", ";
            //std::cout << state.q[i][0] << ", ";
        }
        //std::cout << std::endl;

        arm.setJointData(tau);

        usleep(MILLISECOND);
    }

    return 0;
}
