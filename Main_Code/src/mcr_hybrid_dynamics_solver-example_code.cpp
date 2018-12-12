/*
Author(s): Djordje Vukcevic, Sven Schneider
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

#include <hd_solver_vereshchagin.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <stdlib.h>     /* abs */

//Generic cpp code for using HD solver
const int NUMBER_OF_CONSTRAINTS = 1;

class extended_kinematic_chain
{
    public:
        KDL::Chain chain;
        std::vector<double> joint_inertia;
        std::vector<double> joint_static_friction;
};

class motion_specification
{
    public:
        motion_specification(
                int number_of_joints,
                int number_of_segments,
                int number_of_constraints)
            : q(number_of_joints),
              qd(number_of_joints),
              qdd(number_of_joints),
              feedforward_torque(number_of_joints),  //Initial Q without friction
              end_effector_unit_constraint_forces(number_of_constraints), //alpha
              end_effector_acceleration_energy_setpoint(number_of_constraints), //beta
              external_force(number_of_segments) //U in Vereshchagin 1989 paper
              {

              }

        KDL::JntArray q;
        KDL::JntArray qd;
        KDL::JntArray qdd; // as input to original solver, overwritten during call!
        KDL::JntArray feedforward_torque;
        KDL::Jacobian end_effector_unit_constraint_forces;
        KDL::JntArray end_effector_acceleration_energy_setpoint;
        KDL::Wrenches external_force;
};

// void create_my_5DOF_robot(extended_kinematic_chain &c)
// {
//     //Extract KDL tree from URDF file
//     KDL::Tree yb_tree;
//     urdf::Model yb_model;
//
//     if (!yb_model.initFile("../urdf/youbot_arm_only.urdf")){
//         std::cout << "ERROR: Failed to parse urdf robot model" << '\n';
//         return 0;
//     }
//
//     if (!kdl_parser::treeFromUrdfModel(yb_model, yb_tree)){
//         std::cout << "ERROR: Failed to construct kdl tree" << '\n';
//         return 0;
//     }
//
//     //Extract KDL chain from KDL tree
//     yb_tree.getChain("arm_link_0", "arm_link_5", c.chain);
//
//     int number_of_joints = c.chain.getNrOfJoints();
//
//     c.joint_inertia.resize(number_of_joints);
//     for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.3;
//
//     c.joint_static_friction.resize(number_of_joints);
//     // for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 5.0;
//     c.joint_static_friction = {1.260, 0.956, 0.486, 0.300, 0.177};
//
//     //In total 5 joints (NOT counting fixed - 0), 5 segments (NOT counting base link 0) and 6 frames
// }

void create_my_2DOF_robot(extended_kinematic_chain &c)
{
    int number_of_joints = 2;
    c.joint_inertia.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_inertia[i] = 0.0;

    c.joint_static_friction.resize(number_of_joints);
    for (int i = 0; i < number_of_joints; i++) c.joint_static_friction[i] = 10.0;

    //2 joints, 2 segments
    for (int i = 0; i < number_of_joints; i++) {

        //last 3 inputs...input_scale, offset and joint inertia (d in paper)
        KDL::Joint joint = KDL::Joint(KDL::Joint::RotZ, 1, 0, c.joint_inertia[i]);

        // RPY(roll,pitch,yaw) Rotation built from Roll-Pitch-Yaw angles
        KDL::Frame tip(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 1, 0.0));

        //Frames desctibe pose of the segment tip, wrt joint frame
        KDL::Segment segment = KDL::Segment(joint, tip);

        //rotational inertia around symmetry axis of rotation
        KDL::RotationalInertia rotational_inertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        //spatial inertia
        // center of mass at the same position as tip  of segment???
        KDL::RigidBodyInertia inertia(1, KDL::Vector(0.0, 0.5, 0.0), rotational_inertia);

        segment.setInertia(inertia);

        //adding segments in chain
        c.chain.addSegment(segment);
    }
}

void create_my_motion_specification(motion_specification &m)
{
    m.q(0) = M_PI / 2.0;
    m.q(1) = M_PI / 6.0;

    m.qd(0) = 0.0;
    m.qd(1) = 0.0;

    m.feedforward_torque(0) = 0.0;
    m.feedforward_torque(1) = 0.0;

    KDL::Wrench externalForce1(
        KDL::Vector(0.0, 0.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0)); //Torque
    m.external_force[0] = externalForce1;

    KDL::Wrench externalForce2(
        KDL::Vector(0.0, 0.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0)); //Torque
    m.external_force[1] = externalForce2;

    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 1.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;
    //
    // KDL::Twist unit_constraint_force_y(
    //         KDL::Vector(0.0, 1.0, 0.0),     // linear
    //         KDL::Vector(0.0, 0.0, 0.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    // m.end_effector_acceleration_energy_setpoint(1) = 0.0;
}

void create_5DOF_motion_specification(motion_specification &m)
{
    int number_of_joints = m.q.rows();
    int number_of_segments = m.external_force.size();

    m.q(0) = M_PI / 2.0;
    // m.q(0) = 0.0;
    m.q(1) = M_PI / 2.0;
    // m.q(1) = 0.0;
    m.q(2) = M_PI / 1.0;
    m.q(3) = -M_PI / 2.0;
    m.q(4) = -M_PI / 3.0;
    m.q(5) = M_PI / 4.0;

    // for (int i = 0; i < number_of_joints; i++) m.q(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.qdd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) m.feedforward_torque(i) = 0.0;

    // external forces on the arm
    for (int i = 0; i < number_of_segments - 1; i++) {
        KDL::Wrench externalForce(
            KDL::Vector(0.0, 0.0, 0.0), //Force
            KDL::Vector(0.0, 0.0, 0.0)); //Torque
        m.external_force[i] = externalForce;
    }

    KDL::Wrench externalForceEE(
        KDL::Vector(0.0, 0.0, 0.0), //Force
        KDL::Vector(0.0, 0.0, 0.0));//Torque
    m.external_force[number_of_segments - 1] = externalForceEE;

    //gravity compensation
    KDL::Twist unit_constraint_force_x(
            KDL::Vector(1.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    m.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    m.end_effector_acceleration_energy_setpoint(0) = 0.0;
    //
    // KDL::Twist unit_constraint_force_y(
    //         KDL::Vector(0.0, 1.0, 0.0),     // linear
    //         KDL::Vector(0.0, 0.0, 0.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    // m.end_effector_acceleration_energy_setpoint(1) = 0.0;
    //
    // KDL::Twist unit_constraint_force_z(
    //         KDL::Vector(0.0, 0.0, 1.0),     // linear
    //         KDL::Vector(0.0, 0.0, 0.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_z);
    // m.end_effector_acceleration_energy_setpoint(2) = 0.0;
    // //
    // KDL::Twist unit_constraint_force_x1(
    //         KDL::Vector(0.0, 0.0, 0.0),     // linear
    //         KDL::Vector(1.0, 0.0, 0.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_x1);
    // m.end_effector_acceleration_energy_setpoint(3) = 0.0;
    //
    // KDL::Twist unit_constraint_force_y1(
    //         KDL::Vector(0.0, 0.0, 0.0),     // linear
    //         KDL::Vector(0.0, 1.0, 0.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_y1);
    // m.end_effector_acceleration_energy_setpoint(4) = 0.0;
    //
    // KDL::Twist unit_constraint_force_z1(
    //         KDL::Vector(0.0, 0.0, 0.0),     // linear
    //         KDL::Vector(0.0, 0.0, 1.0));    // angular
    // m.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_z1);
    // m.end_effector_acceleration_energy_setpoint(5) = 0.0;
}

int main(int argc, char* argv[])
{
    extended_kinematic_chain my_robot;
    create_my_2DOF_robot(my_robot);

    motion_specification my_motion(my_robot.chain.getNrOfJoints(),
                                   my_robot.chain.getNrOfSegments(),
                                   NUMBER_OF_CONSTRAINTS);

    create_my_motion_specification(my_motion);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    KDL::Solver_Vereshchagin solver_(my_robot.chain,
                                    root_acc,
                                    NUMBER_OF_CONSTRAINTS);

    int result = solver_.CartToJnt(
                 my_motion.q,
                 my_motion.qd,
                 my_motion.qdd, //qdd_ is overwritten by resulting acceleration
                 my_motion.end_effector_unit_constraint_forces,       // alpha
                 my_motion.end_effector_acceleration_energy_setpoint, // beta
                 my_motion.external_force,
                 my_motion.feedforward_torque);
    assert(result == 0);
    std::cout << "Solver return: "<< result << '\n';

    std::vector<KDL::Twist> frame_acceleration_;
    std::vector<KDL::ArticulatedBodyInertia> articulated_body_inertia_;
    KDL::Wrenches bias_force_;

    int number_of_frames_ = my_robot.chain.getNrOfSegments()+1;
    frame_acceleration_.resize(number_of_frames_);
    articulated_body_inertia_.resize(number_of_frames_);
    bias_force_.resize(number_of_frames_);

    solver_.get_transformed_link_acceleration(frame_acceleration_);
    solver_.get_link_inertias(articulated_body_inertia_);
    solver_.get_bias_force(bias_force_);

    std::cout << "Frame ACC" << '\n';
    for (size_t i = 0; i < number_of_frames_; i++) {
        std::cout << frame_acceleration_[i] << '\n';
    }

    std::cout << "Joint accelerations:" << '\n';
    std::cout << my_motion.qdd << '\n';


    // extended_kinematic_chain five_DOF_robot;
    // create_my_5DOF_robot(five_DOF_robot);
    // motion_specification five_DOF_motion(five_DOF_robot.chain.getNrOfJoints(), five_DOF_robot.chain.getNrOfSegments(), NUMBER_OF_CONSTRAINTS);

	return 0;
}
