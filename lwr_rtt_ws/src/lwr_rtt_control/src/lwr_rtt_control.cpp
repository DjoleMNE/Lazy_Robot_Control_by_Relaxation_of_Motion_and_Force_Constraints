#include "lwr_rtt_control/lwr_rtt_control.hpp"

LwrRttControl::LwrRttControl(const std::string& name):
RTT::TaskContext(name)
{
    // Here you can add your ports, properties and operations
    // ex : this->addOperation("my_super_function",&LwrRttControl::MyFunction,this,RTT::OwnThread);
    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");

}

bool LwrRttControl::configureHook()
{
    // Initialize the arm object
    if(!this->arm.init())
    {
        RTT::log(RTT::Fatal)
        << "Could not initialize arm, make sure roscore is launched"
        " and that tip_link, root_link and robot_description are set in rosparam"
        << RTT::endlog();
    }

    jnt_pos_in.setZero(arm.getNrOfJoints());
    jnt_vel_in.setZero(arm.getNrOfJoints());
    jnt_trq_in.setZero(arm.getNrOfJoints());

    jnt_pos_cmd_out.setZero(arm.getNrOfJoints());
    jnt_vel_cmd_out.setZero(arm.getNrOfJoints());
    jnt_trq_cmd_out.setZero(arm.getNrOfJoints());

    port_joint_position_cmd_out.setDataSample(jnt_pos_cmd_out);
    port_joint_velocity_cmd_out.setDataSample(jnt_vel_cmd_out);
    port_joint_torque_cmd_out.setDataSample(jnt_trq_cmd_out);

    // Check validity of (all) Ports:
    if ( !port_joint_position_in.connected() || 
         !port_joint_velocity_in.connected() ||
         !port_joint_torque_in.connected() ) 
    {
        RTT::log(RTT::Fatal) << "No input connection!"<< RTT::endlog();
        return false;
    }
    if ( !port_joint_position_cmd_out.connected() ||
         !port_joint_torque_cmd_out.connected()) 
    {
           RTT::log(RTT::Warning) << "No output connection!"<< RTT::endlog();  
    }
    return true;
}

void LwrRttControl::updateHook()
{
    // Read status from robot
    port_joint_position_in.read(jnt_pos_in);
    port_joint_velocity_in.read(jnt_vel_in);
    // std::cout << jnt_pos_in.transpose() << std::endl;
    // std::cout << jnt_vel_in.transpose() << std::endl;
    if(!port_joint_position_cmd_out.connected())
    {
        RTT::log(RTT::Warning) << "No position output connection!"<< RTT::endlog();  
    }

    if (!port_joint_torque_cmd_out.connected()) 
    {
           RTT::log(RTT::Warning) << "No torque output connection!"<< RTT::endlog();  
    }
    // jnt_pos_cmd_out(0) = 0.3813;
    // jnt_pos_cmd_out(1) = -1.9312;
    // jnt_pos_cmd_out(2) = -1.7251;
    // jnt_pos_cmd_out(3) = -1.4565;
    // jnt_pos_cmd_out(4) = 0.7169;
    // jnt_pos_cmd_out(5) = 1.056;
    // jnt_pos_cmd_out(6) = -2.123; 
    // port_joint_position_cmd_out.write(jnt_pos_cmd_out);

    jnt_trq_cmd_out(0) = 1.03813;
    jnt_trq_cmd_out(1) = 0.0;
    jnt_trq_cmd_out(2) = 0.0;
    jnt_trq_cmd_out(3) = 0.0;
    jnt_trq_cmd_out(4) = 0.0;
    jnt_trq_cmd_out(5) = 0.0;
    jnt_trq_cmd_out(6) = 0.0; 
    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(LwrRttControl)
