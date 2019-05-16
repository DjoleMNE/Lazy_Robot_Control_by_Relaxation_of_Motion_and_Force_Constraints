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
#include "lwr_rtt_control/lwr_rtt_control.hpp"
const long MILLISECOND = 1000;
const long SECOND = 1000000;

LwrRttControl::LwrRttControl(const std::string& name):
    RTT::TaskContext(name), RATE_HZ_(999), NUM_OF_SEGMENTS_(7), 
    NUM_OF_JOINTS_(7), NUM_OF_CONSTRAINTS_(6), 
    environment_(lwr_environment::LWR_SIMULATION), 
    robot_model_(lwr_model::LWR_URDF), iteration_count_(0),
    loop_total_time_(0.0),
    krc_compensate_gravity_(false), use_transformed_driver_(true),
    desired_task_model_(2), desired_control_mode_(0), desired_dynamics_interface_(1),
    desired_pose_(1), damper_amplitude_(1.0), damper_slope_(4.0), tube_speed_(0.2),
    control_dims_(NUM_OF_CONSTRAINTS_, false),
    desired_ee_pose_(12, 0.0), tube_tolerances_(6, 0.0), tube_start_position_(3, 0.0),
    max_command_(Eigen::VectorXd::Constant(6, 0.0)),
    error_alpha_(Eigen::VectorXd::Constant(6, 0.0)),
    bias_threshold_(Eigen::VectorXd::Constant(6, 0.0)),
    bias_step_(Eigen::VectorXd::Constant(6, 0.0)),
    gain_threshold_(Eigen::VectorXd::Constant(6, 0.0)),
    gain_step_(Eigen::VectorXd::Constant(6, 0.0)),
    abag_error_type_(1), 
    min_bias_sat_(Eigen::VectorXd::Constant(6, -1.0)), 
    min_command_sat_(Eigen::VectorXd::Constant(6, -1.0)),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_SEGMENTS_ + 1, NUM_OF_CONSTRAINTS_)
{
    // Here you can add your ports, properties and operations
    // ex : this->addOperation("my_super_function",&LwrRttControl::MyFunction,this,RTT::OwnThread);
    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");

//     this->addProperty("environment", environment_).doc("environment");
//     this->addProperty("robot_model", robot_model_).doc("robot_model");
    this->addProperty("krc_compensate_gravity", krc_compensate_gravity_).doc("KRC compensate gravity");
    this->addProperty("use_transformed_driver", use_transformed_driver_).doc("use_transformed_driver");
    this->addProperty("desired_task_model", desired_task_model_).doc("desired_task_model");
    this->addProperty("desired_control_mode", desired_control_mode_).doc("desired_control_mode");
    this->addProperty("desired_dynamics_interface", desired_dynamics_interface_).doc("desired_dynamics_interface");
    this->addProperty("desired_pose", desired_pose_).doc("desired pose");
    this->addProperty("tube_tolerances", tube_tolerances_).doc("tube_tolerances");
    this->addProperty("tube_start_position", tube_start_position_).doc("tube_start_position");
    this->addProperty("tube_speed", tube_speed_).doc("tube_speed");
    this->addProperty("control_dims", control_dims_).doc("control dimensions");
    this->addProperty("damper_amplitude", damper_amplitude_).doc("damper_amplitude");
    this->addProperty("damper_slope", damper_slope_).doc("damper_slope");
    this->addProperty("max_command", max_command_).doc("max_command");
    this->addProperty("ERROR_ALPHA", error_alpha_).doc("ABAG ERROR_ALPHA");
    this->addProperty("BIAS_THRESHOLD", bias_threshold_).doc("BIAS_THRESHOLD");
    this->addProperty("BIAS_STEP", bias_step_).doc("BIAS_STEP");
    this->addProperty("GAIN_THRESHOLD", gain_threshold_).doc("GAIN_THRESHOLD");
    this->addProperty("GAIN_STEP", gain_step_).doc("GAIN_STEP");
    this->addProperty("abag_error_type", abag_error_type_).doc("abag_error_type");
    this->addProperty("min_bias_sat", min_bias_sat_).doc("min_bias_sat");
    this->addProperty("min_command_sat", min_command_sat_).doc("min_command_sat");
}

bool LwrRttControl::configureHook()
{
    rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);

    jnt_pos_in.setZero(NUM_OF_JOINTS_);
    jnt_vel_in.setZero(NUM_OF_JOINTS_);
    jnt_trq_in.setZero(NUM_OF_JOINTS_);

    jnt_pos_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_vel_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_trq_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_gravity_trq_out.data.setZero(NUM_OF_JOINTS_);

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

    robot_driver_.initialize(robot_model_, environment_, krc_compensate_gravity_);
    assert(NUM_OF_JOINTS_ == robot_driver_.get_robot_model().getNrOfSegments());

    this->gravity_solver_ = std::make_shared<KDL::ChainDynParam>(robot_driver_.get_robot_model(), 
                                                                 KDL::Vector(0.0, 0.0, -9.81289)); 

    this->controller_ = std::make_shared<dynamics_controller>(&robot_driver_, RATE_HZ_);

    //Create End_effector Cartesian Acceleration task 
    controller_->define_ee_acc_constraint(std::vector<bool>{false, false, false, // Linear
                                                            false, false, false}, // Angular
                                          std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                              0.0, 0.0, 0.0}); // Angular
    //Create External Forces task 
    controller_->define_ee_external_force(std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                              0.0, 0.0, 0.0}); // Angular
    //Create Feedforward torques task s
    controller_->define_feedforward_torque(std::vector<double>{0.0, 0.0, 
                                                               0.0, 0.0, 
                                                               0.0, 0.0, 0.0});
    switch (desired_pose_)
    {
        case desired_pose::CANDLE:
            // Candle Pose
            desired_ee_pose_ = { 0.0,  0.0, 1.175, // Linear: Vector
                                 1.0,  0.0, 0.0, // Angular: Rotation matrix
                                 0.0,  1.0, 0.0,
                                 0.0,  0.0, 1.0};
            break;

        case desired_pose::FOLDED:
            // Folded pose
            desired_ee_pose_ = { 0.260912, -0.014731,  0.0945801, // Linear: Vector
                                 0.575147,  0.789481, -0.214301, // Angular: Rotation matrix
                                 0.174954,  0.137195,  0.974971,
                                 0.799122, -0.598245, -0.059216};
            break;

        case desired_pose::NAVIGATION_2:
            // Navigation pose 2
            desired_ee_pose_ = { 0.2,  0.3, 0.632811, // Linear: Vector
                                 1.0,  0.0, 0.0, // Angular: Rotation matrix
                                 0.0,  1.0, 0.0,
                                 0.0,  0.0, 1.0};
            break;

        default:
            // Navigation pose 1
            desired_ee_pose_ = {-0.200785, -0.308278,  0.632811, // Linear: Vector
                                -0.540302, -0.841471, -0.000860, // Angular: Rotation matrix
                                -0.841470,  0.540302, -0.001340,
                                 0.001592,  0.000000, -0.999999};
            break;
    }

    switch (desired_task_model_)
    {
        case task_model::moveTo:
            controller_->define_moveTo_task(std::vector<bool>{control_dims_[0], control_dims_[1], control_dims_[2], // Linear
                                                              control_dims_[3], control_dims_[4], control_dims_[5]},// Angular
                                    tube_start_position_,
                                    tube_tolerances_,
                                    tube_speed_,
                                    0.1, 0.1, //contact_threshold linear and angular
                                    15.0,// time_limit
                                    desired_ee_pose_); // TF pose
            break;

        case task_model::full_pose:
            controller_->define_desired_ee_pose(std::vector<bool>{control_dims_[0], control_dims_[1], control_dims_[2], // Linear
                                                                  control_dims_[3], control_dims_[4], control_dims_[5]}, // Angular
                                                desired_ee_pose_);
            break;

        default:
            assert(("Unsupported task model", false));
            break;
    }
 

    controller_->set_parameters(damper_amplitude_, damper_slope_, 
                                abag_error_type_, max_command_, error_alpha_,
                                bias_threshold_, bias_step_, gain_threshold_,
                                gain_step_, min_bias_sat_, min_command_sat_);

    controller_->initialize(desired_control_mode_, 
                            desired_dynamics_interface_,
                            use_transformed_driver_, 
                            true);

    // sleep(2); // wait for gazebo to load completely        
    this->visualize_pose(desired_ee_pose_);
    return true;
}


void LwrRttControl::updateHook()
{
    if(iteration_count_ > 12000) 
    {
        // std::cout << "Loop time: " << loop_total_time_ / iteration_count_ << std::endl;
        RTT::TaskContext::stop();
    }

    // Save current time point
    // loop_start_time_ = std::chrono::steady_clock::now();

    // Read status from robot
    port_joint_position_in.read(jnt_pos_in);
    port_joint_velocity_in.read(jnt_vel_in);
    port_joint_torque_in.read(jnt_trq_in);

    robot_state_.q.data  = jnt_pos_in;
    robot_state_.qd.data = jnt_vel_in;
    
    int controller_result = controller_->step(robot_state_.q, 
                                              robot_state_.qd, 
                                              robot_state_.control_torque.data);

    if(!controller_result == 0) RTT::TaskContext::stop();
    //  RTT::TaskContext::stop();

    if(krc_compensate_gravity_) jnt_trq_cmd_out = robot_state_.control_torque.data;
    else
    {
        gravity_solver_->JntToGravity(robot_state_.q, jnt_gravity_trq_out);
        jnt_trq_cmd_out = robot_state_.control_torque.data - jnt_gravity_trq_out.data;
    }

    // jnt_trq_cmd_out << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
    iteration_count_++;

    // loop_total_time_ += std::chrono::duration<double, std::micro>\
    //         (std::chrono::steady_clock::now() -\
    //                                         loop_start_time_).count();
}

void LwrRttControl::stopHook()
{
    controller_->deinitialize();
    RTT::log(RTT::Error) << "Robot stopped!" << RTT::endlog();
}

void LwrRttControl::visualize_pose(const std::vector<double> &pose)
{
    // Create Temp ROS Node
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "lwr_control", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    } this->rosnode_ = std::make_shared<ros::NodeHandle>();

    // Create transform message
    static_transformStamped_.header.stamp = ros::Time::now();
    static_transformStamped_.header.frame_id = "link_0";
    static_transformStamped_.child_frame_id = "desired_pose";

    // Convert data
    // Linear part
    static_transformStamped_.transform.translation.x = pose[0];
    static_transformStamped_.transform.translation.y = pose[1];
    static_transformStamped_.transform.translation.z = pose[2];
    
    tf2::Matrix3x3 desired_matrix = tf2::Matrix3x3(pose[3], pose[4],  pose[5], // Angular: Rotation matrix
                                                   pose[6], pose[7],  pose[8],
                                                   pose[9], pose[10], pose[11]);

    tf2::Quaternion quaternion_rotation;
    desired_matrix.getRotation(quaternion_rotation);
    quaternion_rotation.normalize();

    static_transformStamped_.transform.rotation.x = quaternion_rotation[0];
    static_transformStamped_.transform.rotation.y = quaternion_rotation[1];
    static_transformStamped_.transform.rotation.z = quaternion_rotation[2];
    static_transformStamped_.transform.rotation.w = quaternion_rotation[3];

    // Publish once
    static_broadcaster_.sendTransform(static_transformStamped_);

    if(desired_task_model_ == task_model::moveTo || desired_task_model_ == task_model::moveGuarded)
    {
        ros::NodeHandle handle;
        ros::Publisher vis_pub = handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1);
        sleep(2);

        visualization_msgs::Marker marker_1, marker_2;
        marker_1.header.frame_id = "link_0";
        marker_1.header.stamp = ros::Time::now();
        marker_1.ns = "tube";
        marker_1.id = 0;
        marker_1.type = visualization_msgs::Marker::CYLINDER;
        marker_1.action = visualization_msgs::Marker::ADD;
        marker_1.pose.position.x = pose[0];
        marker_1.pose.position.y = pose[1];
        marker_1.pose.position.z = pose[2];

        desired_matrix = desired_matrix * tf2::Matrix3x3(0.0, 0.0, -1.0, // Rotate frame: Y axis -90deg
                                                         0.0, 1.0,  0.0,
                                                         1.0, 0.0,  0.0);
        desired_matrix.getRotation(quaternion_rotation);
        quaternion_rotation.normalize();
        marker_1.pose.orientation.x = quaternion_rotation[0];
        marker_1.pose.orientation.y = quaternion_rotation[1];
        marker_1.pose.orientation.z = quaternion_rotation[2];
        marker_1.pose.orientation.w = quaternion_rotation[3];

        marker_1.scale.x = tube_tolerances_[1] * 2;
        marker_1.scale.y = tube_tolerances_[2] * 2;
        marker_1.scale.z = 1.5;
        marker_1.color.a = 0.2; // Don't forget to set the alpha!
        marker_1.color.r = 255.0;
        marker_1.color.g = 255.0;
        marker_1.color.b = 255.0;
        //only if using a MESH_RESOURCE marker type:
        marker_1.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_1.lifetime = ros::Duration(1000);

        // Publish the marker
        vis_pub.publish(marker_1);

        marker_2.header.frame_id = "link_0";
        marker_2.header.stamp = ros::Time::now();
        marker_2.ns = "goal_area";
        marker_2.id = 0;
        marker_2.type = visualization_msgs::Marker::CYLINDER;
        marker_2.action = visualization_msgs::Marker::ADD;
        marker_2.pose.position.x = pose[0];
        marker_2.pose.position.y = pose[1];
        marker_2.pose.position.z = pose[2];

        marker_2.pose.orientation.x = quaternion_rotation[0];
        marker_2.pose.orientation.y = quaternion_rotation[1];
        marker_2.pose.orientation.z = quaternion_rotation[2];
        marker_2.pose.orientation.w = quaternion_rotation[3];

        marker_2.scale.x = tube_tolerances_[1] * 2;
        marker_2.scale.y = tube_tolerances_[2] * 2;
        marker_2.scale.z = tube_tolerances_[0] * 2;
        marker_2.color.a = 0.2; // Don't forget to set the alpha!
        marker_2.color.r = 102.0;
        marker_2.color.g = 178.0;
        marker_2.color.b = 255.0;
        //only if using a MESH_RESOURCE marker type:
        marker_2.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_2.lifetime = ros::Duration(1000);

        vis_pub.publish(marker_2);
        sleep(1);
    }
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(LwrRttControl)
