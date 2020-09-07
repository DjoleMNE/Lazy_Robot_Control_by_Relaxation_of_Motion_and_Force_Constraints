#include <constants.hpp>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0

namespace youbot_constants
{
    //Robot ID/Name
    const std::string ID("yb_arm");

    // Number of joints in the manipulator
    const int NUMBER_OF_JOINTS(5);
    const int NUMBER_OF_SEGMENTS(5);
    const int NUMBER_OF_FRAMES(6);

    //Arm's root acceleration
    const std::vector<double> root_acceleration {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};

    //Kuka youBot store position limit values: positive and negative
    const std::vector<double> joint_position_limits_max_1 {  2.9496 - DEG_TO_RAD(3),  1.5707 - DEG_TO_RAD(3),  2.5481 - DEG_TO_RAD(3),  1.7889 - DEG_TO_RAD(3),  2.9234 - DEG_TO_RAD(3)};
    const std::vector<double> joint_position_limits_min_1 { -2.9496 + DEG_TO_RAD(3), -1.1344 + DEG_TO_RAD(3), -2.6354 + DEG_TO_RAD(3), -1.7889 + DEG_TO_RAD(3), -2.9234 + DEG_TO_RAD(3)};

    //Position limit values from URDF file: positive and negative
    const std::vector<double> joint_position_limits_max_2 {5.89921 - DEG_TO_RAD(3), 2.70526 - DEG_TO_RAD(3),  0.00000 - DEG_TO_RAD(1), 3.57792 - DEG_TO_RAD(3), 5.84685 - DEG_TO_RAD(3)};
    const std::vector<double> joint_position_limits_min_2 {0.00000 + DEG_TO_RAD(3), 0.00000 + DEG_TO_RAD(3), -5.16617 + DEG_TO_RAD(1), 0.00000 + DEG_TO_RAD(3), 0.00000 + DEG_TO_RAD(3)};
    // const std::vector<double> joint_position_limits_max_2 { 5.899210,  2.705260,  0.000001,  3.577920,  5.846850};
    // const std::vector<double> joint_position_limits_min_2 {-0.000001, -0.000001, -5.166170, -0.000001, -0.000001};
    
    const std::vector<double> joint_position_limits_max_2_sim { 5.899210,  2.705260,  0.000001,  3.577920,  5.846850};
    const std::vector<double> joint_position_limits_min_2_sim {-0.000001, -0.000001, -5.166170, -0.000001, -0.000001};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
    const std::vector<double> joint_position_thresholds {DEG_TO_RAD(15), DEG_TO_RAD(15), DEG_TO_RAD(3), DEG_TO_RAD(10), DEG_TO_RAD(10)};

    // Robocup URDF file parameters for velocity limits
    // const std::vector<double> joint_velocity_limits {1.5707, 0.8, 1.0, 1.5707, 1.5707};

    // YouBot Store velocity limits
    const std::vector<double> joint_velocity_limits {1.5707, 1.5707, 1.5707, 1.5707, 1.5707};
    const std::vector<double> joint_acceleration_limits {5.0, 5.0, 5.0, 5.0, 5.0};// confirm these numbers

    // JP's max torques
   //  const std::vector<double> joint_torque_limits {12.9012, 12.9012, 8.2700, 4.1748, 1.7550};

   //  // youBot store's max torques 
   //  const std::vector<double> joint_torque_limits {9.5, 9.5, 6.0, 2.0, 1.0};

    // custom max torques
    const std::vector<double> joint_torque_limits {14.9012, 14.9012, 8.0, 3.3, 1.2};
    const std::vector<double> joint_stopping_torque_limits {14.9012, 14.9012, 8.0, 3.3, 1.2};

    // Benjamin Keiser' max torques (fast version)
    // const std::vector<double> joint_torque_limits {17.0, 17.0, 8.0, 2.7, 1.0}};

    // Offsets required for the youBot store model: Negative Candle config values -Robocup
    // const std::vector<double> youbot_joint_offsets {-2.1642, -1.13446, 2.54818, -1.78896, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values - JP 
    const std::vector<double> joint_offsets {-2.94960, -1.13446, 2.54818, -1.78896, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values -Sven's
    // const std::vector<double> joint_offsets {-2.9496, -1.1344, 2.6354, -1.7890, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values: keiser's
    // const std::vector<double> joint_offsets {-2.9496, -1.1344, 2.5481, -1.7889, -2.9234};

    // Rotor inertia - "d" in the algorithm: Computed from youBot store values
    const std::vector<double> joint_inertia {0.33848, 0.33848, 0.13571, 0.04698, 0.01799};

    const std::string config_path = "/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config";    
    const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_only.urdf";
   //  const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_zero_inertia.urdf";

    const std::string root_name    = "arm_link_0";
    const std::string tooltip_name = "arm_link_5";
}

namespace kinova_constants
{
   //Robot ID/Name
   const std::string ID("kinova_gen3_arm");

   // Number of joints in the manipulator
   const int NUMBER_OF_JOINTS(7);
   const int NUMBER_OF_SEGMENTS(7);
   const int NUMBER_OF_FRAMES(8);

   //Arm's root acceleration
   const std::vector<double> root_acceleration_1 {-0.07, 0.04, 9.48, 0.0, 0.0, 0.0}; // For Kinova 1
   const std::vector<double> root_acceleration_2 {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};  // For Kinova 2
//    const std::vector<double> root_acceleration_sim {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0}; // For internal simulation in the mediator
   const std::vector<double> root_acceleration_sim {-0.07, 0.04, 9.48, 0.0, 0.0, 0.0}; // For internal simulation in the mediator

   // Limits from Kinova manual-> Must be confirmed
   const std::vector<double> joint_position_limits_max {DEG_TO_RAD(9999.0), DEG_TO_RAD(127.0), DEG_TO_RAD(9999.0), DEG_TO_RAD(147.8), DEG_TO_RAD(9999.0), DEG_TO_RAD(120.3), DEG_TO_RAD(9999.0)};
   const std::vector<double> joint_position_limits_min {DEG_TO_RAD(-9999.0), DEG_TO_RAD(-127.0), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-147.8), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-120.3), DEG_TO_RAD(-9999.0)};

   const std::vector<double> joint_velocity_limits {DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0)};
   const std::vector<double> joint_acceleration_limits {5.19, 5.19, 5.19, 5.19, 9.99, 9.99, 9.99};
   const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
   const std::vector<double> joint_stopping_torque_limits {39.0, 39.0, 39.0, 39.0, 13.0, 13.0, 13.0};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
   const std::vector<double> joint_position_thresholds {DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10)};

   const std::vector<double> joint_offsets {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Rotor inertia - "d" in the algorithm:
    const std::vector<double> joint_inertia {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
    // const std::vector<double> joint_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> joint_sim_inertia {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
    // const std::vector<double> joint_sim_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12.urdf";
   const std::string urdf_sim_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12_sim.urdf";

   // 7 joints, 7 links, 8 frames
   const std::string root_name = "base_link";

   /**
    * With Bracelet_Link parameter, the last frame is at joint 7.
    * Mass and COM of the last (end-effector) link are included but not the real end-effector's frame.
    * Arm length: 1.12586m
    */
   const std::string tooltip_name = "Bracelet_Link";
   const std::string tooltip_sim_name = "EndEffector_Link";

   /**
    * With EndEffector_Link parameter, last frame is at the real end-effector's frame.
    * However, in the urdf model, joint between Bracelet_Link and EndEffector_Link is fixed (not counted in KDL). 
    * Vereshchagin does not support un-equal number of joints and segments
    * Arm length: 1.1873m
    */ 
//    const std::string tooltip_name = "EndEffector_Link";
}
namespace lwr_constants
{
   //Robot ID/Name
   const std::string ID("lwr_arm");

   // Number of joints in the manipulator
   const int NUMBER_OF_JOINTS(7);
   const int NUMBER_OF_SEGMENTS(7);
   const int NUMBER_OF_FRAMES(8);

   //Arm's root acceleration
   const std::vector<double> root_acceleration {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};

   // Limits from KUKA manual-> Must be confirmed
   const std::vector<double> joint_position_limits_max {DEG_TO_RAD(170), DEG_TO_RAD(120), DEG_TO_RAD(170), DEG_TO_RAD(120), DEG_TO_RAD(170), DEG_TO_RAD(120), DEG_TO_RAD(170)};
   const std::vector<double> joint_position_limits_min {DEG_TO_RAD(-170), DEG_TO_RAD(-120), DEG_TO_RAD(-170), DEG_TO_RAD(-120), DEG_TO_RAD(-170), DEG_TO_RAD(-120), DEG_TO_RAD(-170)};

   const std::vector<double> joint_velocity_limits {DEG_TO_RAD(112.5), DEG_TO_RAD(112.5), DEG_TO_RAD(112.5), DEG_TO_RAD(112.5), DEG_TO_RAD(112.5), DEG_TO_RAD(112.5), DEG_TO_RAD(112.5)};
   const std::vector<double> joint_acceleration_limits {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // confirm these numbers
   const std::vector<double> joint_torque_limits {176.0, 176.0, 100.0, 100.0, 100.0, 30.0, 30.0};
   const std::vector<double> joint_stopping_torque_limits {176.0, 176.0, 100.0, 100.0, 100.0, 30.0, 30.0};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
   const std::vector<double> joint_position_thresholds {DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10)};

   const std::vector<double> joint_offsets {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   // Rotor inertia - "d" in the algorithm:
   // Taken from: Jubien, A., Gautier, M. and Janot, A., 2014. Dynamic identification of the Kuka LWR robot using motor torques
   // and joint torque sensors data. IFAC Proceedings Volumes, 2014., 47(3), pp.8391-8396.
   const std::vector<double> joint_inertia {3.19, 3.05, 1.98, 2.05, 0.787, 0.391, 0.394};

   const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/lwr.urdf";
   const std::string urdf_with_ati_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/lwr_with_ati.urdf";

   const std::string root_name = "link_0";   
   //   7 joints, 7 links, 8 frames. frames from link 0 to link 7
   const std::string tooltip_name = "link_7";
   //  arm as usual: 7 joints & 7 links, ati sensor additional 2 (fixed-not counted in kdl)joints and 2 links
   // const std::string tooltip_name = "ati_link";
}

namespace abag_parameter
{
    // How many dimension ABAG controller is supposed to control
    const int DIMENSIONS(6);

    // Error parameters: Low pass filter threshold
    const Eigen::VectorXd ERROR_ALPHA = (Eigen::VectorXd(DIMENSIONS) << 0.800000, 0.800000, 0.800000, 0.650000, 0.850000, 0.178001).finished();

     // Bias parameters: threshold and step
    const Eigen::VectorXd BIAS_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) << 0.000507, 0.000507, 0.000507, 0.001007, 0.001007, 0.724277).finished();
    const Eigen::VectorXd BIAS_STEP = (Eigen::VectorXd(DIMENSIONS) << 0.000495, 0.000495, 0.000495, 0.003495, 0.003495, 0.503495).finished();

    // Gain parameters: threshold and step
    const Eigen::VectorXd GAIN_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) << 0.452492, 0.452492, 0.452492, 0.252492, 0.252492, 0.432492).finished();
    const Eigen::VectorXd GAIN_STEP = (Eigen::VectorXd(DIMENSIONS) << 0.002052, 0.002052, 0.002052, 0.015152, 0.015152, 0.655152).finished();

    // Parameters for controlling robot's nullspace motion
    const double NULL_SPACE_ERROR_ALPHA    = 0.900000;
    const double NULL_SPACE_BIAS_THRESHOLD = 0.000407;
    const double NULL_SPACE_BIAS_STEP      = 0.000495;
    const double NULL_SPACE_GAIN_THRESHOLD = 0.552492;
    const double NULL_SPACE_GAIN_STEP      = 0.003152;

   //  Saturation limits   
    const Eigen::VectorXd MIN_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).finished();
   //  const Eigen::VectorXd MIN_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS)  << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
    const Eigen::VectorXd MIN_GAIN_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_GAIN_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
    const Eigen::VectorXd MIN_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).finished();
   //  const Eigen::VectorXd MIN_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
}

namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    const int NUMBER_OF_CONSTRAINTS(6);
    const int DECELERATION_UPDATE_DELAY = 5; // Iterations
    const int STEADY_STOP_ITERATION_THRESHOLD = 40; // Iterations
    const double LOWER_DECELERATION_RAMP_THRESHOLD = 0.05; // rad/sec
    const double STOPPING_MOTION_LOOP_FREQ = 750.0; // Hz  ... Higher than 750 Hz not yet feasible with the current Kinova API
    const Eigen::VectorXd MAX_CART_FORCE = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 50.0, 50.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::VectorXd MAX_CART_ACC = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 100.0, 100.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::IOFormat WRITE_FORMAT(6, Eigen::DontAlignCols, " ", "", "", "\n");
    const std::string LOG_FILE_CART_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/control_error.txt");
    const std::string LOG_FILE_STOP_MOTION_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/stop_motion_error.txt");
    const std::string LOG_FILE_CART_BASE_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/control_base_error.txt");
    const std::string LOG_FILE_JOINT_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/joint_torques.txt");
    const std::string LOG_FILE_EXT_WRENCH_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/ext_wrench_data.txt");
    const std::string LOG_FILE_PREDICTIONS_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/prediction_effects.txt");
    const std::string LOG_FILE_NULL_SPACE_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/null_space_error.txt");
}

namespace prediction_parameter
{
		const std::string CURRENT_POSE_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/measured_pose.txt";
		const std::string PREDICTED_POSE_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/predicted_pose.txt";
		const std::string TWIST_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/current_twist.txt";
}