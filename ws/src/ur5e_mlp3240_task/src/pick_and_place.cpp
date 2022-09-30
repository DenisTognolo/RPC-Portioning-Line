#include <ur5e_gripper_interface.h>
#include <chocolate_portioner_env.h>

int main(int argc, char **argv)
{
  // ROS setup
  ros::init(argc, argv, "ur5e_gripper_task");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);

  // Robot definition
  ur5e_gripper_interface robot_ur5e("ur5e_arm", "mlp3240_gripper");
  geometry_msgs::Pose tmp_pose;

  geometry_msgs::Pose robot_pose = robot_ur5e.set_position({0.0, 0.0, 0.75});

  // Environment definition (let's consider all models have origin on the floor [z = 0])
  geometry_msgs::Pose shelf_origin = robot_ur5e.set_position({0.6, 0.0, 0.0});
  geometry_msgs::Pose vision_box_origin = robot_ur5e.set_position({-0.5, -0.2, 0.0});
  geometry_msgs::Pose portioning_machine_origin = robot_ur5e.set_position({-0.55, 0.2, 0.0});

  double shelf_support_height = 0.90;
  double shelf_basement_height = 0.01;
  double shelf_to_chocolate_bar_origin_x_offset = 0.06;
  double vision_box_support_height = 0.75;
  double ring_light_support_height = 0.85;
  double vision_box_base_height = 0.05;
  double vision_box_rod_height = vision_box_support_height + 0.13;
  double portioning_machine_support_height = 0.85;
  double portioning_machine_hole_z_offset = 0.065;
  double portioning_machine_hole_width = 0.180;
  double portioning_machine_hole_hight = 0.06;

  std::vector<double> table_size = {0.75, 0.75, 0.745};
  std::vector<double> shelf_size = {0.17, 0.75, 0.45, shelf_support_height, shelf_basement_height, shelf_to_chocolate_bar_origin_x_offset};
  std::vector<double> vision_box_size = {0.3, 0.3, 0.95, vision_box_support_height, vision_box_base_height, vision_box_rod_height};
  std::vector<double> ring_light_size = {0.2, 0.2, 0.03, ring_light_support_height};
  std::vector<double> portioning_machine_size = {0.3, 0.3, 0.3, portioning_machine_support_height, portioning_machine_hole_z_offset, portioning_machine_hole_width, portioning_machine_hole_hight};

  std::vector<double> chocolate_bar_size = {0.14, 0.10, 0.01};
  std::vector<std::string> chocolate_codes = {"A1","A2","A3","A4","A5","B1","B2","B3","B4","B5","C1","C2","C3","C4","C5"};
  std::vector<int> inventory = {1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1};

  // Define the orientation that the robot must assume when it goes to this positions
  std::vector<double> shelf_desired_approach_RPY = {1.57, 0.0, 0.0};
  std::vector<double> vision_box_desired_approach_RPY = {-1.57, 3.14, 0.0};
  std::vector<double> portioning_machine_desired_approach_RPY = {-1.57, 3.14, 0.0};

  // Build the environment, and setup the robot poses using the classes
  chocolate_portioner_env chocolate_portioner_env(robot_pose, shelf_origin, vision_box_origin, portioning_machine_origin,
                                                  table_size, shelf_size, vision_box_size, ring_light_size,
                                                  portioning_machine_size, chocolate_bar_size, chocolate_codes, inventory);

  std::string chosen_chocolate_bar_pose_code;
  nh.getParam("code", chosen_chocolate_bar_pose_code);
  // ROS_INFO("Got parameter : %s", chosen_chocolate_bar_pose_code.c_str());

  // Compute goal positions for the robot
  chosen_chocolate_bar_pose_code = chocolate_portioner_env.check_chocolate_bar_code(chosen_chocolate_bar_pose_code);
  geometry_msgs::Pose chosen_chocolate_bar_origin = chocolate_portioner_env.compute_chosen_chocolate_bar_origin(chosen_chocolate_bar_pose_code);
  geometry_msgs::Pose chosen_chocolate_bar_goal_pose = chocolate_portioner_env.compute_chosen_chocolate_bar_pose(chosen_chocolate_bar_origin, shelf_desired_approach_RPY);
  geometry_msgs::Pose vision_box_hole_goal_pose = chocolate_portioner_env.compute_vision_box_hole_pose(vision_box_desired_approach_RPY);
  geometry_msgs::Pose portioning_machine_hole_goal_pose = chocolate_portioner_env.compute_portioning_machine_hole_pose(portioning_machine_desired_approach_RPY);

  // Build collision objects
  std::vector<std::string> obstacle_names = chocolate_portioner_env.compute_obstacle_names();
  std::vector<shape_msgs::SolidPrimitive> obstacle_primitives = chocolate_portioner_env.compute_obstacle_primitives();
  std::vector<geometry_msgs::Pose> obstacle_poses = chocolate_portioner_env.compute_obstacle_poses();

  // Add all environment collision info to the robot
  robot_ur5e.add_collision_objects(obstacle_names, obstacle_primitives, obstacle_poses, chosen_chocolate_bar_pose_code);
  std::cout << "HERES..." << std::endl;

  // Setup complete, let's start the task!
  spinner.start();

  // 1. Move to home position
  std::cout << "GOING TO HOME POSITION..." << std::endl;
  robot_ur5e.go_to_config("home");

  // 2. Open the gripper
  std::cout << "OPENING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("open");

  // 3. Move the EE close to the chocolate bar
  std::cout << "GOING TO THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.go_to_pose(chosen_chocolate_bar_goal_pose);

  // 4. Close the  gripper
  std::cout << "CLOSING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("close");

  robot_ur5e.attach_grasped_object();

  // 5. Move the Chocolate Bar inside of the vision box
  std::cout << "GOING INSIDE THE VISON BOX..." << std::endl;
  robot_ur5e.go_to_pose(vision_box_hole_goal_pose);

  std::cout << "WAITING 1 SEC..." << std::endl;
  ros::Duration(1.0).sleep();

  // 6. Rotate the Chocolate Bar inside of the vision box
  std::cout << "FLIPPING THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.actuate_one_joint(5, 3.14);

  std::cout << "WAITING 1 SEC..." << std::endl;
  ros::Duration(1.0).sleep();

  std::cout << "RE-FLIPPING THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.actuate_one_joint(5, -3.14);

  // 7. Move the Chocolate Bar inside the portioning machine
  std::cout << "GOING INSIDE THE PORTIONING MACHINE..." << std::endl;
  robot_ur5e.go_to_pose(portioning_machine_hole_goal_pose);

  // 8. Open the gripper
  std::cout << "OPENING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("open");

  robot_ur5e.detach_grasped_object();

  // 9. Move back to home position
  std::cout << "GOING BACK TO HOME POSITION..." << std::endl;
  robot_ur5e.go_to_config("home");

  robot_ur5e.remove_collision_objects();

  ros::shutdown();
  return 0;
}