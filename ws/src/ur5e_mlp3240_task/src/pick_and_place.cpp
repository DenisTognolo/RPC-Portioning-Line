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

  // environment definition (let's consider all models have origin on the floor [z = 0])
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

  std::vector<double> shelf_size = {0.15, 0.75, 0.45, shelf_support_height, shelf_basement_height, shelf_to_chocolate_bar_origin_x_offset};
  std::vector<double> vision_box_size = {0.3, 0.3, 0.95, vision_box_support_height, vision_box_base_height, vision_box_rod_height};
  std::vector<double> ring_light_size = {0.2, 0.2, 0.03, ring_light_support_height};
  std::vector<double> portioning_machine_size = {0.3, 0.3, 0.3, portioning_machine_support_height};

  std::vector<double> chocolate_bar_size = {0.14, 0.10, 0.01};
  std::vector<int> inventory = {1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1};

  // Define the orientation that the robot must assume when it goes to this positions
  std::vector<double> shelf_desired_approach_RPY = {1.57, 0.0, 0.0};
  std::vector<double> vision_box_desired_approach_RPY = {-1.57, 3.14, 0.0};
  std::vector<double> portioning_machine_desired_approach_RPY = {-1.57, 3.14, 0.0};

  // Build the environment, and setup the robot poses using the classes
  chocolate_portioner_env chocolate_portioner_env(robot_pose, shelf_origin, vision_box_origin, portioning_machine_origin,
                                                  shelf_size, vision_box_size, ring_light_size,
                                                  portioning_machine_size, chocolate_bar_size, inventory);

  std::string chosen_chocolate_bar_pose_code;
  nh.getParam("code", chosen_chocolate_bar_pose_code);
  // ROS_INFO("Got parameter : %s", chosen_chocolate_bar_pose_code.c_str());

  geometry_msgs::Pose chosen_chocolate_bar_origin = chocolate_portioner_env.compute_chosen_chocolate_bar_origin(chosen_chocolate_bar_pose_code);

  geometry_msgs::Pose chosen_chocolate_bar_goal_pose = chocolate_portioner_env.compute_chosen_chocolate_bar_pose(chosen_chocolate_bar_origin, shelf_desired_approach_RPY);
  geometry_msgs::Pose vision_box_hole_goal_pose = chocolate_portioner_env.compute_vision_box_hole_pose(vision_box_desired_approach_RPY);
  geometry_msgs::Pose portioning_machine_hole_goal_pose = chocolate_portioner_env.compute_portioning_machine_hole_pose(portioning_machine_desired_approach_RPY);

  // Build collision objects
  chocolate_portioner_env.compute_obstacle_names();
  chocolate_portioner_env.compute_obstacle_primitives();
  chocolate_portioner_env.compute_obstacle_poses();

  // Add all environment collision info to the robot
  robot_ur5e.add_collision_objects(chocolate_portioner_env.obstacle_names, chocolate_portioner_env.obstacle_primitives, chocolate_portioner_env.obstacle_poses);

  moveit_msgs::AttachedCollisionObject chocolate_bar_attached_collision_object;
  chocolate_bar_attached_collision_object.link_name = "ee_tool";
  chocolate_bar_attached_collision_object.object.id = chosen_chocolate_bar_pose_code;
  chocolate_bar_attached_collision_object.object.primitives.push_back(chocolate_portioner_env.chocolate_bar_primitive);
  chocolate_bar_attached_collision_object.object.primitive_poses.push_back(chosen_chocolate_bar_origin);

  // Define an approach offsets
  double approach_offset = chocolate_bar_size[0] * 3 / 4 + 0.01;

  // Setup complete, let's start the task!
  spinner.start();

  // 1. Move to home position
  std::cout << "GOING TO HOME POSITION..." << std::endl;
  robot_ur5e.go_to_config("home");

  // 2. Open the gripper
  std::cout << "OPENING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("open");

  // 3. Move the EE close to the object
  std::cout << "GRASPING THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.go_to_pose(chosen_chocolate_bar_goal_pose);

  // 4. Close the  gripper
  std::cout << "CLOSING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("close");

  robot_ur5e.add_attached_collision_object(chocolate_bar_attached_collision_object);

  // 5. Place the Chocolate Bar out of the shelf
  std::cout << "EXITING THE SHELF..." << std::endl;
  tmp_pose = chosen_chocolate_bar_goal_pose;
  tmp_pose.position.x -= approach_offset;
  robot_ur5e.go_to_pose(tmp_pose);

  // 6. Move the Chocolate Bar inside of the vision box
  std::cout << "GOING INSIDE THE VISON BOX..." << std::endl;
  robot_ur5e.go_to_pose(vision_box_hole_goal_pose);

  std::cout << "WAITING 1 SEC..." << std::endl;
  ros::Duration(1.0).sleep();

  // 7. Rotate the Chocolate Bar inside of the vision box
  std::cout << "FLIPPING THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.actuate_one_joint(5, 3.14);

  std::cout << "WAITING 1 SEC..." << std::endl;
  ros::Duration(1.0).sleep();

  std::cout << "RE-FLIPPING THE CHOCOLATE BAR..." << std::endl;
  robot_ur5e.actuate_one_joint(5, -3.14);

  // 8. Move the Chocolate Bar outside of the vision box
  std::cout << "EXITING THE VISION BOX..." << std::endl;
  tmp_pose = vision_box_hole_goal_pose;
  tmp_pose.position.x += approach_offset;
  robot_ur5e.go_to_pose(tmp_pose);

  // 9. Entering the Chocolate Bar inside the portioning machine
  std::cout << "ENTERING THE PORTIONING MACHINE..." << std::endl;
  robot_ur5e.go_to_pose(portioning_machine_hole_goal_pose);

  robot_ur5e.remove_attached_collision_object(chocolate_bar_attached_collision_object);

  // 10. Open the gripper
  std::cout << "OPENING THE GRIPPER..." << std::endl;
  robot_ur5e.move_gripper("open");

  // 11. Move back to home position
  std::cout << "GOING BACK TO HOME POSITION..." << std::endl;
  robot_ur5e.go_to_config("home");

  robot_ur5e.remove_collision_objects();

  ros::shutdown();
  return 0;
}