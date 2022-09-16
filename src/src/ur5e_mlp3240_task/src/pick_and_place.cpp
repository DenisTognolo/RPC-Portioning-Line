#include <ur5e_gripper_interface.h>
#include <chocolate_portioner_env.h>

int main(int argc, char** argv)
{
  // ROS setup

  ros::init(argc, argv, "ur5e_gripper_task");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);

  // Robot definition
  ur5e_gripper_interface robot_ur5e("ur5e_arm", "mlp3240_gripper");
  geometry_msgs::Pose tmp_pose;

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = 0.0;
  robot_pose.position.y = 0.05;
  robot_pose.position.z = 0.75;  

  // environment definition (supposing all origin.z on the floor)
  geometry_msgs::Pose shelf_origin;
  shelf_origin.position.x = 0.5;
  shelf_origin.position.y = 0.0;
  shelf_origin.position.z = 0.0;  
  
  geometry_msgs::Pose vision_box_origin;
  vision_box_origin.position.x = -0.50;
  vision_box_origin.position.y = -0.2;
  vision_box_origin.position.z = 0.0;
  
  geometry_msgs::Pose portioning_machine_origin;
  portioning_machine_origin.position.x = -0.55;
  portioning_machine_origin.position.y = 0.2;
  portioning_machine_origin.position.z = 0.0;

  double shelf_support_height = 0.75;
  double vision_box_support_height = 0.95;
  double portioning_machine_support_height = 0.85;

  std::vector<double> shelf_size = {0.15, 0.75, 0.45, shelf_support_height};
  std::vector<double> vision_box_size = {0.2, 0.2, 0.4, vision_box_support_height};
  std::vector<double> portioning_machine_size = {0.3, 0.3, 0.3, portioning_machine_support_height};

  std::vector<double> chocolate_bar_size = {0.14, 0.10, 0.01};
  std::vector<int> inventory = {1, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Get actual Goals poses
  std::string chosen_chocolate_bar_pose_code;
  nh.getParam("code", chosen_chocolate_bar_pose_code); 
  //ROS_INFO("Got parameter : %s", chosen_chocolate_bar_pose_code.c_str());  

  chocolate_portioner_env chocolate_portioner_env(robot_pose, shelf_origin, vision_box_origin, portioning_machine_origin, shelf_size, vision_box_size, portioning_machine_size, chocolate_bar_size, inventory);

  std::vector<double> shelf_desired_approach_RPY = {1.57, 0.0, 0.0};
  std::vector<double> vision_box_desired_approach_RPY = {-1.57, 3.14, 0.0};
  std::vector<double> portioning_machine_desired_approach_RPY = {-1.57, 3.14, 0.0};

  geometry_msgs::Pose chosen_chocolate_bar_pose = chocolate_portioner_env.compute_chosen_chocolate_bar_pose(chosen_chocolate_bar_pose_code, shelf_desired_approach_RPY);
  geometry_msgs::Pose vision_box_hole_pose = chocolate_portioner_env.compute_vision_box_hole_pose(vision_box_desired_approach_RPY);
  geometry_msgs::Pose portioning_machine_hole_pose = chocolate_portioner_env.compute_portioning_machine_hole_pose(portioning_machine_desired_approach_RPY);

  // Obstacle objects setup
  std::vector<std::string> obstacle_names;
  std::vector<shape_msgs::SolidPrimitive> obstacle_primitives;
  std::vector<geometry_msgs::Pose> obstacle_poses;

  // Obstacle 1 - Table  
  /*
  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 0.75;
  table_primitive.dimensions[1] = 0.75;
  table_primitive.dimensions[2] = 0.75;

  geometry_msgs::Pose table_pose;
  table_pose.position.x = 0.0 - robot_pose.position.x;
  table_pose.position.y = 0.0 - robot_pose.position.y;
  table_pose.position.z = (table_primitive.dimensions[2]/2 - robot_pose.position.z);

  obstacle_names.push_back("Table");
  obstacle_primitives.push_back(table_primitive);
  obstacle_poses.push_back(table_pose);
  */
  // Obstacle 2 - Shelf  
  
  shape_msgs::SolidPrimitive shelf_primitive;
  shelf_primitive.type = shelf_primitive.BOX;
  shelf_primitive.dimensions.resize(3);
  shelf_primitive.dimensions[0] = shelf_size[0];
  shelf_primitive.dimensions[1] = shelf_size[1];
  shelf_primitive.dimensions[2] = shelf_size[2];

  geometry_msgs::Pose shelf_pose = shelf_origin;
  shelf_pose.position.x -= robot_pose.position.x;
  shelf_pose.position.y -= robot_pose.position.y;
  shelf_pose.position.z -= robot_pose.position.z;
  shelf_pose.position.z += (shelf_size[2]/2 + shelf_support_height);

  obstacle_names.push_back("Shelf");
  obstacle_primitives.push_back(shelf_primitive);
  obstacle_poses.push_back(shelf_pose);

  /*
  // Obstacle 3 - vision box

  shape_msgs::SolidPrimitive vision_box_primitive;
  vision_box_primitive.type = vision_box_primitive.CYLINDER;
  vision_box_primitive.dimensions.resize(1);
  vision_box_primitive.dimensions[0] = {vision_box_size[2], vision_box_size[0]/2};
  
  geometry_msgs::Pose vision_box_pose = vision_box_origin;
  vision_box_pose.position.x -= robot_pose.position.x;
  vision_box_pose.position.y -= robot_pose.position.y;
  vision_box_pose.position.z -= robot_pose.position.z;
  vision_box_pose.position.z += (vision_box_size[2]/2 + vision_box_support_height);

  obstacle_names.push_back("vision_box");
  obstacle_primitives.push_back(vision_box_primitive);
  obstacle_poses.push_back(vision_box_pose);  
  */

  // Obstacle 4 - portioning machine
  
  shape_msgs::SolidPrimitive portioning_machine_primitive;
  portioning_machine_primitive.type = portioning_machine_primitive.BOX;
  portioning_machine_primitive.dimensions.resize(3);
  portioning_machine_primitive.dimensions[0] = portioning_machine_size[0];
  portioning_machine_primitive.dimensions[1] = portioning_machine_size[1];
  portioning_machine_primitive.dimensions[2] = portioning_machine_size[2];

  geometry_msgs::Pose portioning_machine_pose = portioning_machine_origin;
  portioning_machine_pose.position.x -= robot_pose.position.x;
  portioning_machine_pose.position.y -= robot_pose.position.y;
  portioning_machine_pose.position.z -= robot_pose.position.z;
  portioning_machine_pose.position.z += (portioning_machine_size[2]/2 + portioning_machine_support_height);

  obstacle_names.push_back("portioning_machine");
  obstacle_primitives.push_back(portioning_machine_primitive);
  obstacle_poses.push_back(portioning_machine_pose);  

  // Obstacle 5 - Walls
  
  shape_msgs::SolidPrimitive wall_primitive;
  wall_primitive.type = wall_primitive.BOX;
  wall_primitive.dimensions.resize(3);
  wall_primitive.dimensions[0] = 1.5;
  wall_primitive.dimensions[1] = 0.05;
  wall_primitive.dimensions[2] = 1.0;

  geometry_msgs::Pose back_wall_pose;
  back_wall_pose.position.x = 0.0 - robot_pose.position.x;
  back_wall_pose.position.y = -0.5 - robot_pose.position.y;
  back_wall_pose.position.z = 1.0 - robot_pose.position.z;

  obstacle_names.push_back("back_wall");
  obstacle_primitives.push_back(wall_primitive);
  obstacle_poses.push_back(back_wall_pose);  

  geometry_msgs::Pose front_wall_pose;
  front_wall_pose.position.x = 0.0 - robot_pose.position.x;
  front_wall_pose.position.y = 0.5 - robot_pose.position.y;
  front_wall_pose.position.z = 1.0 - robot_pose.position.z;

  obstacle_names.push_back("front_wall");
  obstacle_primitives.push_back(wall_primitive);
  obstacle_poses.push_back(front_wall_pose);
  

  // Add environment info to the robot
  robot_ur5e.add_collision_objects(obstacle_names, obstacle_primitives, obstacle_poses);

  double approach_offset = 0.05;

  spinner.start();
    
    // 1. Move to home position
    std::cout << "GOING TO HOME POSITION..." << std::endl;
    robot_ur5e.go_to_config("home");

    // 2. Place the EE in front of the chosen chocolate bar
    std::cout << "APPROACHING THE CHOCOLATE BAR..." << std::endl;
    tmp_pose = chosen_chocolate_bar_pose;
    tmp_pose.position.x -= approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 3. Open the gripper
    std::cout << "OPENING THE GRIPPER..." << std::endl;
    robot_ur5e.move_gripper("open");

    // 4. Move the EE close to the object
    std::cout << "GRASPING THE CHOCOLATE BAR..." << std::endl;
    robot_ur5e.go_to_pose(chosen_chocolate_bar_pose);

    // 5. Close the  gripper
    std::cout << "CLOSING THE GRIPPER..." << std::endl;
    robot_ur5e.move_gripper("close");
    ros::Duration(1.0).sleep(); 
    approach_offset += chocolate_bar_size[0]*3/4;

    // 6. Place the EE out of the shelf
    std::cout << "EXITING THE SHELF..." << std::endl;
    tmp_pose = chosen_chocolate_bar_pose;
    tmp_pose.position.x -= approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 7. Move the EE in front of the vision box
    std::cout << "APPROACHING THE VISION BOX..." << std::endl;
    tmp_pose = vision_box_hole_pose;
    tmp_pose.position.x += approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 8. Move the EE inside of the vision box
    std::cout << "GOING INSIDE THE VISON BOX..." << std::endl;
    robot_ur5e.go_to_pose(vision_box_hole_pose);
    /*
    // 9. Rotate the EE inside of the vision box
    std::cout << "WAITING 1 SEC..." << std::endl;
    ros::Duration(1.0).sleep(); 

    std::cout << "FLIPPING THE CHOCOLATE BAR..." << std::endl;
    //tmp_pose = set_RPY(vision_box_hole_pose, {0.0 , 0.0, 1.57});
    robot_ur5e.actuate_one_joint(5, 3.14);

    std::cout << "WAITING 1 SEC..." << std::endl;
    ros::Duration(1.0).sleep(); 

    std::cout << "RE-FLIPPING THE CHOCOLATE BAR..." << std::endl;
    //tmp_pose = set_RPY(vision_box_hole_pose, {0.0 , 0.0, 1.57});
    robot_ur5e.actuate_one_joint(5, -3.14);
    */
    // 10. Move the EE outside of the vision box
    std::cout << "EXITING THE VISION BOX..." << std::endl;
    tmp_pose = vision_box_hole_pose;
    tmp_pose.position.x += approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 11. Move the EE in front of the portioning machine
    std::cout << "APPROACHING THE PORTIONING MACHINE..." << std::endl;
    tmp_pose = portioning_machine_hole_pose;
    tmp_pose.position.x += approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 12. Entering the EE inside the portioning machine
    std::cout << "ENTERING THE PORTIONING MACHINE..." << std::endl;
    robot_ur5e.go_to_pose(portioning_machine_hole_pose);

    // 13. Open the gripper
    std::cout << "OPENING THE GRIPPER..." << std::endl;
    robot_ur5e.move_gripper("open");

    // 14. Move the EE out of the portioning machine
    std::cout << "BACK OFF FROM PORTIONING MACHINE..." << std::endl;
    tmp_pose = portioning_machine_hole_pose;
    tmp_pose.position.x += approach_offset;
    robot_ur5e.go_to_pose(tmp_pose);

    // 15. Move back to home position
    std::cout << "GOING BACK TO HOME POSITION..." << std::endl;
    robot_ur5e.go_to_config("home");

  robot_ur5e.remove_collision_objects();

  ros::shutdown();
  return 0;
}
