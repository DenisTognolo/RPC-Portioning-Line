#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

class ur5e_gripper_interface 
{
public:
    moveit::planning_interface::MoveGroupInterface *move_group_interface_arm;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_gripper;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> obstacle_ids;

    bool success;
  
  ur5e_gripper_interface(const std::string robot_group, const std::string gripper_group)
  {
    move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(robot_group);
    move_group_interface_gripper = new moveit::planning_interface::MoveGroupInterface(gripper_group);
  };

  void add_collision_objects(std::vector<std::string> obstacle_names, std::vector<shape_msgs::SolidPrimitive> obstacle_primitives, std::vector<geometry_msgs::Pose> obstacle_poses){
    // Add to the MoveIt environment information about collision objects, given a list of obstacles (names, primitives and origins)
    
    obstacle_ids = obstacle_names;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::cout << "Adding " << obstacle_names.size() << " objects as obstacles..." << std::endl;

    moveit_msgs::CollisionObject collision_object;
    for(int i=0; i < obstacle_names.size(); i++){
      collision_object.id = obstacle_names[i];
      collision_object.header.frame_id = this->move_group_interface_arm->getPlanningFrame();
      collision_object.primitives.push_back(obstacle_primitives[i]);
      collision_object.primitive_poses.push_back(obstacle_poses[i]);
      collision_object.operation = collision_object.ADD;
      
      collision_objects.push_back(collision_object);
    }
    this->planning_scene_interface.applyCollisionObjects(collision_objects);
  }

  void remove_collision_objects(){
    // Remove from the MoveIt environment all collision objects

    std::cout << "Removing " << obstacle_ids.size() << " objects as obstacles..." << std::endl;
    this->planning_scene_interface.removeCollisionObjects(obstacle_ids);
  }

  geometry_msgs::PoseStamped get_current_pose(){
    // Return the pose of the End Effector (mean of the gripper finger origins) considering the actual robot configuration

    geometry_msgs::PoseStamped actual_pose;
    geometry_msgs::PoseStamped actual_pose_l = move_group_interface_gripper->getCurrentPose("finger1");
    geometry_msgs::PoseStamped actual_pose_r = move_group_interface_gripper->getCurrentPose("finger2");
    actual_pose.pose.position.x = (actual_pose_l.pose.position.x + actual_pose_r.pose.position.x) /2;
    actual_pose.pose.position.y = (actual_pose_l.pose.position.y + actual_pose_r.pose.position.y) /2;
    actual_pose.pose.position.z = (actual_pose_l.pose.position.z + actual_pose_r.pose.position.z) /2;
    return actual_pose;
  }

  void print_current_current_pose(std::string link){
    // Print the pose of a given link name, considering the actual robot configuration

    std::cout << "\n_______________________________Actual EE Pose________________________________ " << std::endl;
    std::cout << move_group_interface_arm->getCurrentPose(link).pose.position <<  std::endl;
    std::cout << "____________________________________________________________________________ " << std::endl;
  }

  void print_current_joints_config(){
    // Print the actual joint configuration (each joints angles/position)

    std::vector<double> actual_config = move_group_interface_arm->getCurrentJointValues();
    std::cout << "\n________________________Actual Joints Configurations________________________ " << std::endl;
    std::cout << "elbow_joint: " << actual_config[0] << " | shoulder_lift_joint: " << actual_config[1] << " | shoulder_pan_joint: " << actual_config[2] << std::endl; 
    std::cout << "wrist_1_joint: " << actual_config[3] << " | wrist_2_joint: " << actual_config[4] << " | wrist_3_joint: " << actual_config[5] << std::endl;
    std::cout << "gripper : " << actual_config[6] << std::endl;
    std::cout << "____________________________________________________________________________ " << std::endl;
  }

  bool go_to_pose(geometry_msgs::Pose desired_pose){
    // Force MoveIt to plan a trajectory to move the robot in a given a desired pose and perform it 

    move_group_interface_arm->setPoseTarget(desired_pose);
    success = (move_group_interface_arm->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("portionig_line", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED [go_to_pose]");
    move_group_interface_arm->move();                        
    move_group_interface_arm->stop();

    print_current_current_pose("ee_tool");
    return success;
  }

  bool go_to_config(std::string desired_config){
    // Force the robot to assume a known and given joint configuration 

    move_group_interface_arm->setJointValueTarget(move_group_interface_arm->getNamedTargetValues(desired_config));
    success = (move_group_interface_arm->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("portionig_line", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED [go_to_config]");
    move_group_interface_arm->move();
    move_group_interface_arm->stop();
    return success;
  }

  bool actuate_one_joint(int joint_index, double desired_angle){
    // Force the robot to move a given joint by a given angle/position

    std::vector<double> tmp_joint_configuration = move_group_interface_arm->getCurrentJointValues();
    tmp_joint_configuration[joint_index] += desired_angle;
    move_group_interface_arm->setJointValueTarget(tmp_joint_configuration);
    success = (move_group_interface_arm->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("portionig_line", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED [go_to_config]");
    move_group_interface_arm->move();
    move_group_interface_arm->stop();
    return success;
  }

  bool move_gripper(std::string desired_config){
    // Force the gripper to move in a known and given desired configuration

    move_group_interface_gripper->setJointValueTarget(move_group_interface_gripper->getNamedTargetValues(desired_config));
    success = (move_group_interface_gripper->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("portionig_line", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED [move_gripper]");
    move_group_interface_gripper->move();
    move_group_interface_arm->stop();
    return success;
  }

  // CLASS DISTRUCTOR
  ~ur5e_gripper_interface(){
           delete move_group_interface_arm;
           delete move_group_interface_gripper;
       }
};

