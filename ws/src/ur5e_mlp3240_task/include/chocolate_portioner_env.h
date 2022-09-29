#include <map>

class chocolate_portioner_env 
{
public:
  geometry_msgs::Pose shelf_origin;
  geometry_msgs::Pose vision_box_origin;
  geometry_msgs::Pose portioning_machine_origin;

  std::vector<double> shelf_size;
  std::vector<double> vision_box_size;
  std::vector<double> portioning_machine_size;
  std::vector<double> chocolate_bar_size;
  std::vector<double> ring_light_size;

  double shelf_support_height;
  double shelf_basement_height;
  double shelf_to_chocolate_bar_origin_x_offset;
  double vision_box_support_height;
  double ring_light_support_height;
  double vision_box_base_height;
  double vision_box_rod_height;
  double portioning_machine_support_height;

  std::vector<std::string> chocolate_code_labels;
  std::vector<int> inventory;
  std::map<std::string, geometry_msgs::Pose> shelf_pose_map;
  std::map<std::string, int> shelf_inventory_map;
  geometry_msgs::Pose tmp_pose;  

  geometry_msgs::Pose robot_pose;

  std::vector<std::string> obstacle_names;
  std::vector<shape_msgs::SolidPrimitive> obstacle_primitives;
  std::vector<geometry_msgs::Pose> obstacle_poses;

  shape_msgs::SolidPrimitive table_primitive;
  shape_msgs::SolidPrimitive chocolate_bar_primitive;

  chocolate_portioner_env(geometry_msgs::Pose robot_pose, geometry_msgs::Pose shelf_origin, geometry_msgs::Pose vision_box_origin, geometry_msgs::Pose portioning_machine_origin, std::vector<double> shelf_size, std::vector<double> vision_box_size, std::vector<double> ring_light_size, std::vector<double> portioning_machine_size, std::vector<double> chocolate_bar_size, std::vector<int> inventory)
  {
    this->robot_pose =  robot_pose;
    this->shelf_origin = shelf_origin;
    this->vision_box_origin = vision_box_origin;
    this->portioning_machine_origin = portioning_machine_origin;

    this->shelf_size = shelf_size;
    this->vision_box_size = vision_box_size;
    this->ring_light_size = ring_light_size;
    this->portioning_machine_size = portioning_machine_size;
    this->chocolate_bar_size = chocolate_bar_size;

    this->shelf_support_height = shelf_size[3];
    this->shelf_basement_height = shelf_size[4];
    this->shelf_to_chocolate_bar_origin_x_offset = shelf_size[5];
    this->vision_box_support_height = vision_box_size[3];
    this->vision_box_base_height = vision_box_size[4];
    this->vision_box_rod_height = vision_box_size[5];
    this->ring_light_support_height = ring_light_size[3];
    this->portioning_machine_support_height = portioning_machine_size[3];

    this->inventory = inventory;
    this->chocolate_code_labels = {"A1","A2","A3","A4","A5","B1","B2","B3","B4","B5","C1","C2","C3","C4","C5"};
    
    if(inventory.size() != chocolate_code_labels.size()){
      std::cout << "Error: this enviroment must specify "<< chocolate_code_labels.size() <<" stocks!" << std::endl;
    }
    // set origin.position.z at the base of the object (over the support) and in robot reference frame

    this->shelf_origin.position.x -= this->robot_pose.position.x;
    this->shelf_origin.position.y -= this->robot_pose.position.y;
    this->shelf_origin.position.z = this->shelf_origin.position.z - this->robot_pose.position.z + this->shelf_support_height;
    
    this->vision_box_origin.position.x -= this->robot_pose.position.x;
    this->vision_box_origin.position.y -= this->robot_pose.position.y;
    this->vision_box_origin.position.z = this->vision_box_origin.position.z - this->robot_pose.position.z + this->vision_box_support_height;

    this->portioning_machine_origin.position.x -= this->robot_pose.position.x;
    this->portioning_machine_origin.position.y -= this->robot_pose.position.y;
    this->portioning_machine_origin.position.z = this->portioning_machine_origin.position.z - this->robot_pose.position.z + this->portioning_machine_support_height;

    this->build_chocolate_bars_map();
  };

  geometry_msgs::Pose set_RPY(geometry_msgs::Pose pose, std::vector<double> desired_RPY){
    // Set the orientation of a given pose by a given RPY angles in radians

    tf::Quaternion quaternion;
    quaternion.setRPY( desired_RPY[0], desired_RPY[1], desired_RPY[2]);
    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();
    return pose;
  }

  geometry_msgs::Pose compute_chosen_chocolate_bar_origin(std::string chosen_chocolate_bar_code){
    // Compute the pose where a chosen chocolate bar is located, given its code

    while(shelf_pose_map.count(chosen_chocolate_bar_code) < 1 || shelf_inventory_map[chosen_chocolate_bar_code] < 1){ 
      if (shelf_pose_map.count(chosen_chocolate_bar_code) < 1){
        std::cout << "ERROR: code does not exist!\nPlease enter another bar code [A->C, 1->5]: "; 
      }
      else{
         std::cout << "ERROR: this bars stock out!\nPlease enter another bar code: ";  
      }
      std::cin >> chosen_chocolate_bar_code;  
    }
    tmp_pose = shelf_pose_map[chosen_chocolate_bar_code];
    return tmp_pose;
  }

  geometry_msgs::Pose compute_chosen_chocolate_bar_pose(geometry_msgs::Pose chosen_chocolate_bar_origin, std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to pick the chocolate bar correctly from the shelf, given the chosen chocolate bar origin and the desired orientation
    
    chosen_chocolate_bar_origin.position.x -= chocolate_bar_size[0] * 1/4;
    chosen_chocolate_bar_origin.position.z += shelf_basement_height/2;
    chosen_chocolate_bar_origin = set_RPY(chosen_chocolate_bar_origin, desired_RPY);  
    return chosen_chocolate_bar_origin;
  }

  geometry_msgs::Pose compute_vision_box_hole_pose(std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to place the chocolate bar correctly inside the vision box, given the desired orientation

    tmp_pose = set_RPY(vision_box_origin, desired_RPY);
    tmp_pose.position.x += chocolate_bar_size[0]*1/4;
    tmp_pose.position.z += ring_light_support_height/2;
    return tmp_pose;
  }

  geometry_msgs::Pose compute_portioning_machine_hole_pose(std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to place the chocolate bar correctly inside the portioning machine hole, given the desired orientation

    tmp_pose = set_RPY(portioning_machine_origin, desired_RPY);
    tmp_pose.position.x += portioning_machine_size[0]/2; 
    tmp_pose.position.z += portioning_machine_size[2]/2;
    //Add the hole offset (the difference along z-axis between the center of the portioning machine and its hole)
    tmp_pose.position.z += 0.06;
    return tmp_pose;
  }

  void build_chocolate_bars_map(){
    // Build the map variable to properly optain the chocolate bar origins, starting from class parameters
    // position.z = 0.0 corrispond on the top of the shelf support.

    int c=0;
    for (int j=2; j>=0; j--){ 
      for(int i=2; i>=-2; i--){
        tmp_pose = this->shelf_origin;
        tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
        tmp_pose.position.z += shelf_basement_height + chocolate_bar_size[2]/2;
        tmp_pose.position.y += 0.15*i;
        tmp_pose.position.z += 0.15*j;
        shelf_pose_map[chocolate_code_labels[c]] = tmp_pose;
        shelf_inventory_map[chocolate_code_labels[c]] = inventory[c];
        c++;
      }
    }
  }

  void compute_obstacle_primitives(){
    // Compute all the geometry primitives for the collision objects, starting from class parameters

    // Obstacle 1 - Table 
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.75;
    table_primitive.dimensions[1] = 0.75;
    table_primitive.dimensions[2] = 0.74;

    obstacle_primitives.push_back(table_primitive);

    // Obstacle 2 - Shelf  
    shape_msgs::SolidPrimitive shelf_primitive_fat;
    shape_msgs::SolidPrimitive shelf_primitive_thin;
    shelf_primitive_fat.type = shelf_primitive_fat.BOX;
    shelf_primitive_fat.dimensions.resize(3);
    // Vertical
    shelf_primitive_fat.dimensions[0] = shelf_size[0];
    shelf_primitive_fat.dimensions[1] = shelf_basement_height*2;
    shelf_primitive_fat.dimensions[2] = shelf_size[2];

    shelf_primitive_thin = shelf_primitive_fat;
    shelf_primitive_thin.dimensions[1] /= 2;

    //1 
    obstacle_primitives.push_back(shelf_primitive_thin);
    //2
    obstacle_primitives.push_back(shelf_primitive_fat);
    //3
    obstacle_primitives.push_back(shelf_primitive_fat);
    //4
    obstacle_primitives.push_back(shelf_primitive_thin);
    //5
    obstacle_primitives.push_back(shelf_primitive_fat);
    //6
    obstacle_primitives.push_back(shelf_primitive_fat);

    // Horizontal 
    shelf_primitive_fat.dimensions[0] = shelf_size[0];
    shelf_primitive_fat.dimensions[1] = shelf_size[1];
    shelf_primitive_fat.dimensions[2] = shelf_basement_height*2;

    shelf_primitive_thin = shelf_primitive_fat;
    shelf_primitive_thin.dimensions[2] /= 2; 

    //7
    obstacle_primitives.push_back(shelf_primitive_thin);
    //8
    obstacle_primitives.push_back(shelf_primitive_fat);
    //9
    obstacle_primitives.push_back(shelf_primitive_fat);
    //10
    obstacle_primitives.push_back(shelf_primitive_thin);

    // Obstacle 3 - vision box (two basement plus the ring light)
    //upper vision box cylinder
    shape_msgs::SolidPrimitive vision_box_primitive;
    vision_box_primitive.type = vision_box_primitive.CYLINDER;
    vision_box_primitive.dimensions.resize(1);
    vision_box_primitive.dimensions = {vision_box_base_height, vision_box_size[0]/2};

    obstacle_primitives.push_back(vision_box_primitive);

    //lower vision box cylinder
    obstacle_primitives.push_back(vision_box_primitive);

    //left rod
    vision_box_primitive.dimensions = {vision_box_rod_height, 0.008};
    obstacle_primitives.push_back(vision_box_primitive);

    //right rod
    vision_box_primitive.dimensions = {vision_box_rod_height, 0.008};
    obstacle_primitives.push_back(vision_box_primitive);

    //back rod
    obstacle_primitives.push_back(vision_box_primitive); 

    //ring light
    vision_box_primitive.dimensions = {ring_light_size[2], ring_light_size[0]/2};
    obstacle_primitives.push_back(vision_box_primitive);

    // Obstacle 4 - portioning machine
    //upper box
    shape_msgs::SolidPrimitive portioning_machine_primitive;
    portioning_machine_primitive.type = portioning_machine_primitive.BOX;
    portioning_machine_primitive.dimensions.resize(3);
    portioning_machine_primitive.dimensions[0] = portioning_machine_size[0];
    portioning_machine_primitive.dimensions[1] = portioning_machine_size[1];
    portioning_machine_primitive.dimensions[2] = 0.025;

    obstacle_primitives.push_back(portioning_machine_primitive);

    //lower box
    portioning_machine_primitive.dimensions[2] = 0.2;
    obstacle_primitives.push_back(portioning_machine_primitive); 

    //left box
    portioning_machine_primitive.dimensions[0] = portioning_machine_size[0];
    portioning_machine_primitive.dimensions[1] = 0.075;
    portioning_machine_primitive.dimensions[2] = 0.065;

    obstacle_primitives.push_back(portioning_machine_primitive);

    //right box
    obstacle_primitives.push_back(portioning_machine_primitive);

    // Obstacle 5 - Walls
    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);
    wall_primitive.dimensions[0] = 1.5;
    wall_primitive.dimensions[1] = 0.05;
    wall_primitive.dimensions[2] = 1.0;

    //back wall
    obstacle_primitives.push_back(wall_primitive);
    //front wall
    obstacle_primitives.push_back(wall_primitive);

    // Moving Object - Chocolate Bar
    chocolate_bar_primitive.type = chocolate_bar_primitive.BOX;
    chocolate_bar_primitive.dimensions.resize(3);
    chocolate_bar_primitive.dimensions[0] = chocolate_bar_size[0];
    chocolate_bar_primitive.dimensions[1] = chocolate_bar_size[1];
    chocolate_bar_primitive.dimensions[2] = chocolate_bar_size[2]/2; 
        
    for(int i=0; i<chocolate_code_labels.size() ; i++){
      if (inventory[i] > 0)
        obstacle_primitives.push_back(chocolate_bar_primitive);
    }
  }

  void compute_obstacle_poses(){
    // Compute all the poses for the collision objects, starting from class parameters

    // Obstacle 1 - Table  
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 0.0 - robot_pose.position.x;
    table_pose.position.y = 0.0 - robot_pose.position.y;
    table_pose.position.z = (table_primitive.dimensions[2]/2 - robot_pose.position.z);

    obstacle_poses.push_back(table_pose);

    // Obstacle 1 - Shelf 
    geometry_msgs::Pose tmp_shelf_pose = shelf_origin;
    //1 Vertical from Left
    tmp_shelf_pose.position.y += shelf_size[1]/2 - shelf_basement_height/2;
    tmp_shelf_pose.position.z += shelf_size[2]/2;
    obstacle_poses.push_back(tmp_shelf_pose);
    //2
    tmp_shelf_pose.position.y -= (0.15 - shelf_basement_height/2);
    obstacle_poses.push_back(tmp_shelf_pose);
    //3
    tmp_shelf_pose.position.y -= 0.15;
    obstacle_poses.push_back(tmp_shelf_pose);


    //4 Vertical from Right
    tmp_shelf_pose = shelf_origin;
    tmp_shelf_pose.position.y -= shelf_size[1]/2 + shelf_basement_height/2;
    tmp_shelf_pose.position.z += shelf_size[2]/2;
    obstacle_poses.push_back(tmp_shelf_pose);
    //5
    tmp_shelf_pose.position.y += (0.15 - shelf_basement_height/2);
    obstacle_poses.push_back(tmp_shelf_pose);
    //6
    tmp_shelf_pose.position.y += 0.15;
    obstacle_poses.push_back(tmp_shelf_pose);

    //7 Horizontal from bottom
    tmp_shelf_pose = shelf_origin;
    tmp_shelf_pose.position.z += shelf_basement_height/2;
    obstacle_poses.push_back(tmp_shelf_pose);
    //8
    tmp_shelf_pose.position.z += (0.15 - shelf_basement_height/2);
    obstacle_poses.push_back(tmp_shelf_pose);
    //9
    tmp_shelf_pose.position.z += 0.15;
    obstacle_poses.push_back(tmp_shelf_pose);
    //10
    tmp_shelf_pose = shelf_origin;
    tmp_shelf_pose.position.z += shelf_size[2];
    obstacle_poses.push_back(tmp_shelf_pose);

    // Obstacle 3 - vision box (two basement plus the ring light)
    geometry_msgs::Pose vision_box_lower_pose = vision_box_origin;
    vision_box_lower_pose.position.x -= robot_pose.position.x;
    vision_box_lower_pose.position.y -= robot_pose.position.y;
    vision_box_lower_pose.position.z -= robot_pose.position.z;
    vision_box_lower_pose.position.z += (vision_box_base_height/2 + vision_box_support_height);

    obstacle_poses.push_back(vision_box_lower_pose);  

    geometry_msgs::Pose vision_box_upper_pose = vision_box_origin;
    vision_box_upper_pose.position.x -= robot_pose.position.x;
    vision_box_upper_pose.position.y -= robot_pose.position.y;
    vision_box_upper_pose.position.z -= robot_pose.position.z;
    vision_box_upper_pose.position.z += (vision_box_size[2] + vision_box_support_height - vision_box_base_height/2);

    obstacle_poses.push_back(vision_box_upper_pose);  

    //left rod
    double vision_box_rod_height = vision_box_support_height + 0.13;

    geometry_msgs::Pose vision_box_left_rod_pose = vision_box_origin;
    vision_box_left_rod_pose.position.x -= robot_pose.position.x;
    vision_box_left_rod_pose.position.y -= robot_pose.position.y +0.12;
    vision_box_left_rod_pose.position.z -= robot_pose.position.z;
    vision_box_left_rod_pose.position.z += (vision_box_size[2]/2 + vision_box_support_height);

    obstacle_poses.push_back(vision_box_left_rod_pose);  

    //right rod
    geometry_msgs::Pose vision_box_right_rod_pose = vision_box_origin;
    vision_box_right_rod_pose.position.x -= robot_pose.position.x;
    vision_box_right_rod_pose.position.y -= robot_pose.position.y -0.12;
    vision_box_right_rod_pose.position.z -= robot_pose.position.z;
    vision_box_right_rod_pose.position.z += (vision_box_size[2]/2 + vision_box_support_height);

    obstacle_poses.push_back(vision_box_right_rod_pose);  

    //back rod
    geometry_msgs::Pose vision_box_back_rod_pose = vision_box_origin;
    vision_box_back_rod_pose.position.x -= robot_pose.position.x +0.12;
    vision_box_back_rod_pose.position.y -= robot_pose.position.y;
    vision_box_back_rod_pose.position.z -= robot_pose.position.z;
    vision_box_back_rod_pose.position.z += (vision_box_size[2]/2 + vision_box_support_height);

    obstacle_poses.push_back(vision_box_back_rod_pose);  

    //ring light
    geometry_msgs::Pose ring_light_pose = vision_box_origin;
    ring_light_pose.position.x -= robot_pose.position.x;
    ring_light_pose.position.y -= robot_pose.position.y;
    ring_light_pose.position.z -= robot_pose.position.z;
    ring_light_pose.position.z += (vision_box_support_height + ring_light_support_height + ring_light_size[2]/2);

    obstacle_poses.push_back(ring_light_pose);

    // Obstacle 4 - portioning machine
    //upper box
    geometry_msgs::Pose upper_portioning_machine_pose = portioning_machine_origin;
    upper_portioning_machine_pose.position.x -= robot_pose.position.x;
    upper_portioning_machine_pose.position.y -= robot_pose.position.y;
    upper_portioning_machine_pose.position.z -= robot_pose.position.z;
    upper_portioning_machine_pose.position.z += (portioning_machine_size[2] + portioning_machine_support_height) - 0.05;

    obstacle_poses.push_back(upper_portioning_machine_pose);  

    //lower box
    geometry_msgs::Pose lower_portioning_machine_pose = portioning_machine_origin;
    lower_portioning_machine_pose.position.x -= robot_pose.position.x;
    lower_portioning_machine_pose.position.y -= robot_pose.position.y;
    lower_portioning_machine_pose.position.z -= robot_pose.position.z;
    lower_portioning_machine_pose.position.z += (portioning_machine_size[2]/4 + portioning_machine_support_height);

    obstacle_poses.push_back(lower_portioning_machine_pose);  

    //left box
    geometry_msgs::Pose left_portioning_machine_pose = portioning_machine_origin;
    left_portioning_machine_pose.position.x -= robot_pose.position.x;
    left_portioning_machine_pose.position.y -= robot_pose.position.y - 0.1125;
    left_portioning_machine_pose.position.z -= robot_pose.position.z;
    left_portioning_machine_pose.position.z += (portioning_machine_size[2] + portioning_machine_support_height) -0.095;

    obstacle_poses.push_back(left_portioning_machine_pose);  

    //right box
    geometry_msgs::Pose right_portioning_machine_pose = portioning_machine_origin;
    right_portioning_machine_pose.position.x -= robot_pose.position.x;
    right_portioning_machine_pose.position.y -= robot_pose.position.y + 0.1125;
    right_portioning_machine_pose.position.z -= robot_pose.position.z;
    right_portioning_machine_pose.position.z += (portioning_machine_size[2] + portioning_machine_support_height) -0.095;

    obstacle_poses.push_back(right_portioning_machine_pose);  

    // Obstacle 5 - Walls
    geometry_msgs::Pose back_wall_origin;
    back_wall_origin.position.x = 0.0 - robot_pose.position.x;
    back_wall_origin.position.y = -0.55 - robot_pose.position.y;
    back_wall_origin.position.z = 1.0 - robot_pose.position.z;

    obstacle_poses.push_back(back_wall_origin);  

    geometry_msgs::Pose front_wall_origin;
    front_wall_origin.position.x = 0.0 - robot_pose.position.x;
    front_wall_origin.position.y = 0.55 - robot_pose.position.y;
    front_wall_origin.position.z = 1.0 - robot_pose.position.z;

    obstacle_poses.push_back(front_wall_origin);

    for(int i=0; i<chocolate_code_labels.size(); i++){
      if (inventory[i] > 0)
        obstacle_poses.push_back(compute_chosen_chocolate_bar_origin(chocolate_code_labels[i]));
    }
  }

  void compute_obstacle_names(){
    obstacle_names.push_back("Table");
    obstacle_names.push_back("Shelf_1");
    obstacle_names.push_back("Shelf_2");
    obstacle_names.push_back("Shelf_3");
    obstacle_names.push_back("Shelf_4");
    obstacle_names.push_back("Shelf_5");
    obstacle_names.push_back("Shelf_6");
    obstacle_names.push_back("Shelf_7");
    obstacle_names.push_back("Shelf_8");
    obstacle_names.push_back("Shelf_9");
    obstacle_names.push_back("Shelf_10");
    obstacle_names.push_back("vision_box_lower_base");
    obstacle_names.push_back("vision_box_upper_base");
    obstacle_names.push_back("vision_box_left_rod");
    obstacle_names.push_back("vision_box_right_rod");
    obstacle_names.push_back("vision_box_back_rod");
    obstacle_names.push_back("ring_light_base");
    obstacle_names.push_back("upper_box_portioning_machine");
    obstacle_names.push_back("lower_box_portioning_machine");
    obstacle_names.push_back("left_box_portioning_machine");
    obstacle_names.push_back("right_box_portioning_machine");
    obstacle_names.push_back("back_wall");
    obstacle_names.push_back("front_wall");

    for(int i=0; i<chocolate_code_labels.size(); i++){
      if (inventory[i] > 0)
        obstacle_names.push_back(chocolate_code_labels[i]);
    }
  }

};