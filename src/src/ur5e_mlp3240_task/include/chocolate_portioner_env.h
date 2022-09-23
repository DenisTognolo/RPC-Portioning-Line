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

  std::map<std::string, geometry_msgs::Pose> shelf_pose_map;
  std::map<std::string, int> shelf_inventory_map;
  geometry_msgs::Pose tmp_pose;  

  geometry_msgs::Pose robot_pose;

  chocolate_portioner_env(geometry_msgs::Pose robot_pose, geometry_msgs::Pose shelf_origin, geometry_msgs::Pose vision_box_origin, geometry_msgs::Pose portioning_machine_origin, std::vector<double> shelf_size, std::vector<double> vision_box_size, std::vector<double> portioning_machine_size, std::vector<double> chocolate_bar_size, std::vector<int> inventory)
  {
    this->robot_pose =  robot_pose;
    this->shelf_origin = shelf_origin;
    this->vision_box_origin = vision_box_origin;
    this->portioning_machine_origin = portioning_machine_origin;

    this->shelf_size = shelf_size;
    this->vision_box_size = vision_box_size;
    this->portioning_machine_size = portioning_machine_size;
    this->chocolate_bar_size = chocolate_bar_size;

    if(inventory.size() != 15){
      std::cout << "Error: this enviroment must specify 15 stocks!" << std::endl;
    }
    // set origin.position.z at the base of the object (over the support) and in robot reference frame

    this->shelf_origin.position.x -= this->robot_pose.position.x;
    this->shelf_origin.position.y -= this->robot_pose.position.y;
    this->shelf_origin.position.z = this->shelf_origin.position.z - this->robot_pose.position.z + this->shelf_size[3];
    
    this->vision_box_origin.position.x -= this->robot_pose.position.x;
    this->vision_box_origin.position.y -= this->robot_pose.position.y;
    this->vision_box_origin.position.z = this->vision_box_origin.position.z - this->robot_pose.position.z + this->vision_box_size[3];

    this->portioning_machine_origin.position.x -= this->robot_pose.position.x;
    this->portioning_machine_origin.position.y -= this->robot_pose.position.y;
    this->portioning_machine_origin.position.z = this->portioning_machine_origin.position.z - this->robot_pose.position.z + this->portioning_machine_size[3];

    this->build_chocolate_bars_map(shelf_size[5], shelf_size[4], inventory);
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
    // Compute the pose that the robot must assume in order to grasp the desired chocolate bar, given it's code and the desired orientation

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
    shelf_inventory_map[chosen_chocolate_bar_code]--;
    return tmp_pose;
  }

  geometry_msgs::Pose compute_chosen_chocolate_bar_pose(geometry_msgs::Pose chosen_chocolate_bar_origin, std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to pick the chocolate bar correctly from the shelf, given the chosen chocolate bar origin and the desired orientation
    chosen_chocolate_bar_origin.position.x -= chocolate_bar_size[0] * 1/4;
    chosen_chocolate_bar_origin = set_RPY(chosen_chocolate_bar_origin, desired_RPY);  
    return chosen_chocolate_bar_origin;
  }

  geometry_msgs::Pose compute_vision_box_hole_pose(std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to place the chocolate bar correctly inside the vision box, given the desired orientation

    tmp_pose = set_RPY(vision_box_origin, desired_RPY);
    tmp_pose.position.x += chocolate_bar_size[0]*1/4;
    tmp_pose.position.z += vision_box_size[4]/2;
    return tmp_pose;
  }

  geometry_msgs::Pose compute_portioning_machine_hole_pose(std::vector<double> desired_RPY){
    // Compute the pose that the robot must assume in order to place the chocolate bar correctly inside the portioning machine hole, given the desired orientation

    tmp_pose = set_RPY(portioning_machine_origin, desired_RPY);
    tmp_pose.position.x += portioning_machine_size[0]/2; 
    tmp_pose.position.z += portioning_machine_size[2]/2;
    //Add the hole offset (the difference along z-axis between the center of the portioning machine and its hole)
    tmp_pose.position.z += 0.07;
    return tmp_pose;
  }

  void build_chocolate_bars_map(double shelf_to_chocolate_bar_origin_x_offset, double shelf_basement_height, std::vector<int> inventory){
    // Build the maps to properly optain the chocolate bars origins, given the shelf to chocolate bar origin offset (on x-axis), the shelf basement height and the quantity of each bars in the inventory
    // position.z = 0.0 corrispond on the top of the shelf support.

    // A row
    tmp_pose = this->shelf_origin; 
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.30;
    tmp_pose.position.z += 0.30 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["A1"] = tmp_pose;
    shelf_inventory_map["A1"] = inventory[0];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.30 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["A2"] = tmp_pose;
    shelf_inventory_map["A2"] = inventory[1];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.z += 0.0;
    tmp_pose.position.z += 0.30 + shelf_basement_height + chocolate_bar_size[2]/2; 
    shelf_pose_map["A3"] = tmp_pose;
    shelf_inventory_map["A3"] = inventory[2];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.30 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["A4"] = tmp_pose;
    shelf_inventory_map["A4"] = inventory[3];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.30 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["A5"] = tmp_pose;
    shelf_inventory_map["A5"] = inventory[4];

    // B row
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.30;
    tmp_pose.position.z += 0.15 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["B1"] = tmp_pose;
    shelf_inventory_map["B1"] = inventory[5];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.15 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["B2"] = tmp_pose;
    shelf_inventory_map["B2"] = inventory[6];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.0;
    tmp_pose.position.z += 0.15 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["B3"] = tmp_pose;
    shelf_inventory_map["B3"] = inventory[7];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.15 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["B4"] = tmp_pose;
    shelf_inventory_map["B4"] = inventory[8];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.15 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["B5"] = tmp_pose;
    shelf_inventory_map["B5"] = inventory[9];

    // C row
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.30;
    tmp_pose.position.z += 0.0 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["C1"] = tmp_pose;
    shelf_inventory_map["C1"] = inventory[10];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.0 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["C2"] = tmp_pose;
    shelf_inventory_map["C2"] = inventory[11];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y += 0.0;
    tmp_pose.position.z += 0.0 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["C3"] = tmp_pose;
    shelf_inventory_map["C3"] = inventory[12];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.0 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["C4"] = tmp_pose;
    shelf_inventory_map["C4"] = inventory[13];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= shelf_to_chocolate_bar_origin_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.0 + shelf_basement_height + chocolate_bar_size[2]/2;
    shelf_pose_map["C5"] = tmp_pose;
    shelf_inventory_map["C5"] = inventory[14];
  }
};