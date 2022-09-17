#include <map>

class chocolate_portioner_env 
{
public:
  // VARIABILI DI CLASSE
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

  // COSTRUTTORE
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

    double grasping_x_offset = this->chocolate_bar_size[0] + 0.05; // 0.05 = shelf_spawn_x - chocolate_bar_spawn_x

    // A row
    tmp_pose = this->shelf_origin; 
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.30;
    tmp_pose.position.z += 0.30;
    shelf_pose_map["A1"] = tmp_pose;
    shelf_inventory_map["A1"] = inventory[0];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.30;
    shelf_pose_map["A2"] = tmp_pose;
    shelf_inventory_map["A2"] = inventory[1];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.z += 0.0;
    tmp_pose.position.z += 0.30; 
    shelf_pose_map["A3"] = tmp_pose;
    shelf_inventory_map["A3"] = inventory[2];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.30;
    shelf_pose_map["A4"] = tmp_pose;
    shelf_inventory_map["A4"] = inventory[3];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.35;
    shelf_pose_map["A5"] = tmp_pose;
    shelf_inventory_map["A5"] = inventory[4];

    // B row
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.30 ;
    tmp_pose.position.z += 0.15;
    shelf_pose_map["B1"] = tmp_pose;
    shelf_inventory_map["B1"] = inventory[5];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.15;
    shelf_pose_map["B2"] = tmp_pose;
    shelf_inventory_map["B2"] = inventory[6];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.0;
    tmp_pose.position.z += 0.15;
    shelf_pose_map["B3"] = tmp_pose;
    shelf_inventory_map["B3"] = inventory[7];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.15;
    shelf_pose_map["B4"] = tmp_pose;
    shelf_inventory_map["B4"] = inventory[8];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.15;
    shelf_pose_map["B5"] = tmp_pose;
    shelf_inventory_map["B5"] = inventory[9];

    // C row
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.30;
    tmp_pose.position.z += 0.0;
    shelf_pose_map["C1"] = tmp_pose;
    shelf_inventory_map["C1"] = inventory[10];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.15;
    tmp_pose.position.z += 0.0;
    shelf_pose_map["C2"] = tmp_pose;
    shelf_inventory_map["C2"] = inventory[11];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y += 0.0;
    tmp_pose.position.z += 0.0;
    shelf_pose_map["C3"] = tmp_pose;
    shelf_inventory_map["C3"] = inventory[12];

    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.15;
    tmp_pose.position.z += 0.0;
    shelf_pose_map["C4"] = tmp_pose;
    shelf_inventory_map["C4"] = inventory[13];
  
    tmp_pose = this->shelf_origin;
    tmp_pose.position.x -= grasping_x_offset;
    tmp_pose.position.y -= 0.30;
    tmp_pose.position.z += 0.0;
    shelf_pose_map["C5"] = tmp_pose;
    shelf_inventory_map["C5"] = inventory[14];
  };


  // Other Utilities

  geometry_msgs::Pose set_RPY(geometry_msgs::Pose pose, std::vector<double> desired_RPY){
    tf::Quaternion quaternion;
    quaternion.setRPY( desired_RPY[0], desired_RPY[1], desired_RPY[2]);
    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();
    return pose;
  }

  // METODI DI CLASSE

  geometry_msgs::Pose compute_chosen_chocolate_bar_pose(std::string chosen_chocolate_bar_code, std::vector<double> desired_RPY){

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
    tmp_pose.position.z += (shelf_inventory_map[chosen_chocolate_bar_code]-1)*chocolate_bar_size[2];
    shelf_inventory_map[chosen_chocolate_bar_code]--;

    tmp_pose.position.z += 0.02;  // 0.019 = basement height (but better be higher)
    tmp_pose = set_RPY(tmp_pose, desired_RPY);
    return tmp_pose;
  }

  geometry_msgs::Pose compute_vision_box_hole_pose(std::vector<double> desired_RPY){
    tmp_pose = set_RPY(vision_box_origin, desired_RPY);
    tmp_pose.position.x += vision_box_size[0]/2 + chocolate_bar_size[0]*2/4;
    tmp_pose.position.z += vision_box_size[2]/2;
    //TODO: Add the hole offset
    
    return tmp_pose;
  }

  geometry_msgs::Pose compute_portioning_machine_hole_pose(std::vector<double> desired_RPY){
    tmp_pose = set_RPY(portioning_machine_origin, desired_RPY);
    tmp_pose.position.x += portioning_machine_size[0]/2 + chocolate_bar_size[0]*3/4;
    tmp_pose.position.z += portioning_machine_size[2]/2;
    //Add the hole offset
    tmp_pose.position.z += 0.08;
    return tmp_pose;
  }
};