#include "obstacle_detector/position_estimator.h"

using namespace std;
using namespace obstacle_detector;


PositionEstimator::PositionEstimator(rclcpp::NodeOptions options) : Node("position_estimator", options)
{
	p_active_ = false;
	

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  params_srv_ = create_service<std_srvs::srv::Empty>("params", std::bind(&PositionEstimator::updateParams, this, std::placeholders::_1, std::placeholders::_2));

  initialize();
}
PositionEstimator::~PositionEstimator(){

}


bool PositionEstimator::updateParams([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request, [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  bool prev_active = p_active_;

  this->declare_parameter("frame_id", "map");
  this->declare_parameter("active", true);
  this->declare_parameter("yellow", true);
  this->declare_parameter("max_filter_distance", 3.75);

  this->get_parameter("frame_id", p_frame_id_);
  this->get_parameter("active", p_active_);
  this->get_parameter("yellow", is_yellow_);
  this->get_parameter("max_filter_distance", max_filter_distance_);




  if (p_active_ != prev_active) {
    if (p_active_) {
			// publisher and subscriber first initialization
      RCLCPP_INFO(this->get_logger(), "Starting position_estimator node...");
			obstacles_sub_ = this->create_subscription<obstacle_msgs::msg::Obstacles>("raw_obstacles", 10, std::bind(&PositionEstimator::obstaclesCallback, this, std::placeholders::_1));
			marker_enemy_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("enemy_marker", 10);
			marker_candidates_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("position_candidates_marker", 10);
			marker_estimated_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("estimated_position_marker", 10);
      estimated_position_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("poteau_position", 10);
		}
    else {


      obstacles_sub_.reset();
      marker_enemy_pub_.reset();
      marker_candidates_pub_.reset();
      marker_estimated_position_pub_.reset();
      estimated_position_pub_.reset();
    }
  }

  return true;
}


void PositionEstimator::obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr new_obstacles){

  base_frame_id_ = new_obstacles->header.frame_id;
  // clear the previous circles and segments
  circles_.clear();
  segments_.clear();

  // get the current time
  stamp_ = this->now();

  // get the obstacles from the message
  for (auto& c : new_obstacles->circles){
    circles_.push_back(Circle(Point(c.center.x, c.center.y), c.radius));
    // RCLCPP_INFO(this->get_logger(), "Circle: %f, %f, %f", c.center.x, c.center.y, c.radius);
  }

  // process the obstacles
  processObstacles();
}

void PositionEstimator::processObstacles(){
  
  // find the 3 main poteaux 
  // They form an isosceles triangle with a short side of 1.9m and a long side of 3.144m
  
  // eliminate circles that are more than 3.75m away from ourselves
  double dist;
  for(auto& c : circles_){
    dist = c.center.x*c.center.x + c.center.y*c.center.y;
    if (sqrt(dist) > max_filter_distance_){
      circles_.remove(c);
    }
  }

  // find the 3 main poteaux

  // first find the two closest poteaux
  Circle poteau1;
  Circle poteau2;
  double closest_dist_diff = 1000;
  for (auto& c1 : circles_){
    for(auto& c2 : circles_){
      if (c1 == c2){
        continue;
      }
      dist = abs(sqrt((c1.center.x-c2.center.x)*(c1.center.x-c2.center.x)  + (c1.center.y-c2.center.y)*(c1.center.y-c2.center.y)) - 1.9);

      if (dist < closest_dist_diff){
        closest_dist_diff = dist;
        poteau1 = c1;
        poteau2 = c2;
      }
    }
  }


  // now find the third poteau

  Circle poteau3;
  closest_dist_diff = 1000;
  for (auto& c : circles_){
    if (c == poteau1 || c == poteau2){
      continue;
    }
    dist = abs(sqrt((c.center.x-poteau1.center.x)*(c.center.x-poteau1.center.x) + (c.center.y-poteau1.center.y)*(c.center.y-poteau1.center.y)) - 3.2843927901516285);
    // RCLCPP_INFO(this->get_logger(), "Dist: %f", dist);
    if (dist < closest_dist_diff){
      poteau3 = c;
      closest_dist_diff = dist;
    }
  }

  if (debug_)
    RCLCPP_INFO(this->get_logger(), "Poteau3: %f, %f, %f", poteau3.center.x, poteau3.center.y, poteau3.radius);

  // order poteau 1 and 2, so that poteau 1,2,3 is in a trigo order
  if (poteau1.center.cross(poteau2.center) < 0){
    Circle temp = poteau1;
    poteau1 = poteau2;
    poteau2 = temp;
  }


  // now we have the 3 poteaux, we can calculate the position of the robot
  // we estimate 3 time using the 3 poteaux and average the results
  
  double x1, y1, theta1;
  double x2, y2, theta2;
  double x3, y3, theta3;

  double dist1, dist2, dist3;
  // based on 1st and 2nd poteau
  if (debug_){
    RCLCPP_INFO(this->get_logger(), "Poteau1: %f, %f, %f", poteau1.center.x, poteau1.center.y, poteau1.radius);
    RCLCPP_INFO(this->get_logger(), "Poteau2: %f, %f, %f", poteau2.center.x, poteau2.center.y, poteau2.radius);
  }
  dist1 = poteau1.center.x*poteau1.center.x + poteau1.center.y*poteau1.center.y;
  dist2 = poteau2.center.x*poteau2.center.x + poteau2.center.y*poteau2.center.y;
  dist3 = poteau3.center.x*poteau3.center.x + poteau3.center.y*poteau3.center.y;

  y1 = - (dist1 - dist2 + 1.9*1.9)/(2*1.9) + 1 ;
  x1 = sqrt(dist2 - (1.9-(0.95-y1))*(1.9-(0.95-y1))) - 1.594 ;

  //based on 1st and third poteau

  double P1x = -1.594;
  double P1y = 0.95;
  double P2x = -1.594;
  double P2y = -0.95;
  double P3x = 1.594;


  double A = 2*(P2x-P1x);
  double B = 2*(P2y-P1y);
  double C = dist1-dist2 - P1x*P1x - P1y*P1y + P2x*P2x + P2y*P2y;
  double D = 2*(P3x-P2x);
  double E = 2*(-P2y);
  double F = dist2-dist3 - P2x*P2x - P2y*P2y + P3x*P3x ;


  x2 = (C*E - F*B)/(A*E - B*D);
  y2 = (C*D - A*F)/(B*D - A*E);


  if (debug_){
    RCLCPP_INFO(this->get_logger(), "x1: %f", x1);
    RCLCPP_INFO(this->get_logger(), "y1: %f", y1);
    
    RCLCPP_INFO(this->get_logger(), "x2: %f", x2);
    RCLCPP_INFO(this->get_logger(), "y2: %f", y2);
  }



  // RCLCPP_INFO(this->get_logger(), "dist1: %f", sqrt(dist1));
  // RCLCPP_INFO(this->get_logger(), "dist2: %f", sqrt(dist2));

  // publish markers for the poteaux

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_id_;
  marker.header.stamp = stamp_;
  marker.ns = "poteaux";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = poteau1.center.x;
  p.y = poteau1.center.y;
  marker.points.push_back(p);
  p.x = poteau2.center.x;
  p.y = poteau2.center.y;
  marker.points.push_back(p);
  p.x = poteau3.center.x;
  p.y = poteau3.center.y;
  marker.points.push_back(p);
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  
  marker_candidates_pub_->publish(marker);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp_;
  odom_msg.header.frame_id = "robot1/odom";

  odom_msg.pose.pose.position.x = x2;
  odom_msg.pose.pose.position.y = y2;
  odom_msg.pose.pose.position.z = 0.0;

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odom_msg.pose.covariance[diagonal_index] =
      0.01;
    // odom_msg.twist.covariance[diagonal_index] =
    //   odom_params_.twist_covariance_diagonal[index];
  }


  double add = 0;
  if (poteau1.center.x < 0 && poteau1.center.y < 0 || poteau1.center.x < 0 && poteau1.center.y > 0){
    add = 3.141592653;
  }

  //! this works but I am not sure why, I kept changing it until it worked
  theta1 = 3.141592653 - atan(poteau1.center.y / poteau1.center.x) - acos((1.594 + x1)/sqrt(dist1)) + add - 1;
  
  if (debug_){

    RCLCPP_INFO(this->get_logger(), "Dist1: %f", sqrt(dist1));
    RCLCPP_INFO(this->get_logger(), "Dist2: %f", sqrt(dist2));
  
    RCLCPP_INFO(this->get_logger(), "atan in : %f", poteau1.center.y / poteau1.center.x);
    RCLCPP_INFO(this->get_logger(), "atan: %f", atan(poteau1.center.y / poteau1.center.x));
    RCLCPP_INFO(this->get_logger(), "acos: %f", acos((1.594 + x1)/sqrt(dist1)));
  
    if (poteau1.center.x < 0){
      RCLCPP_INFO(this->get_logger(), "Negative x");
    }
  
    if (poteau1.center.y < 0){
      RCLCPP_INFO(this->get_logger(), "Negative y");
      
    }
    RCLCPP_INFO(this->get_logger(), "Theta: %f", theta1);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, theta1);
  odom_msg.pose.pose.orientation = tf2::toMsg(orientation);

  estimated_position_pub_->publish(odom_msg);

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::PositionEstimator)

