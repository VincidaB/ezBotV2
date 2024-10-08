#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/empty.hpp>


#include <obstacle_msgs/msg/obstacles.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/circle.h"
#include "obstacle_detector/utilities/point_set.h"

namespace obstacle_detector
{

class PositionEstimator : public rclcpp::Node
{

public:

	PositionEstimator(rclcpp::NodeOptions options);
	~PositionEstimator();

private:
  bool updateParams(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void initialize() { std_srvs::srv::Empty::Request::SharedPtr empt;
											std_srvs::srv::Empty::Response::SharedPtr empt2;
											updateParams(empt, empt2); }

	void processObstacles();

	void obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr new_obstacles);

	rclcpp::Subscription<obstacle_msgs::msg::Obstacles>::SharedPtr obstacles_sub_;

	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_enemy_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_candidates_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_estimated_position_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_position_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;

  rclcpp::Time stamp_;

	// frame_id to transform to
	std::string p_frame_id_;
	//frame_id of the received obstacles (lidar frame)
	std::string base_frame_id_;

	std::list<Circle> circles_;
	std::list<Segment> segments_;

	// previously active
  bool p_active_;
	bool is_yellow_;
	double max_filter_distance_;

};

} // namespace obstacle_detector

#endif