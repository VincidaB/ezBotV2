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

void PositionEstimator::processObstacles()
{
  // 1) filter out distant detections (as before)...
  circles_.erase(std::remove_if(
    circles_.begin(), circles_.end(),
    [&](const Circle& c){
      return std::hypot(c.center.x, c.center.y) > max_filter_distance_;
    }), circles_.end());

  // 1) Extract exactly three LiDAR detections into a small std::array:
std::array<Eigen::Vector2d,3> lidar_pts;
if (circles_.size() < 3) {
  RCLCPP_WARN(this->get_logger(), "Not enough circles detected, expected 3, got %zu", circles_.size());
  return; // Not enough points to estimate position
}
{
  auto it = circles_.begin();
  for(int i = 0; i < 3; ++i, ++it) {
    lidar_pts[i].x() = it->center.x;
    lidar_pts[i].y() = it->center.y;
  }
}

// 2) Prepare a permutation of the indices [0,1,2] to assign these to your
//    three known map‐points (yellow_poteaux_positions_[i].x / .y):
std::vector<int> perm = {0,1,2};

// 3) Variables to hold the best result:
Eigen::Matrix2d bestR;
Eigen::Vector2d best_t;
double best_error = std::numeric_limits<double>::infinity();

// 4) Try all 6 matchings:
do {
  // Build the two 2×3 matrices for this assignment:
  Eigen::Matrix<double,2,3> P_lidar, P_map;
  for(int j = 0; j < 3; ++j) {
    P_lidar.col(j) = lidar_pts[j];
    P_map  (0,j) = yellow_poteaux_positions_[ perm[j] ].x;
    P_map  (1,j) = yellow_poteaux_positions_[ perm[j] ].y;
  }

  // Compute centroids:
  Eigen::Vector2d cL = P_lidar.rowwise().mean();
  Eigen::Vector2d cM = P_map.  rowwise().mean();

  // Demean:
  Eigen::Matrix<double,2,3> Ql = P_lidar.colwise() - cL;
  Eigen::Matrix<double,2,3> Qm = P_map.  colwise() - cM;

  // Cross‐covariance:
  Eigen::Matrix2d H = Ql * Qm.transpose();

  // SVD:
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
    H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();

  // Rotation (enforce det=+1):
  Eigen::Matrix2d R = V * U.transpose();
  if (R.determinant() < 0) {
    V.col(1) *= -1;
    R = V * U.transpose();
  }

  // Translation:
  Eigen::Vector2d t = cM - R * cL;

  // Compute sum of squared residuals:
  double err = 0;
  for(int j = 0; j < 3; ++j) {
    Eigen::Vector2d pred = R * P_lidar.col(j) + t;
    err += (pred - P_map.col(j)).squaredNorm();
  }
  // RCLCPP_INFO(this->get_logger(), "Permutation %s: error = %f",
  //   std::to_string(perm[0]) + std::to_string(perm[1]) + std::to_string(perm[2]).c_str(), err);
  // Keep best:
  if (err < best_error) {
    best_error = err;
    bestR       = R;
    best_t      = t;
  }

} while(std::next_permutation(perm.begin(), perm.end()));

// RCLCPP_INFO(this->get_logger(), "Best permutation: %s, error = %f",
  // std::to_string(perm[0]) + std::to_string(perm[1]) + std::to_string(perm[2]).c_str(), best_error);

// 5) Extract heading from bestR:
double theta = std::atan2(bestR(1,0), bestR(0,0));

// 6) Now publish exactly as before, but using best_t:
nav_msgs::msg::Odometry odom;
odom.header.stamp    = stamp_;
odom.header.frame_id = p_frame_id_;        // e.g. "map"
odom.child_frame_id  = "robot1/odom";

odom.pose.pose.position.x = best_t.x();
odom.pose.pose.position.y = best_t.y();
odom.pose.pose.position.z = 0.0;
// TODO : actually evaluate and change these values
odom.pose.covariance[0] = best_error;  // x variance
odom.pose.covariance[7] = best_error;  // y variance
odom.pose.covariance[35] = best_error; // yaw variance // this makes no sense but I think it is still correlated with the error
tf2::Quaternion q;
q.setRPY(0, 0, theta);
odom.pose.pose.orientation = tf2::toMsg(q);

estimated_position_pub_->publish(odom);
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::PositionEstimator)

