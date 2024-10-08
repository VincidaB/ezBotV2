/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "obstacle_detector/obstacle_extractor.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"

using namespace std;
using namespace obstacle_detector;

// ObstacleExtractor::ObstacleExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
ObstacleExtractor::ObstacleExtractor(rclcpp::NodeOptions options) : Node("obstacle_extractor", options) {
  p_active_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  params_srv_ = create_service<std_srvs::srv::Empty>("params", std::bind(&ObstacleExtractor::updateParams, this, std::placeholders::_1, std::placeholders::_2));
  initialize();
}

ObstacleExtractor::~ObstacleExtractor() {

}

// bool ObstacleExtractor::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
bool ObstacleExtractor::updateParams([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request, [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  bool prev_active = p_active_;


  this->declare_parameter("active", true);
  this->declare_parameter("use_scan", false);
  this->declare_parameter("use_pcl", true);

  this->declare_parameter("use_split_and_merge", true);
  this->declare_parameter("circles_from_visibles", true);
  this->declare_parameter("discard_converted_segments", true);
  this->declare_parameter("transform_coordinates", true);

  this->declare_parameter("min_group_points", 5);

  this->declare_parameter("max_group_distance", 0.1);
  this->declare_parameter("distance_proportion", 0.00628);
  this->declare_parameter("max_split_distance", 0.2);
  this->declare_parameter("max_merge_separation", 0.2);
  this->declare_parameter("max_merge_spread", 0.2);
  this->declare_parameter("max_circle_radius", 0.6);
  this->declare_parameter("radius_enlargement", 0.25);

  this->declare_parameter("min_x_limit", -10.0);
  this->declare_parameter("max_x_limit",  10.0);
  this->declare_parameter("min_y_limit", -10.0);
  this->declare_parameter("max_y_limit",  10.0);

  this->declare_parameter("frame_id", "map");

  this->get_parameter("active", p_active_);
  this->get_parameter("use_scan", p_use_scan_);
  this->get_parameter("use_pcl", p_use_pcl_);

  this->get_parameter("use_split_and_merge", p_use_split_and_merge_);
  this->get_parameter("circles_from_visibles", p_circles_from_visibles_);
  this->get_parameter("discard_converted_segments", p_discard_converted_segments_);
  this->get_parameter("transform_coordinates", p_transform_coordinates_);

  this->get_parameter("min_group_points", p_min_group_points_);

  this->get_parameter("max_group_distance", p_max_group_distance_);
  this->get_parameter("distance_proportion", p_distance_proportion_);
  this->get_parameter("max_split_distance", p_max_split_distance_);
  this->get_parameter("max_merge_separation", p_max_merge_separation_);
  this->get_parameter("max_merge_spread", p_max_merge_spread_);
  this->get_parameter("max_circle_radius", p_max_circle_radius_);
  this->get_parameter("radius_enlargement", p_radius_enlargement_);

  this->get_parameter("min_x_limit", p_min_x_limit_);
  this->get_parameter("max_x_limit", p_max_x_limit_);
  this->get_parameter("min_y_limit", p_min_y_limit_);
  this->get_parameter("max_y_limit", p_max_y_limit_);

  this->get_parameter("frame_id", p_frame_id_);

  if (p_active_ != prev_active) {
    if (p_active_) {
      if (p_use_scan_)
        // scan_sub_ = nh_.subscribe("scan", 10, &ObstacleExtractor::scanCallback, this);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleExtractor::scanCallback, this, std::placeholders::_1));
      else if (p_use_pcl_)
        // pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleExtractor::pclCallback, this);
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pcl", 10, std::bind(&ObstacleExtractor::pclCallback, this, std::placeholders::_1));

      // obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("raw_obstacles", 10);
      obstacles_pub_ = this->create_publisher<obstacle_msgs::msg::Obstacles>("raw_obstacles", 10);
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("raw_obstacles_marker", 10);
      marker_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("raw_lines_marker", 10);
    }
    else {
      
      RCLCPP_INFO(this->get_logger(), "Shutting down obstacle_extractor node...");
      // Send empty message
      // obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
      obstacle_msgs::msg::Obstacles::SharedPtr obstacles_msg = std::make_shared<obstacle_msgs::msg::Obstacles>();
      visualization_msgs::msg::Marker::SharedPtr marker_sphere_msg = std::make_shared<visualization_msgs::msg::Marker>();
      visualization_msgs::msg::Marker::SharedPtr marker_line_msg = std::make_shared<visualization_msgs::msg::Marker>();

      obstacles_msg->header.frame_id = p_frame_id_;
      
      // obstacles_msg->header.stamp = ros::Time::now();
      // obstacles_pub_.publish(obstacles_msg);

      marker_sphere_msg->header.frame_id = p_frame_id_;
      marker_line_msg->header.frame_id = p_frame_id_;


      obstacles_msg->header.stamp = this->now();
      obstacles_pub_->publish(*obstacles_msg);
      marker_pub_->publish(*marker_sphere_msg);

      // scan_sub_.shutdown();
      // pcl_sub_.shutdown();
      // obstacles_pub_.shutdown();
      scan_sub_.reset();
      pcl_sub_.reset();
      obstacles_pub_.reset();
      marker_pub_.reset();
      marker_line_pub_.reset();
    }
  }

  return true;
}

// void ObstacleExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg) {
void ObstacleExtractor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  base_frame_id_ = scan_msg->header.frame_id;
  // stamp_ = scan_msg->header.stamp;
  stamp_ = this->now();

  double phi = scan_msg->angle_min;

  for (const float r : scan_msg->ranges) {
    if (r >= scan_msg->range_min && r <= scan_msg->range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan_msg->angle_increment;
  }

  processPoints();
}

// void ObstacleExtractor::pclCallback(const sensor_msgs::PointCloud2::ConstPtr pcl_msg) {
//   base_frame_id_ = pcl_msg->header.frame_id;
//   stamp_ = pcl_msg->header.stamp;

//   for (const geometry_msgs::Point32& point : pcl_msg->points)
void ObstacleExtractor::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
  base_frame_id_ = pcl_msg->header.frame_id;
  stamp_ = this->now();

  input_points_.push_back(Point(point.x, point.y));

  processPoints();
}

void ObstacleExtractor::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPoints();  // Grouping points simultaneously detects segments
  //mergeSegments();

  detectCircles();
  mergeCircles();

  publishObstacles();

  input_points_.clear();
}

void ObstacleExtractor::groupPoints() {
  static double sin_dp = sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin = input_points_.begin();
  point_set.end = input_points_.begin();
  point_set.num_points = 1;
  point_set.is_visible = true;

  int i =0;
  visualization_msgs::msg::Marker marker_point_sets_msg;
  marker_point_sets_msg.header.frame_id = base_frame_id_;
  // marker_point_sets_msg.header.stamp = stamp_;
  // marker_point_sets_msg.action =  visualization_msgs::msg::Marker::DELETE_ALL;
  // marker_point_sets_msg.action =  3;
  // marker_pub_->publish(marker_point_sets_msg);

  for (PointIterator point = input_points_.begin()++; point != input_points_.end(); ++point) {
    double range = (*point).length();
    double distance = (*point - *point_set.end).length();

    // RCLCPP_INFO(this->get_logger(), "range: %f, distance: %f, sin_dp: %f", range, distance, sin_dp);

    if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
      point_set.end = point;
      point_set.num_points++;

      geometry_msgs::msg::Point p;
      p.x = (*point).x;
      p.y = (*point).y;

      // clearing the points 
      if(point_set.num_points == 1){
        marker_point_sets_msg.points.clear();
      }

      marker_point_sets_msg.points.push_back(p);
    }
    else {
      double prev_range = (*point_set.end).length();

      // Heron's equation
      double p = (range + prev_range + distance) / 2.0;
      double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
      double sin_d = 2.0 * S / (range * prev_range); // Sine of angle between beams

      // TODO: This condition can be fulfilled if the point are on the opposite sides
      // of the scanner (angle = 180 deg). Needs another check.
      if (abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);



      marker_point_sets_msg.header.stamp = stamp_;
      marker_point_sets_msg.ns = "point_sets";
      marker_point_sets_msg.type = visualization_msgs::msg::Marker::POINTS;

      marker_point_sets_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_point_sets_msg.id = i;
      i++;

      if (i % 2 == 0) {
        marker_point_sets_msg.color.b = 1.0;  // Blue
        marker_point_sets_msg.color.g = 1.0;  // Cyan
      } else {
        marker_point_sets_msg.color.b = 1.0;  // Blue
        marker_point_sets_msg.color.g = 0.0;  // Blue
      }
      marker_point_sets_msg.color.a = 1.0;
      marker_point_sets_msg.scale.x = 0.01;
      marker_point_sets_msg.scale.y = 0.01;
      marker_point_sets_msg.scale.z = 0.01;


      // marker_pub_->publish(marker_point_sets_msg);
      
      
      // Begin new point set
      point_set.begin = point;
      point_set.end = point;
      point_set.num_points = 1;
      point_set.is_visible = (abs(sin_d) > sin_dp || range < prev_range);
    }
  }
  
  marker_point_sets_msg.header.stamp = stamp_;
  marker_point_sets_msg.ns = "point_sets";
  marker_point_sets_msg.type = visualization_msgs::msg::Marker::POINTS;

  marker_point_sets_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_point_sets_msg.id = i;
  i++;

  marker_point_sets_msg.color.b = 1.0;
  marker_point_sets_msg.color.a = 1.0;
  marker_point_sets_msg.scale.x = 0.01;
  marker_point_sets_msg.scale.y = 0.01;
  marker_point_sets_msg.scale.z = 0.01;


  // marker_pub_->publish(marker_point_sets_msg);
  
  


  detectSegments(point_set); // Check the last point set too!
}

void ObstacleExtractor::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_)
    return;

  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // Natural index of splitting point (counting from 1)
  int point_index = 0; // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
  }
}

void ObstacleExtractor::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        RCLCPP_INFO(this->get_logger(), "Merging segments");
        i = --temp_itr; // Check the new segment against others
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2) {
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible)
        continue;
    }

    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleExtractor::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleExtractor::publishObstacles() {
  // obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
  // obstacles_msg->header.stamp = stamp_;
  obstacle_msgs::msg::Obstacles obstacles_msg;
  obstacles_msg.header.stamp = stamp_;

  visualization_msgs::msg::Marker marker_sphere_msg, marker_line_msg;
  marker_sphere_msg.header.stamp = stamp_;
  marker_sphere_msg.ns = "obstacles";

  marker_line_msg.ns = "lineobstacles";

  if (p_transform_coordinates_) {
    // tf::StampedTransform transform;
    tf2::Stamped<tf2::Transform> transform;


    try {
      tf_buffer_->lookupTransform(p_frame_id_, base_frame_id_, stamp_, rclcpp::Duration(0,100000000));
    }
    // catch (tf::TransformException& ex) {
    //   ROS_INFO_STREAM(ex.what());
    //   return;
    // }
    catch (tf2::TransformException& ex) {
      RCLCPP_INFO_STREAM(get_logger(), ex.what());
      return;
    }

    for (Segment& s : segments_) {
      s.first_point = transformPoint(s.first_point, transform);
      s.last_point = transformPoint(s.last_point, transform);
    }

    for (Circle& c : circles_)
      c.center = transformPoint(c.center, transform);

    // obstacles_msg->header.frame_id = p_frame_id_;
    obstacles_msg.header.frame_id = p_frame_id_;
    marker_sphere_msg.header.frame_id = p_frame_id_;
    marker_line_msg.header.frame_id = p_frame_id_;
  }
  else{
    obstacles_msg.header.frame_id = base_frame_id_;
    marker_sphere_msg.header.frame_id = base_frame_id_;
    marker_line_msg.header.frame_id = base_frame_id_;


  }
  // obstacles_msg->header.frame_id = base_frame_id_;


  marker_line_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_line_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_line_msg.id = 0;
  marker_line_msg.scale.x = 0.025;
  marker_line_msg.color.g = 1.0;
  marker_line_msg.color.a = 0.5;
  geometry_msgs::msg::Point p;
  int i = 0;

  for (const Segment& s : segments_) {
    // SegmentObstacle segment;
    obstacle_msgs::msg::SegmentObstacle segment;

    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;

    // obstacles_msg->segments.push_back(segment);
    obstacles_msg.segments.push_back(segment);

    


    p.x = s.first_point.x;
    p.y = s.first_point.y;

    marker_line_msg.points.push_back(p);
    
    p.x = s.last_point.x;
    p.y = s.last_point.y;

    marker_line_msg.points.push_back(p);
    // RCLCPP_INFO(this->get_logger(), "Segment: %f, %f, %f, %f", s.first_point.x, s.first_point.y, s.last_point.x, s.last_point.y);
    // RCLCPP_INFO(this->get_logger(), "seg number %d", i); 
    i++;

  }

  marker_line_msg.header.stamp = stamp_;
  // RCLCPP_INFO(this->get_logger(), "Publishing line markers");
  marker_line_pub_->publish(marker_line_msg);
  
  //marker_line_msg.color.a = 0.0;
  //marker_line_msg.id = 2;
  //marker_pub_->publish(marker_line_msg);


  //marker_sphere_msg.type = visualization_msgs::msg::Marker::SPHERE;
  i = 0;
  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        // CircleObstacle circle;
        obstacle_msgs::msg::CircleObstacle circle;

        circle.center.x = c.center.x;
        circle.center.y = c.center.y;
        circle.velocity.x = 0.0;
        circle.velocity.y = 0.0;
        circle.radius = c.radius;
        circle.true_radius = c.radius - p_radius_enlargement_;

        // obstacles_msg->circles.push_back(circle);
        obstacles_msg.circles.push_back(circle);
       
        //marker_sphere_msg.points.push_back(circle.center);
        marker_sphere_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_sphere_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_sphere_msg.pose.position.x = c.center.x;
        marker_sphere_msg.pose.position.y = c.center.y;
        marker_sphere_msg.id = i;
        i++;

        marker_sphere_msg.scale.x = (c.radius - p_radius_enlargement_)*2;
        marker_sphere_msg.scale.y = (c.radius - p_radius_enlargement_)*2;
        marker_sphere_msg.scale.z = (c.radius - p_radius_enlargement_)*2;
      
        marker_sphere_msg.color.r = 0.0;
        marker_sphere_msg.color.g = 1.0; 
        marker_sphere_msg.color.b = 0.0;
        marker_sphere_msg.color.a = 0.5;
        
        marker_pub_->publish(marker_sphere_msg);
    }



  }
  marker_sphere_msg.type = visualization_msgs::msg::Marker::SPHERE;
  marker_sphere_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_sphere_msg.pose.position.x = 0.0;
  marker_sphere_msg.pose.position.y = 0.0;
  marker_sphere_msg.id = i;
  
  marker_sphere_msg.scale.x = 0.01;
  marker_sphere_msg.scale.y = 0.01;
  marker_sphere_msg.scale.z = 0.01;

  marker_sphere_msg.color.r = 1.0;
  marker_sphere_msg.color.g = 0.0; 
  marker_sphere_msg.color.b = 1.0;
  marker_sphere_msg.color.a = 0.5;
  // publishing another point because rviz seem to have a bug where the last point is not shown
   marker_pub_->publish(marker_sphere_msg);


  obstacles_pub_->publish(obstacles_msg);

  

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::ObstacleExtractor)
