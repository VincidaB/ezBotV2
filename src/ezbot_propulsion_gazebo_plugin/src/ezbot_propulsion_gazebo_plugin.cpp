#include "ezbot_propulsion_gazebo_plugin.hpp"

#include <fcntl.h>
namespace ezbot_propulsion_gazebo_plugin
{
ezbotPropulsionGazeboPlugin::ezbotPropulsionGazeboPlugin() : impl_(std::make_unique<ezbotPropulsionGazeboPluginPrivate>())
{

}

ezbotPropulsionGazeboPlugin::~ezbotPropulsionGazeboPlugin()
{

}

void ezbotPropulsionGazeboPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();


  // Dynamic properties
  impl_->max_wheel_accel_ = _sdf->Get<double>("wheelAccel", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("WheelTorque", 5.0).first;

  // Get joints and Kinematic properties
  gazebo::physics::JointPtr joint1, joint2, joint3;

  auto joint1_elem = _sdf->GetElement("North");
  auto joint2_elem = _sdf->GetElement("West");
  auto joint3_elem = _sdf->GetElement("South");

  auto joint1_name = joint1_elem->Get<std::string>();
  auto joint2_name = joint2_elem->Get<std::string>();
  auto joint3_name = joint3_elem->Get<std::string>();

  joint1 = _model->GetJoint(joint1_name);
  joint2 = _model->GetJoint(joint2_name);
  joint3 = _model->GetJoint(joint3_name);

  joint1->SetParam("fmax", 0, impl_->max_wheel_torque_);
  joint2->SetParam("fmax", 0, impl_->max_wheel_torque_);
  joint3->SetParam("fmax", 0, impl_->max_wheel_torque_);

  impl_->joints_.push_back(joint1);
  impl_->joints_.push_back(joint2);
  impl_->joints_.push_back(joint3);
  
  impl_->robot_radius_ = _sdf->Get<double>("robotRadius", 0.15).first;
  impl_->wheel_diameter_ = _sdf->Get<double>("wheelDiameter", 0.06).first;

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Robot radius set to [%fm]", impl_->robot_radius_);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Wheel diameter set to [%fm]", impl_->wheel_diameter_);

  impl_->steps_per_revolution_ = _sdf->Get<int>("stepsPerRevolution", 200).first;
  impl_->microstepping = _sdf->Get<int>("microstepping", 16).first;


  impl_->wheel_speed_instr_.assign(3, 0);
  impl_->desired_wheel_speed_.assign(3, 0);

  
  auto update_rate = _sdf->Get<double>("odometryRate", 20.0).first;
  if (update_rate > 0.0)
  {
    impl_->update_period_ = 1.0 / update_rate;
  }
  else
  {
    impl_->update_period_ = 0.0;
  }
  
  impl_->last_update_time_ = _model->GetWorld()->SimTime();
  

  

  

  // TODO remove the cmd_vel subscribtion
  //impl_->cmd_topic_ = _sdf->Get<std::string>("commandTopic", "cmd_vel").first;
  //impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(impl_->cmd_topic_, qos.get_subscription_qos(impl_->cmd_topic_, rclcpp::QoS(1)),
  //                                                                                        std::bind(&ezbotPropulsionGazeboPluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));
  
  //RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());
  
  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometryFrame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robotBaseFrame", "base_footprint").first;
  
  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publishOdom", false).first;
  if(impl_->publish_odom_)
  {
    impl_->odometry_topic_ = _sdf->Get<std::string>("odometryTopic", "odom").first;
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(impl_->odometry_topic_, qos.get_publisher_qos(impl_->odometry_topic_, rclcpp::QoS(1)));
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]", impl_->odometry_pub_->get_topic_name());
  }
  
  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publishWheelTF", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publishOdomTF", false).first;
  if(impl_->publish_wheel_tf_ || impl_->publish_odom_tf_)
  {
    impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
    if(impl_->publish_odom_tf_)
    {
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
                                                                                                      impl_->robot_base_frame_.c_str());
    }
    

  }

  impl_->isRollerModel_ = _sdf->Get<bool>("isRollerModel", true).first;
  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;


  // open the simulated sreial interface (generated privously with socat) (on /tmp/rppicoRX)

  impl_->input_serial_port = "/tmp/rppicoRX";
  impl_->output_serial_port = "/tmp/rppicoTX";
  
  impl_->serverSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (impl_->serverSocket == -1)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  //bind the socket to localhost port 12346
  sockaddr_in server_address{};
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(12346);
  server_address.sin_addr.s_addr = INADDR_ANY;

  if (bind(impl_->serverSocket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  if (listen(impl_->serverSocket, 3) < 0)
  {
    perror("listen");
    exit(EXIT_FAILURE);
  }

  sockaddr_in client_address{};
  socklen_t client_address_size = sizeof(client_address);

  impl_->clientSocket = accept(impl_->serverSocket, (struct sockaddr *)&client_address, &client_address_size);
  if (impl_->clientSocket < 0)
  {
    perror("accept");
    exit(EXIT_FAILURE);
  }



  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ezbotPropulsionGazeboPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void ezbotPropulsionGazeboPlugin::Reset()
{
  impl_->last_update_time_ = impl_->joints_[0]->GetWorld()->SimTime();
  
  
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;
}

void ezbotPropulsionGazeboPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
#endif
  
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  

  
  
  if(seconds_since_last_update < update_period_)
  {
    return;
  }
  

  //read what was sent to the socket
  char buffer[512] = {0};
  ssize_t n = read(clientSocket, buffer, sizeof(buffer)-1);
  if (n < 0)
  {
    RCLCPP_WARN(ros_node_->get_logger(), "Error reading from socket");
  }
  else if (n > 0)
  {
    //RCLCPP_INFO(ros_node_->get_logger(), "Received: %s", buffer);
  }


  switch (buffer[0])
  {
  case 'm':
    int motor_speeds[3];
    sscanf(buffer, "m %i %i %i", &motor_speeds[0], &motor_speeds[1], &motor_speeds[2]);
    // setting the motor speeds
    for (int i = 0; i < 3; i++)
    {
      //RCLCPP_INFO(ros_node_->get_logger(), "Setting motor %d to %d", i, motor_speeds[i]);
      //std::cout << "Setting motor " << i << " to " << motor_speeds[i] << std::endl;
      //impl_->steps_per_revolution_
      //impl_->microstepping
      double converted_motor_speed = motor_speeds[i] * 2 * M_PI / steps_per_revolution_ / microstepping;
     // RCLCPP_INFO(ros_node_->get_logger(), "Setting motor %d to %f", i, converted_motor_speed);
      
      joints_[i]->SetParam("vel", 0, converted_motor_speed);
    }
    //empty the buffer
    buffer[0] = 0;

    break;

  // send encode position value in meters
  case 'e':
    buffer[0] = 0;
    break;

  default:
    break;
  }
  




n = 0;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
  if(publish_odom_)
  {
    PublishOdometryMsg(_info.simTime);
  }
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
  //if(publish_wheel_tf_)
  //{
  //  PublishWheelsTf(_info.simTime);
  //}
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
  if(publish_odom_tf_)
  {
    PublishOdometryTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("UpdateWheelVelocities");
#endif
// Update robot in case new velocities have been requested
//UpdateWheelVelocities();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Current speed
  std::vector<double> current_speed(3);
  
  current_speed[0] = joints_[0]->GetVelocity(0);
  current_speed[1] = joints_[1]->GetVelocity(0);
  current_speed[2] = joints_[2]->GetVelocity(0);

  
  // upddated previously
  // joints_[2 * i + 0]->SetParam("vel", 0, desired_wheel_speed_[2 * i + 0]);
  // joints_[2 * i + 1]->SetParam("vel", 0, desired_wheel_speed_[2 * i + 1]);
  // joints_[2 * i + 2]->SetParam("vel", 0, desired_wheel_speed_[2 * i + 2]);
  // joints_[2 * i + EAST]->SetParam("vel", 0, desired_wheel_speed_[2 * i + EAST]);
  if(!isRollerModel_)
  {
    ignition::math::Pose3d pose = model_->WorldPose();
    float yaw = pose.Rot().Yaw();
    calkinematics(cal_LineVel_);
    model_->SetLinearVel(ignition::math::Vector3d(cal_LineVel_.vel_x * cosf(yaw) - cal_LineVel_.vel_y * sinf(yaw), 
                                                    cal_LineVel_.vel_y * cosf(yaw) + cal_LineVel_.vel_x * sinf(yaw), 0));
    model_->SetAngularVel(ignition::math::Vector3d(0, 0, cal_LineVel_.vel_th));
  }

  last_update_time_ = _info.simTime;
}

void ezbotPropulsionGazeboPluginPrivate::UpdateWheelVelocities()
{
 
}

//void ezbotPropulsionGazeboPluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
//{
//  std::lock_guard<std::mutex> scoped_lock(lock_);
//  target_x_ = _msg->linear.x;
//  target_y_ = _msg->linear.y;
//  target_rot_ = _msg->angular.z;
//}

void ezbotPropulsionGazeboPluginPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
  gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

//void ezbotPropulsionGazeboPluginPrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
//{
//  for (unsigned int i = 0; i < 4 * num_wheel_pairs_; ++i)
//  {
//    auto pose_wheel = joints_[i]->GetChild()->RelativePose();
//
//    geometry_msgs::msg::TransformStamped msg;
//    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
//    msg.header.frame_id = joints_[i]->GetParent()->GetName();
//    msg.child_frame_id = joints_[i]->GetChild()->GetName();
//    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
//    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());
//
//    transform_broadcaster_->sendTransform(msg);
//  }
//}

void ezbotPropulsionGazeboPluginPrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}

void ezbotPropulsionGazeboPluginPrivate::calkinematics(linear_vel &line_vel)
{
  double wheel_encoder[3];
  wheel_encoder[0] = joints_[0]->GetVelocity(0);
  wheel_encoder[1] = joints_[1]->GetVelocity(0);
  wheel_encoder[2] = joints_[2]->GetVelocity(0);

  double l = 1.0 / (4 * robot_radius_);

  line_vel.vel_x = (wheel_encoder[0] + wheel_encoder[1] + wheel_encoder[2] ) * wheel_diameter_ / 2;
  line_vel.vel_y = (wheel_encoder[0] - wheel_encoder[1] + wheel_encoder[2] ) * wheel_diameter_ / 2;
  line_vel.vel_th = (wheel_encoder[0] + wheel_encoder[1] + wheel_encoder[2] ) * l * wheel_diameter_ / 2;
}

GZ_REGISTER_MODEL_PLUGIN(ezbotPropulsionGazeboPlugin)
}  // namespace ezbot_propulsion_gazebo_plugin