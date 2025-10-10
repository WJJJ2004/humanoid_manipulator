#include "node/main_node.hpp"
#include <iostream>

#define GRIPPER_OPEN_RAD -0.98017690792001511
#define GRIPPER_CLOSE_RAD 0.24022122533307867
#define FIRST_STEP 2

using std::placeholders::_1;
namespace SRCIRC2025_HUMANOID_LOCOMOTION
{
GripperMainNode::GripperMainNode(const std::string& urdf_path, const std::string& srdf_path, const std::string& AUTO_CTRL)
: Node("gripper_main_node")
{
  // ROS PARAMETER
  this->declare_parameter<int>("MAX_ITERATIONS", 200);
  this->declare_parameter<double>("POSITION_TOLERANCE", 1e-4);
  this->declare_parameter<double>("SE3_TOLERANCE", 1e-4);
  this->declare_parameter<double>("ALPHA", 0.2);
  this->declare_parameter<double>("LAMBDA", 1e-3);
  this->declare_parameter<double>("STEP_MAX", 0.05);
  this->declare_parameter<bool>("USE_REFERENCE", true);
  this->declare_parameter<std::string>("REF_NAME", "home");
  this->declare_parameter<bool>("IK_TILT_ONLY", true);
  this->declare_parameter<double>("YAW_WEIGHT", 0.0);
  this->declare_parameter<double>("ROLL_WEIGHT", 0.0);
  this->declare_parameter<double>("PITCH_WEIGHT", 0.0);
  this->declare_parameter<std::string>("BASE_FRAME", "base_link");
  this->declare_parameter<std::string>("CAMERA_FRAME", "eye_1");
  this->declare_parameter<bool>("USE_LATEST_TF", true);
  this->declare_parameter<double>("TF_TIMEOUT_SEC", 0.1);
  this->declare_parameter<int>("TF_THROTTLE_MS", 2000);
  this->declare_parameter<bool>("DEBUG_TF", false);
  this->declare_parameter<double>("WAYPOINT_D", 0.035);
  this->declare_parameter<double>("GRAVITY_OFFSET_ROLL", 0.0);
  this->declare_parameter<std::string>("MOTION_PATH", "/motion/grip.yaml");
  this->declare_parameter<bool>("DEBUG_VISUALIZATION", false);
  this->declare_parameter<double>("TARGETPOINT_D", 0.0);

  // IK MODULE INITIALIZE
  ik_ = std::make_shared<IKModule>(urdf_path, srdf_path, "base_link", "ee_link_1");
  this->getParamsFromRos();                       // 파라미터 로드   >> 위치 수정 X
  q_current_ = ik_->initialConfiguration();       // 시드 자세

  // CONTROL MODE SET
  if(AUTO_CTRL == "teleop")
  {
    ctrl_mode_ = ControlMode::Teleop;
    debug_mode_ = true;
    RCLCPP_INFO(get_logger(), "Control Mode: Teleop (Direct control, Debug TF enabled)");
  }
  else if(AUTO_CTRL == "auto")
  {
    ctrl_mode_ = ControlMode::Auto;
    debug_mode_ = false;
    RCLCPP_INFO(get_logger(), "Control Mode: Auto (Debug TF disabled)");
  }
  else if(AUTO_CTRL == "debug")
  {
    ctrl_mode_ = ControlMode::Auto;
    debug_mode_ = true;
    RCLCPP_INFO(get_logger(), "Control Mode: Auto (Debug TF enabled)");
  }
  else
  {
    ctrl_mode_ = ControlMode::Teleop;
    debug_mode_ = true;
    RCLCPP_ERROR(get_logger(), "Unknown control mode string '%s'. Defaulting to Teleop.", AUTO_CTRL.c_str());
  }

  // IK MODULE DEBUG MODE SET
  ik_->setDebugIKMode(debug_mode_);
  if (debug_mode_) {
    RCLCPP_WARN(get_logger(), "Debug IK mode enabled: printing SRDF status.");
    ik_->printSRDFStatus();
  }

  // MOTION EDITOR INITIALIZE
  yaml_path = share_dir + motion_path;
  try {
    motion_editor_ = std::make_shared<MotionEditor>();
    motion_editor_->loadFromFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "ERR: " << e.what() << "\n";
  }

  // TF INITIALIZE
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, true);

  // PUSBLISHER & SUBSCRIBER
  pub_joints_     = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);             // RVIZ2
  pub_collision_  = create_publisher<std_msgs::msg::Bool>(
    "/self_collision", 10);           // SELF-COLLISION FLAG
  pub_ee_current_ = create_publisher<geometry_msgs::msg::Point>(
    "/ee_current_position", 10);      // 타이머 콜백 바인딩 현재 EE 위치 퍼블리시 (FOR RVIZ2 TEST)
  pub_ctrl_flg_   = create_publisher<std_msgs::msg::Bool>(
    "/mani2master", 10);              // CONTROL FLAG

  if(debug_mode_) {
    pub_com_marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/com_marker", 10);
  }
  if (ctrl_mode_ == ControlMode::Auto)
  {
    RCLCPP_INFO(get_logger(), "Auto control mode: transforming /ee_target_position from camera frame to base_link frame.");

    if(debug_mode_)
    {
      RCLCPP_WARN(get_logger(), "Debug TF enabled: subscribing to /vision topic.");
      vision_sub_ = create_subscription<intelligent_humanoid_interfaces::msg::Vision2MasterMsg>(
        "/vision", 10,
        std::bind(&GripperMainNode::onVision, this, _1));
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Debug TF disabled. Subscribing to /master2mani topic.");
      sub_master_request_ = create_subscription<geometry_msgs::msg::Point32>(
        "/master2mani", 10,
        std::bind(&GripperMainNode::onMasterRequest, this, _1));
    }

    tilt_sub_ = create_subscription<dynamixel_rdk_msgs::msg::DynamixelMsgs>(
      "/pan_dxl", 10,
      [this](const dynamixel_rdk_msgs::msg::DynamixelMsgs::SharedPtr msg)
      {
        tilt_angle = msg->goal_position * M_PI / 180.0; // degree to rad
      });
  }
  else  //  Teleop 모드
  {
    RCLCPP_INFO(get_logger(), "Teleop mode: using /ee_target_position as-is (base_link frame).");

    sub_target_ = create_subscription<geometry_msgs::msg::Point>(
      "/ee_target_position", 10,
      std::bind(&GripperMainNode::target_cmd_callback, this, _1));

    sub_load_param_cmd_ = create_subscription<std_msgs::msg::Bool>(
      "/telop_cmd_load", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          RCLCPP_INFO(get_logger(), "Loading parameters from ROS parameters...");
          getParamsFromRos();
        }
      });
  }

  auto period = std::chrono::duration<double>(1.0 / 33.0);
  timer_pub = this->create_wall_timer
  (
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]()
    {
      if (ctrl_mode_ == ControlMode::Teleop)
      {
        const Eigen::Vector3d p = ik_->currentEEPose(q_current_).translation();
        geometry_msgs::msg::Point ee_msg;
        ee_msg.x = p.x();
        ee_msg.y = p.y();
        ee_msg.z = p.z();
        pub_ee_current_->publish(ee_msg);
      }

      // Joint State msg initialize
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      js.name = {"rotate_torso", "rotate_1", "rotate_3", "rotate_5", "rotate_tilt"};
      js.position.reserve(q_current_.size());

      if(debug_visualization_)
      {
        // 512 state for debug
        for (int i = 0; i < q_current_.size() - 1; ++i) js.position.push_back(0.0);
        js.position.push_back(tilt_angle);
        pub_joints_->publish(js);
      }
      else
      {
        for (int i = 0; i < q_current_.size() - 1; ++i) js.position.push_back(q_current_[i]);
        js.position.push_back(tilt_angle);
        pub_joints_->publish(js);
      }
    }
  );
  RCLCPP_INFO(get_logger(), "GripperMainNode ready. Waiting for /ee_target_position ...");
}

void GripperMainNode::onMasterRequest(const geometry_msgs::msg::Point32::SharedPtr msg)
{
  if (!msg)
  {
    RCLCPP_ERROR(get_logger(), "onMasterRequest: received null msg");
    return;
  }
  if (camera_frame_.empty() || base_frame_.empty())
  {
    RCLCPP_ERROR(get_logger(), "onMasterRequest: frame id empty (camera='%s', base='%s')",
                 camera_frame_.c_str(), base_frame_.c_str());
    return;
  }
  if (!tf_buffer_)
  {
    RCLCPP_ERROR(get_logger(), "onMasterRequest: tf_buffer_ is null (did you construct it and keep a member TransformListener alive?)");
    return;
  }

  if(!is_master_request)
  {
    RCLCPP_INFO(get_logger(), "onMasterRequest: Received target: x=%.3f, y=%.3f, z=%.3f",
                msg->x, msg->y, msg->z);
    is_master_request = true;
  }
  else
  {
    RCLCPP_WARN(get_logger(), "onMasterRequest: Previous request is still being processed. Ignoring this request.");
    return; // 이전 요청이 아직 처리 중이면 무시
  }

  geometry_msgs::msg::PointStamped in_eye;
  in_eye.header.frame_id = camera_frame_;
  in_eye.header.stamp    = this->now();

  // mm to m, camera frame to eye frame 변환
  in_eye.point.x =  0.001 * msg->z;
  in_eye.point.y = -0.001 * msg->x;
  in_eye.point.z = -0.001 * msg->y;

  geometry_msgs::msg::PointStamped out_base;

  bool tf_status_flag = transformPointToBase(in_eye, out_base);
  if (!tf_status_flag)
  {
    RCLCPP_ERROR(get_logger(), "onMasterRequest: transformPointToBase failed");
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "onMasterRequest: IN EYE target: x=%.3f, y=%.3f, z=%.3f",
                in_eye.point.x, in_eye.point.y, in_eye.point.z);
    RCLCPP_INFO(get_logger(), "onMasterRequest: Transformed target: x=%.3f, y=%.3f, z=%.3f",
                out_base.point.x, out_base.point.y, out_base.point.z);
  }

  // 현재 YAW 기준으로 base link Frame에서 World Frame으로 좌표 변환
  Eigen::Vector3d pB(out_base.point.x, out_base.point.y, out_base.point.z);
  Eigen::Vector3d t_pivot(0.0, 0.0, 0.0);

  const double yaw = q_current_[0];
  const double c = std::cos(-yaw);
  const double s = std::sin(-yaw);

  Eigen::Matrix3d R;
  R << c, -s, 0,
      s,  c, 0,
      0,  0, 1;

  Eigen::Vector3d pW0 = R * pB;

  // 중력 방향 벡터 계산 (롤 오프셋 적용)
  Eigen::Vector3d unit_vertical_vector = {-1.0 * cos(gravity_offset_roll), 0.0, sin(gravity_offset_roll)};
  Eigen::Vector3d target_dir = unit_vertical_vector * targetpoint_d;
  Eigen::Vector3d interp_dir = unit_vertical_vector * waypoint_d;


  auto target_pos = std::make_shared<geometry_msgs::msg::Point>();
  target_pos->x = pW0.x() + target_dir.x();
  target_pos->y = pW0.y() + target_dir.y();
  target_pos->z = pW0.z() + target_dir.z();

  auto interp_pos = std::make_shared<geometry_msgs::msg::Point>();
  // 중력 방향 벡터 기준 waypoint_D 지점에 경유점 생성
  interp_pos->x = pW0.x() + interp_dir.x();
  interp_pos->y = pW0.y() + interp_dir.y();
  interp_pos->z = pW0.z() + interp_dir.z();

  std_msgs::msg::Bool ctrl_flg;

  try {
    bindTargetToMotion(interp_pos, 2);   // 접근 포즈 + 그리퍼 개발
    bindTargetToMotion(target_pos, 3);   // 타겟 지점 + 그리퍼 개방
    bindTargetToMotion(target_pos, 4);   // 경유점   + 그리퍼 닫힘
    bindTargetToMotion(interp_pos, 5);   // 경유점 + 그리퍼 닫힘
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "[OMR]: bindTargetToMotion exception: %s", e.what());
    is_master_request = false;
    ctrl_flg.data = false;
    pub_ctrl_flg_->publish(ctrl_flg);
    return;
  }

  is_master_request = false;
  ctrl_flg.data = true;
  pub_ctrl_flg_->publish(ctrl_flg);
  RCLCPP_INFO(get_logger(), "onMasterRequest: Published control flag to /mani2master.");
  RCLCPP_INFO(get_logger(), "onMasterRequest: Processed target: x=%.3f, y=%.3f, z=%.3f",
              target_pos->x, target_pos->y, target_pos->z);
  RCLCPP_INFO(get_logger(), "onMasterRequest: Interpolated waypoint: x=%.3f, y=%.3f, z=%.3f",
              interp_pos->x, interp_pos->y, interp_pos->z);
}

void GripperMainNode::pubBallMarker(const Eigen::Vector3d& com)
{
  if (!pub_com_marker_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_;
  marker.header.stamp = this->now();
  marker.ns = "COM";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = com.x();
  marker.pose.position.y = com.y();
  marker.pose.position.z = com.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 0.1f;
  marker.color.b = 0.1f;
  marker.color.a = 0.9f;
  marker.lifetime = rclcpp::Duration(0, 0); // 지속

  pub_com_marker_->publish(marker);
}

void GripperMainNode::onVision(const intelligent_humanoid_interfaces::msg::Vision2MasterMsg::SharedPtr msg)
{
  if (!msg)
  {
    RCLCPP_ERROR(get_logger(), "onVision: received null msg");
    return;
  }

  if (camera_frame_.empty() || base_frame_.empty()) {
    RCLCPP_ERROR(get_logger(), "onVision: frame id empty (camera='%s', base='%s')",
                 camera_frame_.c_str(), base_frame_.c_str());
    return;
  }

  if (!tf_buffer_) {
    RCLCPP_ERROR(get_logger(), "onVision: tf_buffer_ is null (did you construct it and keep a member TransformListener alive?)");
    return;
  }

  geometry_msgs::msg::PointStamped in_eye;
  in_eye.header.frame_id = camera_frame_;
  in_eye.header.stamp    = this->now();

  in_eye.point.x =  0.001 * msg->ball_cam_z;
  in_eye.point.y = -0.001 * msg->ball_cam_x;
  in_eye.point.z = -0.001 * msg->ball_cam_y;

  geometry_msgs::msg::PointStamped out_base;
  bool tf_status_flag = transformPointToBase(in_eye, out_base);

  if (!tf_status_flag)
  {
    RCLCPP_ERROR(get_logger(), "onVision: transformPointToBase failed");
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "onVision: IN EYE target: x=%.3f, y=%.3f, z=%.3f",
                in_eye.point.x, in_eye.point.y, in_eye.point.z);
    RCLCPP_INFO(get_logger(), "onVision: Transformed target: x=%.3f, y=%.3f, z=%.3f",
                out_base.point.x, out_base.point.y, out_base.point.z);
  }

  // 현재 YAW 기준으로 base link Frame에서 World Frame으로 좌표 변환
  Eigen::Vector3d pB(out_base.point.x, out_base.point.y, out_base.point.z);
  Eigen::Vector3d t_pivot(0.0, 0.0, 0.0);

  const double yaw = q_current_[0];
  const double c = std::cos(-yaw);
  const double s = std::sin(-yaw);

  Eigen::Matrix3d R;
  R << c, -s, 0,
      s,  c, 0,
      0,  0, 1;

  Eigen::Vector3d pW0 = R * pB;

  auto target = std::make_shared<geometry_msgs::msg::Point>();
  target->x = pW0.x();
  target->y = pW0.y();
  target->z = pW0.z();
  Eigen::Vector3d ball_pos(target->x, target->y, target->z);

  pubBallMarker(ball_pos);
  target_cmd_callback(target);
}

bool GripperMainNode::transformPointToBase(const geometry_msgs::msg::PointStamped& in_eye,
                                           geometry_msgs::msg::PointStamped& out_base)
{
  if (!tf_buffer_) {
    RCLCPP_ERROR(get_logger(), "transformPointToBase: tf_buffer_ is null");
    return false;
  }

  const rclcpp::Time query_time = rclcpp::Time(0);

  geometry_msgs::msg::TransformStamped T_base_eye;
  try {
    auto clk = this->get_clock();
    if (!clk)
    {
      RCLCPP_WARN(get_logger(), "transformPointToBase: clock is null, skipping throttle");
    }

    if (!tf_buffer_->canTransform(base_frame_, in_eye.header.frame_id, query_time, tf2::durationFromSec(tf_timeout_sec_)))
    {
      RCLCPP_WARN(get_logger(), "Waiting TF %s -> %s (t=%.3f)", base_frame_.c_str(), in_eye.header.frame_id.c_str(), query_time.seconds());
      return false;
    }

    T_base_eye = tf_buffer_->lookupTransform(base_frame_, in_eye.header.frame_id, query_time);
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }

  try {
    tf2::doTransform(in_eye, out_base, T_base_eye);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "doTransform failed: %s", e.what());
    return false;
  }
  return true;
}

// ===================== teleop 바인딩용 target_cmd_callback =====================
void GripperMainNode::target_cmd_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  Eigen::Vector3d target(msg->x, msg->y, msg->z);
  const Eigen::Matrix3d R_cur = ik_->currentEEPose(q_current_).rotation();
  Eigen::Matrix3d R_des;
  ik_->buildTiltOnlyRotation(R_cur, R_des, Eigen::Vector3d(0,0,-1));
  pinocchio::SE3 M_des(R_des, target);
  bool ok = ik_->computeIKPrioritySE3(M_des, q_current_, /*tilt_only=*/false);

  if(!ok)
  {
    ok = ik_->computeIKPosition(target, q_current_);
    if(debug_mode_)
    {
      RCLCPP_WARN(get_logger(), "[MN] controlling by computeIKPosition");
    }
  }

  // Self-collision 체크
  bool in_collision = ik_->checkSelfCollision(q_current_);
  std_msgs::msg::Bool cmsg;
  cmsg.data = in_collision;
  pub_collision_->publish(cmsg);
  if (in_collision)
  {
    RCLCPP_WARN(get_logger(), "Self-collision detected.");
  }

  if (!ok)
  {
    RCLCPP_ERROR(get_logger(), "[IK] IK failed. target=(%.3f, %.3f, %.3f)", msg->x, msg->y, msg->z);
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),"[IK] IK success target=(%.3f, %.3f, %.3f)", msg->x, msg->y, msg->z);
  }
}

// ===================== called by main sequence =====================
void GripperMainNode::bindTargetToMotion(const geometry_msgs::msg::Point::SharedPtr msg, int step_num)
{
  if (!msg)
  {
    RCLCPP_ERROR(get_logger(), "[BT2M] received null msg");
    throw std::runtime_error("[BT2M] received null msg");
  }

  // 최초 시드 자세 리셋
  if(step_num == FIRST_STEP)
  {
    params_.ref_name = "waypoint_offset";
    ik_->setParams(params_);
    q_current_ = ik_->initialConfiguration();
  }

  // compute IK Position
  Eigen::Vector3d target(msg->x, msg->y, msg->z);
  bool ok = ik_->computeIKPosition(target, q_current_);

  // Self-collision 체크
  bool in_collision = ik_->checkSelfCollision(q_current_);

  std_msgs::msg::Bool ctrl_msg;
  ctrl_msg.data = false;

  if (in_collision)
  {
    pub_ctrl_flg_->publish(ctrl_msg);
    RCLCPP_ERROR(get_logger(), "[BT2M] Self-collision detected.");
    throw std::runtime_error("Self-collision detected. step_num=" + std::to_string(step_num));
  }
  if (!ok)
  {
    pub_ctrl_flg_->publish(ctrl_msg);
    RCLCPP_ERROR(get_logger(), "[BT2M] IK failed. target=(%.3f, %.3f, %.3f)", msg->x, msg->y, msg->z);
    throw std::runtime_error("IK failed. step_num=" + std::to_string(step_num));
  }
  else
  {
    RCLCPP_INFO(get_logger(),"[BT2M] IK success  target=(%.3f, %.3f, %.3f) in step %d", msg->x, msg->y, msg->z , step_num);
  }

  // MOTION EDITOR 바인딩
  JointPosMap jpm;
  jpm["rotate_torso"] = q_current_[0];
  jpm["rotate_0"]     = q_current_[1];
  jpm["rotate_1"]     = q_current_[1];
  jpm["rotate_2"]     = -1.0 * q_current_[2];
  jpm["rotate_3"]     = q_current_[2];
  jpm["rotate_4"]     = -1.0 * q_current_[3];
  jpm["rotate_5"]     = q_current_[3];

  // 그리퍼 조작 >> STEP_NUM 2, 5 : OPEN / STEP_NUM 3, 4 : CLOSE
  if(step_num % 3 == 2 )
  {
    jpm["gripper"] = GRIPPER_OPEN_RAD;
  }
  else
  {
    jpm["gripper"] = GRIPPER_CLOSE_RAD;
  }

  std::string step_name = std::to_string(step_num);
  motion_editor_->editByStepAndMap(step_name, jpm);
  motion_editor_->saveToFile(yaml_path);
  RCLCPP_INFO(get_logger(), "[MN] MotionEditor updated step '%s' with new joint positions.", step_name.c_str());
}

void GripperMainNode::getParamsFromRos()
{
  RCLCPP_INFO(get_logger(), "Loading IK parameters from ROS parameters...");
  this->get_parameter("MAX_ITERATIONS", params_.max_iter);
  this->get_parameter("POSITION_TOLERANCE", params_.pos_tol);
  this->get_parameter("SE3_TOLERANCE", params_.se3_tol);
  this->get_parameter("ALPHA", params_.alpha);
  this->get_parameter("LAMBDA", params_.lambda);
  this->get_parameter("STEP_MAX", params_.step_max);
  this->get_parameter("USE_REFERENCE", params_.use_ref_home);
  this->get_parameter("REF_NAME", params_.ref_name);
  this->get_parameter("IK_TILT_ONLY", tilt_only);
  this->get_parameter("YAW_WEIGHT", params_.yaw_weight);
  this->get_parameter("ROLL_WEIGHT", params_.roll_weight);
  this->get_parameter("PITCH_WEIGHT", params_.pitch_weight);
  this->get_parameter("BASE_FRAME", base_frame_);
  this->get_parameter("CAMERA_FRAME", camera_frame_);
  this->get_parameter("USE_LATEST_TF", use_latest_tf_);
  this->get_parameter("TF_TIMEOUT_SEC", tf_timeout_sec_);
  this->get_parameter("TF_THROTTLE_MS", tf_throttle_ms_);
  this->get_parameter("DEBUG_TF", debug_mode_);
  this->get_parameter("WAYPOINT_D", waypoint_d);
  this->get_parameter("GRAVITY_OFFSET_ROLL", gravity_offset_roll);
  this->get_parameter("MOTION_PATH", motion_path);
  this->get_parameter("DEBUG_VISUALIZATION", debug_visualization_);
  this->get_parameter("TARGETPOINT_D", targetpoint_d);

  ik_->setParams(params_);

  if(debug_mode_)
  {
    RCLCPP_INFO(get_logger(), "=== Parameters ===");
    RCLCPP_INFO(get_logger(), "MAX_ITERATIONS: %d", params_.max_iter);
    RCLCPP_INFO(get_logger(), "POSITION_TOLERANCE: %.6f", params_.pos_tol);
    RCLCPP_INFO(get_logger(), "SE3_TOLERANCE: %.6f", params_.se3_tol);
    RCLCPP_INFO(get_logger(), "ALPHA: %.6f", params_.alpha);
    RCLCPP_INFO(get_logger(), "LAMBDA: %.6f", params_.lambda);
    RCLCPP_INFO(get_logger(), "STEP_MAX: %.6f", params_.step_max);
    RCLCPP_INFO(get_logger(), "USE_REFERENCE: %s", params_.use_ref_home ? "true" : "false");
    RCLCPP_INFO(get_logger(), "REF_NAME: %s", params_.ref_name.c_str());
    RCLCPP_INFO(get_logger(), "IK_TILT_ONLY: %s", tilt_only ? "true" : "false");
    RCLCPP_INFO(get_logger(), "YAW_WEIGHT: %.6f", params_.yaw_weight);
    RCLCPP_INFO(get_logger(), "ROLL_WEIGHT: %.6f", params_.roll_weight);
    RCLCPP_INFO(get_logger(), "PITCH_WEIGHT: %.6f", params_.pitch_weight);
    RCLCPP_INFO(get_logger(), "BASE_FRAME: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "CAMERA_FRAME: %s", camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "USE_LATEST_TF: %s", use_latest_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "TF_TIMEOUT_SEC: %.3f", tf_timeout_sec_);
    RCLCPP_INFO(get_logger(), "TF_THROTTLE_MS: %d", tf_throttle_ms_);
    RCLCPP_INFO(get_logger(), "DEBUG_TF: %s", debug_mode_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "WAYPOINT_D: %.6f", waypoint_d);
    RCLCPP_INFO(get_logger(), "GRAVITY_OFFSET_ROLL: %.3f", gravity_offset_roll);
    RCLCPP_INFO(get_logger(), "MOTION_PATH: %s", motion_path.c_str());
    RCLCPP_INFO(get_logger(), "DEBUG_VISUALIZATION: %s", debug_visualization_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "TARGETPOINT_D: %.6f", targetpoint_d);
    RCLCPP_INFO(get_logger(), "==================");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Parameters loaded.");
  }
}
} // namespace SRCIRC2025_HUMANOID_LOCOMOTION

// ----- main -----
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 4) {
    std::cerr << "Usage: gripper_main_node <urdf_path> <srdf_path> <auto_control>\n";
    return 1;
  }

  auto node = std::make_shared<SRCIRC2025_HUMANOID_LOCOMOTION::GripperMainNode>(argv[1], argv[2], argv[3]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
