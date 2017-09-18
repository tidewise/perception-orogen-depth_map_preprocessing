dynamic_transform "seekur_drv.robot_pose",
  "body_tr" => "odometry"

dynamic_transform "xsens_imu.orientation_samples",
   "body" => "body_tr"

static_transform(Eigen::Vector3.new(0.0, 0.0, 1.2),
    Eigen::Quaternion.from_euler(Eigen::Vector3.new(-1.5707,0,0),2,1,0),
    "laser" => "body")