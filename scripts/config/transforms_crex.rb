# dynamic_transform "odometry.odometry_samples",
#   "body" => "odometry"

static_transform(Eigen::Vector3.new(0.208997, 0.0, 0.053925),
    Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI*0.5, -3.14159, 0.0),2,1,0),
    "imu" => "body")

dynamic_transform "xsens.orientation_samples", "imu" => "imu_enu"

static_transform(Eigen::Vector3.new(0.0, 0.0, 0.0),
    Eigen::Quaternion.from_euler(Eigen::Vector3.new(-Math::PI*0.75,0,0),2,1,0),
    "imu_enu" => "odometry")

static_transform(Eigen::Vector3.new(0.0093, 0.0, 0.308987),
    Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI,0,0),2,1,0),
    "laser" => "body")