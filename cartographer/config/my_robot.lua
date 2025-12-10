-- turtlebot3_lds_2d.lua
-- RB-35GM + RPi4 + ROS2 Humble + odom.py(휠+IMU 융합 오돔) + YDLIDAR(/scan) 용
--
-- TF 트리 목표:
--   map ──(cartographer_node)──> odom ──(odom.py)──> base_link ──> laser_frame
--                                                └──────────────> imu_link (선택)
--
-- 사용 프레임
--   map_frame      : "map"
--   odom_frame     : "odom"       (/odom, odom.py 에서 퍼블리시)
--   tracking_frame : "base_link"  (Cartographer가 추적하는 프레임)
--   published_frame: "base_link"  (pose 를 퍼블리시할 child_frame)
--   laser_frame    : /scan 의 frame_id (ydlidar_ros2_driver 에서 "laser_frame")

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  ------------------------------------------------------------------
  -- Cartographer 기본 설정
  ------------------------------------------------------------------
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  ------------------------------------------------------------------
  -- Frame 설정
  ------------------------------------------------------------------
  map_frame = "map",

  -- odom.py 에서 이미 wheel + IMU 융합해서 base_link 의 자세를 계산하므로
  -- Cartographer 의 tracking_frame 은 base_link 로 둔다.
  tracking_frame = "base_link",

  -- Cartographer 가 publish 하는 로봇 포즈의 child_frame
  published_frame = "base_link",

  -- odom.py 가 사용하는 odom frame 과 동일하게 맞춘다.
  odom_frame = "odom",

  -- odom frame 은 odom.py 가 이미 제공하므로 Cartographer 가 새로 만들지 않는다.
  provide_odom_frame = false,

  -- 2D 평면 상의 프레임만 사용할 것이므로 2D 로 프로젝션
  publish_frame_projected_to_2d = true,

  ------------------------------------------------------------------
  -- 센서 사용 여부
  ------------------------------------------------------------------
  use_odometry = true,     -- /odom 사용 (odom.py)
  use_nav_sat = false,
  use_landmarks = false,

  -- LiDAR (LaserScan) 1개 사용 (YDLIDAR)
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  ------------------------------------------------------------------
  -- 타이밍 관련
  ------------------------------------------------------------------
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  ------------------------------------------------------------------
  -- 샘플링 비율 (처음에는 모두 1.0 으로 전체 사용)
  ------------------------------------------------------------------
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--------------------------------------------------------------------
-- 2D Trajectory Builder 사용
--------------------------------------------------------------------
MAP_BUILDER.use_trajectory_builder_2d = true

--------------------------------------------------------------------
-- Laser 설정 (YDLIDAR 범위에 맞춰 설정)
--  ydlidar_ros2_driver 의 range_min / range_max 와 맞추는 것이 좋음
--------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- LaserScan 을 몇 개 누적해서 처리할지 (1 이면 실시간성 ↑, 노이즈에는 조금 약함)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

--------------------------------------------------------------------
-- IMU 사용 여부
-- 현재는 odom.py 내부에서 IMU yaw 를 이미 사용하므로
-- Cartographer 에서는 비활성화해서 구조를 단순하게 유지
--------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.use_imu_data = false

--------------------------------------------------------------------
-- Local SLAM: 온라인 코릴레이티브 스캔 매칭 (초기 정합에 도움, CPU 조금 더 사용)
--------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 너무 작은 자세 변화는 무시 (노이즈 필터링용)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

--------------------------------------------------------------------
-- Ceres 스캔 매처 기본 튜닝
--  필요하면 나중에 translation_weight / rotation_weight 등을 조정
--------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

--------------------------------------------------------------------
-- Pose Graph / Constraint Builder (TB3 기본값과 유사한 세팅)
--------------------------------------------------------------------
POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

