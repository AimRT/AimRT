// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_ros2_interface/util/ros2_type_support.h"
// action_msgs includes
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

// actionlib_msgs includes
#include "actionlib_msgs/msg/goal_id.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"
#include "actionlib_msgs/msg/goal_status_array.hpp"

// actuator_msgs includes
#include "actuator_msgs/msg/actuators.hpp"
#include "actuator_msgs/msg/actuators_angular_position.hpp"
#include "actuator_msgs/msg/actuators_angular_velocity.hpp"
#include "actuator_msgs/msg/actuators_linear_position.hpp"
#include "actuator_msgs/msg/actuators_linear_velocity.hpp"
#include "actuator_msgs/msg/actuators_normalized.hpp"
#include "actuator_msgs/msg/actuators_position.hpp"
#include "actuator_msgs/msg/actuators_velocity.hpp"

// diagnostic_msgs includes
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

// geometry_msgs includes
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/velocity_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// grid_map_msgs includes
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_msgs/msg/grid_map_info.hpp"

// lifecycle_msgs includes
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

// map_msgs includes
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "map_msgs/msg/point_cloud2_update.hpp"
#include "map_msgs/msg/projected_map.hpp"
#include "map_msgs/msg/projected_map_info.hpp"

// nav2_msgs includes
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"

// nav_msgs includes
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// octomap_msgs includes
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/msg/octomap_with_pose.hpp"

// pcl_msgs includes
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "pcl_msgs/msg/vertices.hpp"

// pendulum_msgs includes
#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"
#include "pendulum_msgs/msg/rttest_results.hpp"

// plotjuggler_msgs includes
#include "plotjuggler_msgs/msg/data_point.hpp"
#include "plotjuggler_msgs/msg/data_points.hpp"
#include "plotjuggler_msgs/msg/dictionary.hpp"
#include "plotjuggler_msgs/msg/statistics_names.hpp"
#include "plotjuggler_msgs/msg/statistics_values.hpp"

// rosgraph_msgs includes
#include "rosgraph_msgs/msg/clock.hpp"

// sensor_msgs includes
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"
#include "sensor_msgs/msg/laser_echo.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

// shape_msgs includes
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

// statistics_msgs includes
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

// std_msgs includes
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

// stereo_msgs includes
#include "stereo_msgs/msg/disparity_image.hpp"

// test_msgs includes
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/bounded_plain_sequences.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/builtins.hpp"
#include "test_msgs/msg/constants.hpp"
#include "test_msgs/msg/defaults.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"
#include "test_msgs/msg/w_strings.hpp"

// tf2_msgs includes
#include "tf2_msgs/msg/tf2_error.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// trajectory_msgs includes
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

// unique_identifier_msgs includes
#include "unique_identifier_msgs/msg/uuid.hpp"

// vision_msgs includes
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d_array.hpp"
#include "vision_msgs/msg/classification.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/label_info.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "vision_msgs/msg/pose2_d.hpp"
#include "vision_msgs/msg/vision_class.hpp"
#include "vision_msgs/msg/vision_info.hpp"

// visualization_msgs includes
#include "visualization_msgs/msg/image_marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/interactive_marker_init.hpp"
#include "visualization_msgs/msg/interactive_marker_pose.hpp"
#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/menu_entry.hpp"
#include "visualization_msgs/msg/mesh_file.hpp"
#include "visualization_msgs/msg/uv_coordinate.hpp"

static const aimrt_type_support_base_t* type_support_array[]{
    // std_msgs
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Bool>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Byte>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::ByteMultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Char>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::ColorRGBA>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Empty>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Float32>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Float32MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Float64>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Float64MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Header>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int16>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int16MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int32>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int32MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int64>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int64MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int8>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::Int8MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::MultiArrayDimension>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::MultiArrayLayout>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::String>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt16>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt16MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt32>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt32MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt64>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt64MultiArray>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt8>(),
    aimrt::GetRos2MessageTypeSupport<std_msgs::msg::UInt8MultiArray>(),

    // action_msgs
    aimrt::GetRos2MessageTypeSupport<action_msgs::msg::GoalInfo>(),
    aimrt::GetRos2MessageTypeSupport<action_msgs::msg::GoalStatus>(),
    aimrt::GetRos2MessageTypeSupport<action_msgs::msg::GoalStatusArray>(),

    // actionlib_msgs
    aimrt::GetRos2MessageTypeSupport<actionlib_msgs::msg::GoalID>(),
    aimrt::GetRos2MessageTypeSupport<actionlib_msgs::msg::GoalStatus>(),
    aimrt::GetRos2MessageTypeSupport<actionlib_msgs::msg::GoalStatusArray>(),

    // actuator_msgs
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::Actuators>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsAngularPosition>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsAngularVelocity>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsLinearPosition>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsLinearVelocity>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsNormalized>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsPosition>(),
    aimrt::GetRos2MessageTypeSupport<actuator_msgs::msg::ActuatorsVelocity>(),

    // diagnostic_msgs
    aimrt::GetRos2MessageTypeSupport<diagnostic_msgs::msg::DiagnosticArray>(),
    aimrt::GetRos2MessageTypeSupport<diagnostic_msgs::msg::DiagnosticStatus>(),
    aimrt::GetRos2MessageTypeSupport<diagnostic_msgs::msg::KeyValue>(),

    // geometry_msgs
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Accel>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::AccelStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::AccelWithCovariance>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::AccelWithCovarianceStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Inertia>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::InertiaStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Point>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Point32>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PointStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Polygon>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PolygonStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Pose>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Pose2D>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PoseArray>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PoseStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PoseWithCovariance>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::PoseWithCovarianceStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Quaternion>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::QuaternionStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Transform>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::TransformStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Twist>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::TwistStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::TwistWithCovariance>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::TwistWithCovarianceStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Vector3>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Vector3Stamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::VelocityStamped>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::Wrench>(),
    aimrt::GetRos2MessageTypeSupport<geometry_msgs::msg::WrenchStamped>(),

    // grid_map_msgs
    aimrt::GetRos2MessageTypeSupport<grid_map_msgs::msg::GridMap>(),
    aimrt::GetRos2MessageTypeSupport<grid_map_msgs::msg::GridMapInfo>(),

    // lifecycle_msgs
    aimrt::GetRos2MessageTypeSupport<lifecycle_msgs::msg::State>(),
    aimrt::GetRos2MessageTypeSupport<lifecycle_msgs::msg::Transition>(),
    aimrt::GetRos2MessageTypeSupport<lifecycle_msgs::msg::TransitionDescription>(),
    aimrt::GetRos2MessageTypeSupport<lifecycle_msgs::msg::TransitionEvent>(),

    // map_msgs
    aimrt::GetRos2MessageTypeSupport<map_msgs::msg::OccupancyGridUpdate>(),
    aimrt::GetRos2MessageTypeSupport<map_msgs::msg::PointCloud2Update>(),
    aimrt::GetRos2MessageTypeSupport<map_msgs::msg::ProjectedMap>(),
    aimrt::GetRos2MessageTypeSupport<map_msgs::msg::ProjectedMapInfo>(),

    // nav2_msgs
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::BehaviorTreeLog>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::BehaviorTreeStatusChange>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::Costmap>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::CostmapFilterInfo>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::CostmapMetaData>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::Particle>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::ParticleCloud>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::SpeedLimit>(),
    aimrt::GetRos2MessageTypeSupport<nav2_msgs::msg::VoxelGrid>(),

    // nav_msgs
    aimrt::GetRos2MessageTypeSupport<nav_msgs::msg::GridCells>(),
    aimrt::GetRos2MessageTypeSupport<nav_msgs::msg::MapMetaData>(),
    aimrt::GetRos2MessageTypeSupport<nav_msgs::msg::OccupancyGrid>(),
    aimrt::GetRos2MessageTypeSupport<nav_msgs::msg::Odometry>(),
    aimrt::GetRos2MessageTypeSupport<nav_msgs::msg::Path>(),

    // octomap_msgs
    aimrt::GetRos2MessageTypeSupport<octomap_msgs::msg::Octomap>(),
    aimrt::GetRos2MessageTypeSupport<octomap_msgs::msg::OctomapWithPose>(),

    // pcl_msgs
    aimrt::GetRos2MessageTypeSupport<pcl_msgs::msg::ModelCoefficients>(),
    aimrt::GetRos2MessageTypeSupport<pcl_msgs::msg::PointIndices>(),
    aimrt::GetRos2MessageTypeSupport<pcl_msgs::msg::PolygonMesh>(),
    aimrt::GetRos2MessageTypeSupport<pcl_msgs::msg::Vertices>(),

    // pendulum_msgs
    aimrt::GetRos2MessageTypeSupport<pendulum_msgs::msg::JointCommand>(),
    aimrt::GetRos2MessageTypeSupport<pendulum_msgs::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<pendulum_msgs::msg::RttestResults>(),

    // plotjuggler_msgs
    aimrt::GetRos2MessageTypeSupport<plotjuggler_msgs::msg::DataPoint>(),
    aimrt::GetRos2MessageTypeSupport<plotjuggler_msgs::msg::DataPoints>(),
    aimrt::GetRos2MessageTypeSupport<plotjuggler_msgs::msg::Dictionary>(),
    aimrt::GetRos2MessageTypeSupport<plotjuggler_msgs::msg::StatisticsNames>(),
    aimrt::GetRos2MessageTypeSupport<plotjuggler_msgs::msg::StatisticsValues>(),

    // rosgraph_msgs
    aimrt::GetRos2MessageTypeSupport<rosgraph_msgs::msg::Clock>(),

    // sensor_msgs
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::BatteryState>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::CameraInfo>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::ChannelFloat32>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::CompressedImage>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::FluidPressure>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Illuminance>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Image>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Imu>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Joy>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::JoyFeedback>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::JoyFeedbackArray>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::LaserEcho>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::LaserScan>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::MagneticField>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::MultiDOFJointState>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::MultiEchoLaserScan>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::NavSatFix>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::NavSatStatus>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::PointCloud>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::PointCloud2>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::PointField>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Range>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::RegionOfInterest>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::RelativeHumidity>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Temperature>(),
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::TimeReference>(),

    // shape_msgs
    aimrt::GetRos2MessageTypeSupport<shape_msgs::msg::Mesh>(),
    aimrt::GetRos2MessageTypeSupport<shape_msgs::msg::MeshTriangle>(),
    aimrt::GetRos2MessageTypeSupport<shape_msgs::msg::Plane>(),
    aimrt::GetRos2MessageTypeSupport<shape_msgs::msg::SolidPrimitive>(),

    // statistics_msgs
    aimrt::GetRos2MessageTypeSupport<statistics_msgs::msg::MetricsMessage>(),
    aimrt::GetRos2MessageTypeSupport<statistics_msgs::msg::StatisticDataPoint>(),
    aimrt::GetRos2MessageTypeSupport<statistics_msgs::msg::StatisticDataType>(),

    // stereo_msgs
    aimrt::GetRos2MessageTypeSupport<stereo_msgs::msg::DisparityImage>(),

    // test_msgs
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Arrays>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::BasicTypes>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::BoundedPlainSequences>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::BoundedSequences>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Builtins>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Constants>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Defaults>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Empty>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::MultiNested>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Nested>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::Strings>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::UnboundedSequences>(),
    aimrt::GetRos2MessageTypeSupport<test_msgs::msg::WStrings>(),

    // tf2_msgs
    aimrt::GetRos2MessageTypeSupport<tf2_msgs::msg::TF2Error>(),
    aimrt::GetRos2MessageTypeSupport<tf2_msgs::msg::TFMessage>(),

    // trajectory_msgs
    aimrt::GetRos2MessageTypeSupport<trajectory_msgs::msg::JointTrajectory>(),
    aimrt::GetRos2MessageTypeSupport<trajectory_msgs::msg::JointTrajectoryPoint>(),
    aimrt::GetRos2MessageTypeSupport<trajectory_msgs::msg::MultiDOFJointTrajectory>(),
    aimrt::GetRos2MessageTypeSupport<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(),

    // unique_identifier_msgs
    aimrt::GetRos2MessageTypeSupport<unique_identifier_msgs::msg::UUID>(),

    // vision_msgs
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::BoundingBox2D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::BoundingBox2DArray>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::BoundingBox3D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::BoundingBox3DArray>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Classification>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Detection2D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Detection2DArray>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Detection3D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Detection3DArray>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::LabelInfo>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::ObjectHypothesis>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::ObjectHypothesisWithPose>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Point2D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::Pose2D>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::VisionClass>(),
    aimrt::GetRos2MessageTypeSupport<vision_msgs::msg::VisionInfo>(),

    // visualization_msgs
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::ImageMarker>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarker>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarkerControl>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarkerFeedback>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarkerInit>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarkerPose>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::InteractiveMarkerUpdate>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::Marker>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::MarkerArray>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::MenuEntry>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::MeshFile>(),
    aimrt::GetRos2MessageTypeSupport<visualization_msgs::msg::UVCoordinate>(),
};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}