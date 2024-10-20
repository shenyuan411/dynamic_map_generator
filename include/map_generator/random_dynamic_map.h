/**
 * @file random_dynamic_map.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __RANDOM_DYNAMIC_MAP_H__
#define __RANDOM_DYNAMIC_MAP_H__

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <map_generator/obstacles.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <random>
#include <string>
#include <vector>

class RandomDynamicMap {
 private:
  ros::NodeHandle nh_;
  ros::Publisher  all_map_cloud_pub_;
  ros::Publisher  obstacle_vis_pub_;
  ros::Publisher  obstacle_state_pub_;
  ros::Publisher  local_future_map_pub_;

  /* map parameters */
  float       x_size_, y_size_, z_size_;
  float       x_l_, x_h_, y_l_, y_h_, z_l_, z_h_;
  float       v_h_;
  float       resolution_;
  float       sense_rate_;
  std::string frame_id_;

  /* random seed */
  int                        seed_;
  std::default_random_engine eng_;

  /* obstacles */
  int                                    num_obstacles_;
  std::vector<std::unique_ptr<Obstacle>> obstacles_;

  AABBConfig       aabb_config_;
  CylinderConfig   cylinder_config_;
  CircleGateConfig circlegate_config_;

  /* future map */
  int   num_future_map_;
  float future_step_size_;

  /* mics */
  bool is_test_mode_;
  bool is_rendered_{false};
  int  mode_;

 public:
  RandomDynamicMap(ros::NodeHandle &nh) : nh_(nh) {}
  ~RandomDynamicMap() {}
  void init();
  void renderMap();
  void publishMap();
  void publishObstacleState();

  Eigen::Vector3f sampleRandomPosition();
  Eigen::Vector3f sampleRandomVelocity2D();
  Eigen::Vector3f sampleRandomVelocity();

  /* helper functions */
  inline float getSenseRate() const { return sense_rate_; }
  inline float uniformSample(float max, float min) {
    return std::uniform_real_distribution<float>(min, max)(eng_);
  }
};

#endif  // __RANDOM_DYNAMIC_MAP_H__
