/**
 * @file dynamic_forest_seq_sensing.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate a sequence of dynamic forest maps for fake sensing
 * To generate the current global map and local map in next time steps,
 * we need to publish current global point cloud and velocity
 *
 * The cylinder velocity is published as the difference between current position
 * and next position, which is (marker.points[1] - marker.points[0])
 *
 * @version 1.0
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

// for cylinders
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>

// for dynamic obstacles
#include <Eigen/Eigen>
#include <random>

#include "map_generator/moving_circle.h"
#include "map_generator/moving_cylinder_backandforth.h"
#include "map_generator/wall.h"

using namespace std;

vector<int>   pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device                     rd;
default_random_engine             eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
// geometry_msgs::PoseStamped obs1_pose, obs2_pose, obs3_pose;

// ros::Publisher _local_map_pub;
ros::Publisher _all_map_cloud_pub, _all_map_cylinder_pub, _all_map_cylinder_pub_vis;
ros::Publisher _click_map_pub_, _cylinder_state_pub;

vector<double> _state;

int         _obs_num, _box_num, _wall_num;
double      _x_size, _y_size, _z_size;
double      _w_l, _w_h, _h_l, _h_h, _v_h, _dr;
double      _x_l1,_x_h1,_y_l1,_y_h1;
double      _x_l2,_x_h2,_y_l2,_y_h2;
double      _x_l3,_x_h3,_y_l3,_y_h3;
double      _x_l4,_x_h4,_y_l4,_y_h4;
double      _x_l5,_x_h5,_y_l5,_y_h5;
double      _radius_h, _radius_l, _z_l, _z_h, _theta, _omega_h;
double      _z_limit, _sensing_range, _resolution, _sense_rate;
double      _obs1x, _obs1y, _obs1w, _obs1h, _obs2x, _obs2y, _obs2w, _obs2h, _obs3x, _obs3y, _obs3w, _obs3h, _obs4x, _obs4y, _obs4w, _obs4h, _obs5x, _obs5y, _obs5w, _obs5h;
double _wall_x1_begin, _wall_x1_end, _wall_y1_begin, _wall_y1_end;
double _wall_x2_begin, _wall_x2_end, _wall_y2_begin, _wall_y2_end;
double _box1_x, _box1_y;
double _box2_x, _box2_y;
// std::vector<double> wall_x_begin, wall_x_end, wall_y_begin, wall_y_end;
std::string _frame_id;

bool _map_ok       = false;
bool _has_odom     = false;
bool _test_mode    = false;

/**@brief map mode
 * 0: use both vx and vy
 * 1: use vx, vy = 0
 * 2: use vy, vx = 0
 * 3: vy, vx = 0
 */
int _mode1 = 0;
int _mode2 = 0;
int _mode3 = 0;
int _mode4 = 0;
int _mode5 = 0;

std::vector<double> _given_vel1 = {0.0, 0.0};
std::vector<double> _given_vel2 = {0.0, 0.0};
std::vector<double> _given_vel3 = {0.0, 0.0};
std::vector<double> _given_vel4 = {0.0, 0.0};
std::vector<double> _given_vel5 = {0.0, 0.0};

double _noiseLevel = 0.05;

/* map sequence settings */
bool   _future_map = false;
int    _num_future_map;
double _future_step_size;

sensor_msgs::PointCloud2 _globalMap_pcd;
sensor_msgs::PointCloud2 _globalCylinders_pcd;

pcl::PointCloud<pcl::PointXYZ> _clouds;

visualization_msgs::MarkerArray _cylinders_vis;
visualization_msgs::Marker      _cylinder_mk;
visualization_msgs::MarkerArray _circle_vis;
visualization_msgs::Marker      _circle_mk;
visualization_msgs::MarkerArray _obstacle_state_list;
visualization_msgs::Marker      _obstacle_state;
geometry_msgs::PoseStamped _walker_pose;

std::vector<dynamic_map_objects::MovingCylinder> _dyn_cylinders;
std::vector<dynamic_map_objects::MovingCircle>   _dyn_circles;

std::vector<static_env::Wall> _sta_walls;
std::vector<static_env::Box> _sta_boxs;


bool _dyn_obj_cld_pub = true;  // 是否发布动态圆柱行人的点云

bool _load_pcd_file = false; // 是否根据pcd文件载入点云
std::string _pcd_file_path;


// 函数：给一个入参添加噪声
double addNoise(double input, double noiseLevel) {
    // 创建一个随机数生成器
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(0.0, noiseLevel);

    // 生成噪声
    double noise = distribution(generator);

    // 将噪声与原始值相加
    double noisyInput = input + noise;

    return noisyInput;
}

/**
 * @brief generate random map
 *
 */
void RandomMapGenerate() {
  // generate pillar obstacles
  if (!_dyn_cylinders.empty()) _dyn_cylinders.clear();
  _dyn_cylinders.reserve(_obs_num);
  cout << "obs_num: " << _obs_num << endl;
  for (int i = 1; i <= _obs_num; i++) {
    double obs_x, obs_y, obs_w, obs_h, x_l, x_h, y_l, y_h;
    double mode;
	std::vector<double> given_vel;

    switch (i) {
        case 1:
            obs_x = _obs1x;
            obs_y = _obs1y;
            obs_w = _obs1w;
            obs_h = _obs1h;
            x_l = _x_l1;
            x_h = _x_h1;
            y_l = _y_l1;
            y_h = _y_h1;
            given_vel = _given_vel1;
            mode = _mode1;
            break;
        case 2:
            obs_x = _obs2x;
            obs_y = _obs2y;
            obs_w = _obs2w;
            obs_h = _obs2h;
            x_l = _x_l2;
            x_h = _x_h2;
            y_l = _y_l2;
            y_h = _y_h2;
            given_vel = _given_vel2;
            mode = _mode2;
            break;
        case 3:
            obs_x = _obs3x;
            obs_y = _obs3y;
            obs_w = _obs3w;
            obs_h = _obs3h;
            x_l = _x_l3;
            x_h = _x_h3;
            y_l = _y_l3;
            y_h = _y_h3;
            given_vel = _given_vel3;
            mode = _mode3;
            break;
        case 4:
            obs_x = _obs4x;
            obs_y = _obs4y;
            obs_w = _obs4w;
            obs_h = _obs4h;
            x_l = _x_l4;
            x_h = _x_h4;
            y_l = _y_l4;
            y_h = _y_h4;
            given_vel = _given_vel4;
            mode = _mode4;
            break;
        case 5:
            obs_x = _obs5x;
            obs_y = _obs5y;
            obs_w = _obs5w;
            obs_h = _obs5h;
            x_l = _x_l5;
            x_h = _x_h5;
            y_l = _y_l5;
            y_h = _y_h5;
            given_vel = _given_vel5;
            mode = _mode5;
            break;
        default:
            obs_x = 0.0;
            obs_y = 0.0;
            obs_w = 0.0;
            obs_h = 0.0;
            x_l = 0.0;
            x_h = 0.0;
            y_l = 0.0;
            y_h = 0.0;
            given_vel = {0.0,0.0};
            mode = 3; // 静止
            break;
    }

    dynamic_map_objects::MovingCylinder cylinder(x_l, x_h, y_l, y_h, _w_l, _w_h, _h_l, _h_h,
                                                _v_h, eng, _resolution, obs_x, obs_y, obs_h, obs_w);
	given_vel[0] /= _sense_rate; // 统一速度单位
	given_vel[1] /= _sense_rate;
    cylinder.setVel(given_vel);
    cylinder.setVelMode(mode);
    _dyn_cylinders.push_back(cylinder);
  }

  // generate wall obstacles
  if (!_sta_walls.empty()) _sta_walls.clear();
  _sta_walls.reserve(_wall_num);
  cout << "wall_num: " << _wall_num << endl;
  for (int i = 1; i <= _wall_num; i++) {
    double x_begin, x_end, y_begin, y_end;
    switch (i) {
      case 1:
        x_begin = _wall_x1_begin;
        x_end = _wall_x1_end;
        y_begin = _wall_y1_begin;
        y_end = _wall_y1_end;
        break;
      case 2:
        x_begin = _wall_x2_begin;
        x_end = _wall_x2_end;
        y_begin = _wall_y2_begin;
        y_end = _wall_y2_end;
        break;
      default:
        x_begin = 0.0;
        x_end = 0.0;
        y_begin = 0.0;
        y_end = 0.0;
        break;
    }
    static_env::Wall wall(x_begin, y_begin, x_end, y_end);
    _sta_walls.emplace_back(wall);
  }

  // generate box obstacles
  if (!_sta_boxs.empty()) _sta_boxs.clear();
  _sta_boxs.reserve(_box_num);
  cout << "box_num: " << _box_num << endl;
  for (int i = 1; i <= _box_num; i++) {
    double box_x, box_y;
    switch (i) {
      case 1:
        box_x = _box1_x;
        box_y = _box1_y;
        break;
      case 2:
        box_x = _box2_x;
        box_y = _box2_y;
        break;
      default:
        box_x = 0.0;
        box_y = 0.0;
        break;
    }
    static_env::Box box(box_x, box_y);
    _sta_boxs.emplace_back(box);
  }

  ROS_WARN("Finished generate obstacle map ");

  _map_ok = true;
}

/**
 * @brief
 *
 */
void pubSensedPoints() {
  // concatenate all points
  if(!_clouds.points.empty()) _clouds.points.clear();
  _clouds.points.reserve(_obs_num);

  if(!_cylinders_vis.markers.empty()) _cylinders_vis.markers.clear();
  _cylinders_vis.markers.reserve(_obs_num);

  if(!_obstacle_state_list.markers.empty()) _obstacle_state_list.markers.clear();
  _obstacle_state_list.markers.reserve(_obs_num);

  _cylinder_mk.header.stamp = ros::Time::now();
  _cylinder_mk.id           = 0;

  _obstacle_state.header.stamp = ros::Time::now();
  if(!_obstacle_state.points.empty()) _obstacle_state.points.clear();
  _obstacle_state.id = 0;

  pcl::PointCloud<pcl::PointXYZ> cloud_all;

//   std::cout << ros::package::getPath("dyn_map_generator") << std::endl;
  if (_load_pcd_file) pcl::io::loadPCDFile(ros::package::getPath("dyn_map_generator")+_pcd_file_path,cloud_all);//通过launch文件修改路径, 表示从文件中读取

  if (!_dyn_cylinders.empty()) {
	for (auto& dyn_cld : _dyn_cylinders) {
		if (!_test_mode) {
			dyn_cld.update();// 更新地图内容
		}

		// publish cylinder markers
		pcl::PointXYZ pt_center;
		pt_center.x = dyn_cld.x;
		pt_center.y = dyn_cld.y;
		pt_center.z = 0.5 * dyn_cld.h;
		_clouds.points.push_back(pt_center);// clouds存放圆柱的中心位置

		geometry_msgs::Pose pose;
		pose.position.x    = dyn_cld.x;
		pose.position.y    = dyn_cld.y;
		pose.position.z    = 0.5 * dyn_cld.h;
		pose.orientation.w = 1.0;

		_cylinder_mk.pose    = pose;
		_cylinder_mk.scale.x = _cylinder_mk.scale.y = dyn_cld.w;  // less then 1
		_cylinder_mk.scale.z                       = dyn_cld.h;
		_cylinder_mk.color.a = 0.3;
		_cylinders_vis.markers.push_back(_cylinder_mk);// 每个圆柱都做半透明边界可视化
		_cylinder_mk.id += 1;

		if (dyn_cld.getVelMode() == 3) {
			cloud_all += dyn_cld._cloud;  // 3 表示静止的，只有静止的才加入静态地图中
		} else {
		if (_dyn_obj_cld_pub) cloud_all += dyn_cld._cloud;  // 若设置动态点云也发布则动态cylinder也加入静态地图中
			// 只有动态的才发布state消息
			_obstacle_state.pose               = pose;
			_obstacle_state.pose.position.x    = addNoise(dyn_cld.x, _noiseLevel);
			_obstacle_state.pose.position.y    = addNoise(dyn_cld.y, _noiseLevel);
			_obstacle_state.pose.position.z    = 0.5 * dyn_cld.h;
			_obstacle_state.pose.orientation.w = 1.0;
			_obstacle_state.points.clear();
			geometry_msgs::Point pts;
			pts.x = pose.position.x;
			pts.y = pose.position.y;
			pts.z = pose.position.z;
			_obstacle_state.points.push_back(pts);
			// pts.x += dyn_cld.vx / _sense_rate;
			// pts.y += dyn_cld.vy / _sense_rate;
			// obstacle_state.points.push_back(pts);
			_obstacle_state.scale.x = dyn_cld.w;
			_obstacle_state.scale.y = dyn_cld.w;
			_obstacle_state.scale.z = dyn_cld.h;
			_obstacle_state.type    = visualization_msgs::Marker::CYLINDER;
			_obstacle_state_list.markers.push_back(_obstacle_state);
			_obstacle_state.id += 1;
		}
	}
  }

  if (!_sta_walls.empty()) {
	for (auto& sta_wall : _sta_walls) {
		cloud_all += sta_wall._cloud;
	}    
  }

  if (!_sta_boxs.empty()) {
	for (auto& sta_box : _sta_boxs) {
		cloud_all += sta_box._cloud;
	}    
  }

  cloud_all.width    = cloud_all.points.size();
  cloud_all.height   = 1;
  cloud_all.is_dense = true;

  // ROS_WARN_STREAM("Publishing " << cloud_all.points.size() << " points");
  // ROS_WARN_STREAM("Publishing " << cylinders.points.size() << " cylinders");
  // ROS_WARN_STREAM("Publishing " << cylinders_vis.markers.size() << " cylinders markers");

  // publish cloud
  pcl::toROSMsg(cloud_all, _globalMap_pcd);
  _globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(_globalMap_pcd);

  // publish cylinder markers for visualization
  pcl::toROSMsg(_clouds, _globalCylinders_pcd);
  _globalCylinders_pcd.header.frame_id = _frame_id;
  _all_map_cylinder_pub.publish(_globalCylinders_pcd);// 捏马，只发布了中心位置，而且全白色的谁看的清啊啊啊啊
  _all_map_cylinder_pub_vis.publish(_cylinders_vis);// 这个倒是发布了能看的清的圆柱边界

  // state
  _cylinder_state_pub.publish(_obstacle_state_list);// 其实这个里面好像也有发布障碍物位置信息

  return;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_map_sequence_sensing");
  ros::NodeHandle n("~");

  n.param("map/future", _future_map, true);
  n.param("map/future_num", _num_future_map, 6);
  n.param("map/time_step", _future_step_size, 0.2);
  n.param("map/x_size", _x_size, 10.0);
  n.param("map/y_size", _y_size, 10.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/test", _test_mode, false); 


  n.param("map/obs_num", _obs_num, 3);
  n.param("map/wall_num", _wall_num, 0);
  n.param("map/box_num", _box_num, 0);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/frame_id", _frame_id, string("map"));

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);
  n.param("ObstacleShape/upper_vel", _v_h, 0.1);

  n.param("ObstacleShape/radius_l", _radius_l, 7.0);
  n.param("ObstacleShape/radius_h", _radius_h, 7.0);
  n.param("ObstacleShape/z_l", _z_l, 7.0);
  n.param("ObstacleShape/z_h", _z_h, 7.0);
  n.param("ObstacleShape/dr", _dr, 0.2);
  n.param("ObstacleShape/theta", _theta, 7.0);
  n.param("ObstacleShape/omega", _omega_h, 2.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);
  n.param("sensing/noiseLevel", _noiseLevel, 0.05);
  n.param("dyn_obj_cld_pub", _dyn_obj_cld_pub, false);

  n.param("obs1w", _obs1w, 0.0);
  n.param("obs1x", _obs1x, 0.0);
  n.param("obs1y", _obs1y, 0.0);
  n.param("obs1h", _obs1h, 0.0);
  n.param("mode1", _mode1, 3);
  n.param("given_vel1x", _given_vel1[0], 0.0);
  n.param("given_vel1y", _given_vel1[1], 0.0);
  n.param("obs1x_l", _x_l1, 0.0);
  n.param("obs1x_h", _x_h1, 0.0);
  n.param("obs1y_l", _y_l1, 0.0);
  n.param("obs1y_h", _y_h1, 0.0);

  n.param("obs2w", _obs2w, 0.0);
  n.param("obs2x", _obs2x, 0.0);
  n.param("obs2y", _obs2y, 0.0);
  n.param("obs2h", _obs2h, 0.0);
  n.param("mode2", _mode2, 3);
  n.param("given_vel2x", _given_vel2[0], 0.0);
  n.param("given_vel2y", _given_vel2[1], 0.0);
  n.param("obs2x_l", _x_l2, 0.0);
  n.param("obs2x_h", _x_h2, 0.0);
  n.param("obs2y_l", _y_l2, 0.0);
  n.param("obs2y_h", _y_h2, 0.0);

  n.param("obs3w", _obs3w, 0.0);
  n.param("obs3x", _obs3x, 0.0);
  n.param("obs3y", _obs3y, 0.0);
  n.param("obs3h", _obs3h, 0.0);
  n.param("mode3", _mode3, 3);
  n.param("given_vel3x", _given_vel3[0], 0.0);
  n.param("given_vel3y", _given_vel3[1], 0.0);
  n.param("obs3x_l", _x_l3, 0.0);
  n.param("obs3x_h", _x_h3, 0.0);
  n.param("obs3y_l", _y_l3, 0.0);
  n.param("obs3y_h", _y_h3, 0.0);

  n.param("obs4w", _obs4w, 0.0);
  n.param("obs4x", _obs4x, 0.0);
  n.param("obs4y", _obs4y, 0.0);
  n.param("obs4h", _obs4h, 0.0);
  n.param("mode4", _mode4, 3);
  n.param("given_vel4x", _given_vel4[0], 0.0);
  n.param("given_vel4y", _given_vel4[1], 0.0);
  n.param("obs4x_l", _x_l4, 0.0);
  n.param("obs4x_h", _x_h4, 0.0);
  n.param("obs4y_l", _y_l4, 0.0);
  n.param("obs4y_h", _y_h4, 0.0);

  n.param("obs5w", _obs5w, 0.0);
  n.param("obs5x", _obs5x, 0.0);
  n.param("obs5y", _obs5y, 0.0);
  n.param("obs5h", _obs5h, 0.0);
  n.param("mode5", _mode5, 3);
  n.param("given_vel5x", _given_vel5[0], 0.0);
  n.param("given_vel5y", _given_vel5[1], 0.0);
  n.param("obs5x_l", _x_l5, 0.0);
  n.param("obs5x_h", _x_h5, 0.0);
  n.param("obs5y_l", _y_l5, 0.0);
  n.param("obs5y_h", _y_h5, 0.0);

  n.param("wall_x1_begin", _wall_x1_begin, 0.0);
  n.param("wall_y1_begin", _wall_y1_begin, 0.0);
  n.param("wall_x1_end", _wall_x1_end, 0.0);
  n.param("wall_y1_end", _wall_y1_end, 0.0);
  n.param("wall_x2_begin", _wall_x2_begin, 0.0);
  n.param("wall_y2_begin", _wall_y2_begin, 0.0);
  n.param("wall_x2_end", _wall_x2_end, 0.0);
  n.param("wall_y2_end", _wall_y2_end, 0.0);
  
  n.param("box1_x", _box1_x, 0.0);
  n.param("box1_y", _box1_y, 0.0);
  n.param("box2_x", _box2_x, 0.0);
  n.param("box2_y", _box2_y, 0.0);

  n.param("load_pcd_file", _load_pcd_file, false);
  n.param<std::string>("file_path", _pcd_file_path, "/home/nros/bigHouse2.pcd");

  _all_map_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_cylinder_pub = n.advertise<sensor_msgs::PointCloud2>("global_cylinders", 1);
  _all_map_cylinder_pub_vis =
      n.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);
  _cylinder_state_pub = n.advertise<visualization_msgs::MarkerArray>("global_cylinder_state", 1);

  // clearance for multi robots.
  _x_size -= 2.0;
  _y_size -= 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  _cylinder_mk.header.frame_id = _frame_id;
  _cylinder_mk.type            = visualization_msgs::Marker::CYLINDER;
  _cylinder_mk.action          = visualization_msgs::Marker::ADD;
  _cylinder_mk.id              = 0;
  _cylinder_mk.color.r         = 0.5;
  _cylinder_mk.color.g         = 0.5;
  _cylinder_mk.color.b         = 0.5;
  _cylinder_mk.color.a         = 0.6;

  _obstacle_state.header = _cylinder_mk.header;
  _obstacle_state.type   = visualization_msgs::Marker::ARROW;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);
  ROS_WARN_STREAM("_sense_rate: " << _sense_rate);

  while (ros::ok()) {
    // update map
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}