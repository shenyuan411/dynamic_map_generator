
/**
 * @file moving_cylinder.h
 * @author Rocky Quan
 * @brief
 * @version 1.0
 * @date 2024-10-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __WALL_
#define __WALL_

#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>

namespace static_env {

class Wall {
private:
  double _start_x, _start_y, _end_x, _end_y;
  double _w, _h;
  double _resolution;

public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;
  Wall(double start_x, double start_y, double end_x, double end_y,
       double width = 0.2, double height = 3.0, double resolution = 0.1);
  ~Wall() = default;
};

Wall::Wall(double start_x, double start_y, double end_x, double end_y,
           double width, double height, double resolution)
    : _start_x(start_x), _start_y(start_y), _end_x(end_x), _end_y(end_y),
      _w(width), _h(height), _resolution(resolution) {
  double length = std::sqrt(std::pow((_end_x - _start_x), 2) +
                            std::pow((_end_y - _start_y), 2));
  // define height and width
  int heightNum = ceil(_h / _resolution);
  int widNum = ceil(_w / _resolution);
  int lengthNum = ceil(length / _resolution);

  // random point in grid scale
  double mid_x = floor((_end_x + _start_x) / 2 / _resolution) * _resolution +
                 _resolution / 2.0;
  double mid_y = floor((_end_y + _start_y) / 2 / _resolution) * _resolution +
                 _resolution / 2.0;

  // generate point cloud
  _cloud.points.resize(0);
  _cloud.width = 0;
  _cloud.height = 0;

  pcl::PointXYZ pt;
  // _cloud.points.push_back(pt);

  // NOTE: x direction is length, y direction is width
  for (int r = -lengthNum / 2.0; r < lengthNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      for (int t = -2.0; t < heightNum; t++) {
        pt.x = mid_x + r * _resolution + 1e-2;
        pt.y = mid_y + s * _resolution + 1e-2;
        pt.z = (t + 0.5) * _resolution + 1e-2;
        _cloud.points.push_back(pt);
      }
    }

  _cloud.width = _cloud.points.size();
  _cloud.height = 1;
  _cloud.is_dense = true;
}; // namespace dynamic_map_objects

class Box {
private:
  double _x, _y;
  double _w, _h;
  double _resolution;

public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;
  Box(double _x, double _y,
       double width = 0.6, double height = 2.0, double resolution = 0.1);
  ~Box() = default;
};

Box::Box(double x, double y,
           double width, double height, double resolution)
    : _x(x), _y(y),
      _w(width), _h(height), _resolution(resolution) {
	std::cout << "get into the constructor of Box" << std::endl;
  // define height and width
  int heightNum = ceil(_h / _resolution);
  int widNum = ceil(_w / _resolution);
//   int lengthNum = ceil(length / _resolution);

  // random point in grid scale
  double mid_x = floor(_x / _resolution) * _resolution +
                 _resolution / 2.0;
  double mid_y = floor(_y / _resolution) * _resolution +
                 _resolution / 2.0;

  // generate point cloud
  _cloud.points.resize(0);
  _cloud.width = 0;
  _cloud.height = 0;

  pcl::PointXYZ pt;
  // _cloud.points.push_back(pt);

  // NOTE: x direction is length, y direction is width
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      for (int t = -2.0; t < heightNum; t++) {
        pt.x = mid_x + r * _resolution + 1e-2;
        pt.y = mid_y + s * _resolution + 1e-2;
        pt.z = (t + 0.5) * _resolution + 1e-2;
        _cloud.points.push_back(pt);
      }
    }

  _cloud.width = _cloud.points.size();
  _cloud.height = 1;
  _cloud.is_dense = true;
}; // namespace dynamic_map_objects
} // namespace static_env

#endif // __MOVING_CYLINDER_H__
