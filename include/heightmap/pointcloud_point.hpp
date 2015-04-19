#ifndef POINTCLOUD_POINT_HPP
#define POINTCLOUD_POINT_HPP

namespace heightmap {

template <typename T> struct Point {
  Point() {
    x = y = z = 0;
  }

  T x;
  T y;
  T z;
};

}

#endif // POINTCLOUD_POINT_HPP
