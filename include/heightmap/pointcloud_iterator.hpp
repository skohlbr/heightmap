#ifndef HEIGHTMAP_POINTCLOUD_ITERATOR_HPP
#define HEIGHTMAP_POINTCLOUD_ITERATOR_HPP

#include "heightmap/pointcloud_point.hpp"

#include <boost/iterator_adaptors.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace heightmap {

  enum {
    TOF_SKIP_BYTES = 20
  };


// Lazy Pointcloud2 3D point iterator

template <class T>
class PointCloud2Iterator
  : public boost::iterator_facade<
        PointCloud2Iterator<T>
      , Point<T>
      , boost::forward_traversal_tag
    >
{

private:
    explicit PointCloud2Iterator(const sensor_msgs::PointCloud2& pcl)
      : pcl(pcl),
      nextPointIndex(0),
      skipBytes(0)
      {
        // TODO: evaluate bytes to skip from fields, do not use fixed values
        skipBytes = TOF_SKIP_BYTES;
      }

 public:

  static PointCloud2Iterator<T> begin(const sensor_msgs::PointCloud2& pcl) {
    return PointCloud2Iterator(pcl);
  }

  static PointCloud2Iterator<T> end(const sensor_msgs::PointCloud2& pcl) {
    PointCloud2Iterator<T> it(pcl);
    it.goToEnd();
    return it;
  }

 private:
    friend class boost::iterator_core_access;
    template <class> friend class PointCloud2Iterator;

    template <class OtherValue>
    bool equal(PointCloud2Iterator<OtherValue> const& other) const
    {
        return this->nextPointIndex == other.nextPointIndex;
    }

    void load() const {
       nextPoint.x = *reinterpret_cast<const T*>(&pcl.data[TOF_SKIP_BYTES*nextPointIndex + 0 * sizeof(T)]);
       nextPoint.y = *reinterpret_cast<const T*>(&pcl.data[TOF_SKIP_BYTES*nextPointIndex + 1 * sizeof(T)]);
       nextPoint.z = *reinterpret_cast<const T*>(&pcl.data[TOF_SKIP_BYTES*nextPointIndex + 2 * sizeof(T)]);
    }

    void increment()
    {
       nextPointIndex++;
    }

    Point<T>& dereference() const
    {
      load();
      return nextPoint;
    }

    void goToEnd() {
      nextPointIndex = pcl.width*pcl.height;
    }

    const sensor_msgs::PointCloud2& pcl;
    mutable Point<T> nextPoint;
    int nextPointIndex;
    unsigned int skipBytes;

};

}

#endif // HEIGHTMAP_POINTCLOUD_ITERATOR_HPP
