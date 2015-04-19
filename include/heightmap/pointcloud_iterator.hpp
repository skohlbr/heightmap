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

template <class T>
class Pointcloud2Iterator
  : public boost::iterator_facade<
        Pointcloud2Iterator<T>
      , Point<T>
      , boost::forward_traversal_tag
    >
{

private:
    explicit Pointcloud2Iterator(const sensor_msgs::PointCloud2& pcl)
      : pcl(pcl),
      nextPointIndex(0),
      skipBytes(0)
      {
        // TODO: evalueate bytes to skip, use boost MPL at compile time
//         for(sensor_msgs::PointField t : pcl.fields) {
//           t.
//         }

        skipBytes = TOF_SKIP_BYTES;
      }

 public:
//     template <class OtherValue>
//     Pointcloud2Iterator(Pointcloud2Iterator<OtherValue> const& other)
//       : m_node(other.m_node) {}

  static Pointcloud2Iterator<T> begin(const sensor_msgs::PointCloud2& pcl) {
    return Pointcloud2Iterator(pcl);
  }

  static Pointcloud2Iterator<T> end(const sensor_msgs::PointCloud2& pcl) {
    Pointcloud2Iterator<T> it(pcl);
    it.goToEnd();
    return it;
  }

 private:
    friend class boost::iterator_core_access;
    template <class> friend class Pointcloud2Iterator;

    template <class OtherValue>
    bool equal(Pointcloud2Iterator<OtherValue> const& other) const
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

// typedef impl::node_iterator<node_base> node_iterator;
// typedef impl::node_iterator<node_base const> node_const_iterator;

}

#endif // HEIGHTMAP_POINTCLOUD_ITERATOR_HPP
