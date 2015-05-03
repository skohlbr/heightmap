#ifndef HEIGHTMAP_POINTCLOUD_ITERATOR2_HPP
#define HEIGHTMAP_POINTCLOUD_ITERATOR2_HPP

#include "heightmap/pointcloud_point.hpp"

// #include <boost/iterator_adaptors.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace heightmap {

	template <class> class PointCloud2Iterator;
	template <typename T>
	PointCloud2Iterator<T> begin(const sensor_msgs::PointCloud2& pcl);
	template <typename T>
	PointCloud2Iterator<T> end(const sensor_msgs::PointCloud2& pcl);
	
	//
	// Lazy Pointcloud2 3D point iterator
	//
	template <class T>
	class PointCloud2Iterator
		: public boost::iterator_facade< PointCloud2Iterator<T>,
										 Point<T>,
										 boost::forward_traversal_tag >
	{
	private:
		enum {
			RECORD_SIZE = 20 // in bytes
		};

		const sensor_msgs::PointCloud2& pcl_;
		mutable Point<T> point_; // `mutable' needed for boost::iterator_facade
		int index_;
		unsigned int recordSize_;
		
		explicit PointCloud2Iterator(const sensor_msgs::PointCloud2& pcl,
									 bool initAtEnd = false)
			: pcl_(pcl),
			  index_(0), 
			  // TODO: evaluate bytes to skip from fields, do not use fixed values
			  recordSize_(RECORD_SIZE)
			{
				if (initAtEnd)
					index_ = pcl.width * pcl.height;
				load();
			}

		friend class boost::iterator_core_access;
		template <class> friend class PointCloud2Iterator;

		template <typename _T>
		friend PointCloud2Iterator<_T> begin(const sensor_msgs::PointCloud2& pcl);
		template <typename _T>
		friend PointCloud2Iterator<_T> end(const sensor_msgs::PointCloud2& pcl);
		
		template <class OtherT>
		bool equal(PointCloud2Iterator<OtherT> const& other) const {
			return (&pcl_ == &other.pcl_ && index_ == other.index_);
		}

		void load() {
			point_.x = *(T*) &pcl_.data[recordSize_*index_];
			point_.y = *(T*) &pcl_.data[recordSize_*index_ + sizeof(T)];
			point_.z = *(T*) &pcl_.data[recordSize_*index_ + 2*sizeof(T)];
		}

		void increment() {
			index_++;
			load();
		}

		Point<T>& dereference() const {
			return point_;
		}
	};

	template <typename T>
	PointCloud2Iterator<T> begin(const sensor_msgs::PointCloud2& pcl) {
		return PointCloud2Iterator<T>(pcl);
	}

	template <typename T>
	PointCloud2Iterator<T> end(const sensor_msgs::PointCloud2& pcl) {
		return PointCloud2Iterator<T>(pcl, true);
	}

}

#endif // HEIGHTMAP_POINTCLOUD_ITERATOR_HPP
