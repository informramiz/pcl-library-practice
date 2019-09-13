#ifndef MODEL_H_
#define MODEL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

template<typename PointT>
class Model{
public:
	virtual float distanceFromPoint(const PointT& point) const = 0;
};
#endif