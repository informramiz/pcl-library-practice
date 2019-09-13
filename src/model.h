#ifndef MODEL_H_
#define MODEL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

class Model{
public:
	virtual float distanceFromPoint(const pcl::PointXYZ& point) const = 0;
};
#endif