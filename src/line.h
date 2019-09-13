#ifndef LINE_H_
#define LINE_H_

#include <math.h>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "model.h"

template<typename PointT>
struct Line: Model<PointT> {
	int a;
	int b;
	int c;

	Line() {
		a = 0;
		b = 0;
		c = 0;
	}

	Line(const PointT& point1, const PointT& point2) {
		a = point1.y - point2.y;
		b = point2.x - point1.x;
		c = (point1.x * point2.y) - (point2.x * point1.y);
	}

	float distanceFromPoint(const PointT& point) const {
		return abs(a * point.x + b * point.y + c) / sqrtf(a * a + b * b);
	}
};

#endif