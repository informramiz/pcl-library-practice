#ifndef LINE_H_
#define LINE_H_

#include <math.h>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "model.h"

struct Line: Model {
	int a;
	int b;
	int c;

	Line() {
		a = 0;
		b = 0;
		c = 0;
	}

	Line(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
		a = point1.y - point2.y;
		b = point2.x - point1.x;
		c = (point1.x * point2.y) - (point2.x * point1.y);
	}

	float distanceFromPoint(const pcl::PointXYZ& point) const {
		return abs(a * point.x + b * point.y + c) / sqrtf(a * a + b * b);
	}
};

#endif