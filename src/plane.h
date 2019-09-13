#ifndef PLANE_H_
#define PLANE_H_

#include <math.h>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "model.h"

struct Plane: Model {
	int a;
	int b;
	int c;
	int d;

	Plane() {
		a = 0;
		b = 0;
		c = 0;
		d = 0;
	}

	Plane(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2, const pcl::PointXYZ& point3) {
		//a = (y2−y1)(z3−z1)−(z2−z1)(y3−y1)
		a = (point2.y - point1.y)*(point3.z - point1.z) - (point2.z - point1.z)*(point3.y - point1.y);
		//b = (z2-z1)(x3-x1)-(x2-x1)(z3-z1)
		b = (point2.z - point1.z)*(point3.x - point1.x) - (point2.x - point1.x)*(point3.z - point1.z);
		//c = (x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		c = (point2.x - point1.x)*(point3.y-point1.y) - (point2.y - point1.y)*(point3.x-point1.x);
		// d = −(ix1+jy1+kz1)
		d = -(a*point1.x + b*point1.y + c*point1.z);
	}

	float distanceFromPoint(const pcl::PointXYZ& point) const {
		//d=∣A∗x+B∗y+C∗z+D∣/sqrt(A^2+B^2+C^2)
		return fabs(a*point.x + b*point.y + c*point.z + d) / sqrtf(a*a + b*b + c*c);
	}
};
#endif