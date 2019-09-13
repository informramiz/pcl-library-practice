# PCL Practice

A repo in which I practice PCL library.

- Below is segmentation of road (green) and obstacles (red) using _RANSAC_ using _PCL_. 

	![obstacles+raod](demo/road+obstacles.png)
	
- Below is a line fitted to given points using RANSAC, with `maxIterations = 50`, `distanceTolerance = 0.8`. `Green points` are the inliers for line and `Red points` are outliers.

	![obstacles+raod](demo/fit-line.png)
- Below is a plane fitted to given set of points using RANSAC with `maxIterations = 50`, `distanceTolerance = 0.3`
	![obstacles+raod](demo/plane-fitted.png)
	
- Below is segmentation of road (green) and obstacles (red) using `plane fitting model` and `RANSAC` that I wrote without using _PCL_ library.

	![manual-obstancle-detection](demo/manual-plane-fit-obstacle-detection.png)