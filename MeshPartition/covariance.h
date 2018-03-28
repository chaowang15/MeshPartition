#ifndef COVARIANCE_H
#define COVARIANCE_H
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
using namespace Eigen;

class CovObj
{
public:
	Matrix3d cov_;
	Vector3d normal_;
	Vector3d center_;
	double area_;
	int size_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	CovObj();
	CovObj(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3);
	CovObj& operator+=(const CovObj& Q);
	CovObj& operator-=(const CovObj& Q);
	double energy();
	void computePlaneNormal();
	void clearCov();

};


#endif