#include "qemquadrics.h"

QEMQuadrics::QEMQuadrics()
{
	reset();
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)
{
	Vector3d e1 = v2 - v1, e2 = v3 - v1, p = (v1 + v2 + v3) / 3;
	e1.normalize();
	e2 -= (e2.dot(e1)) * e1; // make e2 orthogonal to e1
	e2.normalize();
	A_ = Matrix3d::Identity() - e1 * e1.transpose() - e2 * e2.transpose();
	b_ = -A_ * p;
	c_ = p.dot(-b_);
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1, const Vector3d& v2)
{
	Vector3d e1 = v2 - v1, p = (v1 + v2) / 2;
	e1.normalize();
	A_ = Matrix3d::Identity() - e1 * e1.transpose();
	b_ = -A_ * p;
	c_ = p.dot(-b_);
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1)
{
	A_ = Matrix3d::Identity();
	b_ = -v1;
	c_ = v1.dot(v1);
}

bool QEMQuadrics::optimize(Vector3d& v, double& energy)
{
	if (A_.determinant() < 1e-12) return false;
	v = -A_.inverse() * b_;
	energy = b_.dot(v) + c_;
	return true;
}

double QEMQuadrics::evaluate(const Vector3d& v) const
{
	return v.dot(A_*v) + 2*b_.dot(v) + c_;
}