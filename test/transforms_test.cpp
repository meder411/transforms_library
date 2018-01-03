#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "Transforms"

#include <boost/test/unit_test.hpp>

#include "../transforms.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <cmath>

using namespace Util::Transforms;

#define eps 1e-6

// TODO: Test singularities
// TODO: Test similarity transform composition and intermediate transforms


// Float test values
static const float F_RX = PI / 4;
static const float F_RY = PI / 4;
static const float F_RZ = PI / 4;
static float ARRAY_F_ROT[9] = {0.5000000000f, 0.5000000000f, -0.7071067811f,
					-0.1464466094f, 0.8535533906f, 0.5000000000f,
					0.8535533906f, -0.1464466094f, 0.5000000000f};
static const Eigen::Vector4f F_QUAT(0.8446231986f, 0.1913417162f, 0.4619397663f, 0.1913417162f);
static const Eigen::Vector4f F_QUAT_SQ(0.4267766953f, 0.3232233047f, 0.7803300859f, 0.3232233047f);
static const Eigen::Vector3f F_AXIS(0.3574067443f, 0.8628562095f, 0.3574067443f);
static const float F_THETA = 1.1298707548f;
static const Eigen::Vector3f F_PT(1.0f, 0.0f, 0.0f);
static const Eigen::Vector3f F_ROT_PT(0.5f, 0.5f, -0.707106781186547f);

// Double test values
static const double D_RX = PI / 4;
static const double D_RY = PI / 4;
static const double D_RZ = PI / 4;
static double ARRAY_D_ROT[9] = {0.5000000000, 0.5000000000, -0.7071067811,
					-0.1464466094, 0.8535533906, 0.5000000000,
					0.8535533906, -0.1464466094, 0.5000000000};
static const Eigen::Vector4d D_QUAT(0.8446231986, 0.1913417162, 0.4619397663, 0.1913417162);
static const Eigen::Vector4d D_QUAT_SQ(0.4267766953, 0.3232233047, 0.7803300859, 0.3232233047);
static const Eigen::Vector3d D_AXIS(0.3574067443, 0.8628562095, 0.3574067443);
static const double D_THETA = 1.1298707548;
static const Eigen::Vector3d D_PT(1.0, 0.0, 0.0);
static const Eigen::Vector3d D_ROT_PT(0.5, 0.5, -0.707106781186547);


BOOST_AUTO_TEST_CASE(TestRotationMatrixToEulerAngles)
{
	// float
	Eigen::Matrix3f f_rot = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	float f_rx;
	float f_ry;
	float f_rz;

	rotationMatrixToEulerAngles(f_rot, f_rx, f_ry, f_rz);

	BOOST_CHECK_LT(std::abs(f_rx - F_RX), eps);
	BOOST_CHECK_LT(std::abs(f_ry - F_RY), eps);
	BOOST_CHECK_LT(std::abs(f_rz - F_RZ), eps);

	// double
	Eigen::Matrix3d d_rot = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	double d_rx;
	double d_ry;
	double d_rz;
	rotationMatrixToEulerAngles(d_rot, d_rx, d_ry, d_rz);

	BOOST_CHECK_LT(std::abs(d_rx - D_RX), eps);
	BOOST_CHECK_LT(std::abs(d_ry - D_RY), eps);
	BOOST_CHECK_LT(std::abs(d_rz - D_RZ), eps);
}

BOOST_AUTO_TEST_CASE(TestEulerAnglesToRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot = eulerAnglesToRotationMatrix(F_RX, F_RY, F_RX);
	size_t k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot(j, i) - ARRAY_F_ROT[k++]), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot = eulerAnglesToRotationMatrix(D_RX, D_RY, D_RX);
	k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot(j, i) - ARRAY_D_ROT[k++]), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestAxisAngleToRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot = axisAngleToRotationMatrix(F_AXIS, F_THETA);
	size_t k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot(j, i) - ARRAY_F_ROT[k++]), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot = axisAngleToRotationMatrix(D_AXIS, D_THETA);
	k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot(j, i) - ARRAY_D_ROT[k++]), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestRotationMatrixToAxisAngle)
{
	// float
	Eigen::Matrix3f f_rot = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_axis;
	float f_theta;
	rotationMatrixToAxisAngle(f_rot, f_axis, f_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(f_axis(i) - F_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(f_theta - F_THETA), eps);

	// double
	Eigen::Matrix3d d_rot = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_axis;
	double d_theta;
	rotationMatrixToAxisAngle(d_rot, d_axis, d_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(d_axis(i) - D_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(d_theta - D_THETA), eps);
}

BOOST_AUTO_TEST_CASE(TestRotationMatrixToQuaternion)
{
	// float
	Eigen::Matrix3f f_rot = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector4f f_quat = rotationMatrixToQuaternion(f_rot);

	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_quat(i) - F_QUAT(i)), eps);
	}

	// double
	Eigen::Matrix3d d_rot = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector4d d_quat = rotationMatrixToQuaternion(d_rot);

	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_quat(i) - D_QUAT(i)), eps);
	}
}

BOOST_AUTO_TEST_CASE(TestQuaternionToRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot = quaternionToRotationMatrix(F_QUAT);
	size_t k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot(j, i) - ARRAY_F_ROT[k++]), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot = quaternionToRotationMatrix(D_QUAT);
	k = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot(j, i) - ARRAY_D_ROT[k++]), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestQuaternionToEulerAngles)
{
	// float
	float f_rx;
	float f_ry;
	float f_rz;
	quaternionToEulerAngles(F_QUAT, f_rx, f_ry, f_rz);

	BOOST_CHECK_LT(std::abs(f_rx - F_RX), eps);
	BOOST_CHECK_LT(std::abs(f_ry - F_RY), eps);
	BOOST_CHECK_LT(std::abs(f_rz - F_RZ), eps);

	// double
	double d_rx;
	double d_ry;
	double d_rz;
	quaternionToEulerAngles(D_QUAT, d_rx, d_ry, d_rz);

	BOOST_CHECK_LT(std::abs(d_rx - D_RX), eps);
	BOOST_CHECK_LT(std::abs(d_ry - D_RY), eps);
	BOOST_CHECK_LT(std::abs(d_rz - D_RZ), eps);
}

BOOST_AUTO_TEST_CASE(TestEulerAnglesToQuaternion)
{
	// float
	Eigen::Vector4f f_quat = eulerAnglesToQuaternion(F_RX, F_RY, F_RZ);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_quat(i) - F_QUAT(i)), eps);
	}

	// double
	Eigen::Vector4d d_quat = eulerAnglesToQuaternion(D_RX, D_RY, D_RZ);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(d_quat(i) - D_QUAT(i)), eps);
	}
}

BOOST_AUTO_TEST_CASE(TestAxisAngleToQuaternion)
{
	// float
	Eigen::Vector4f f_quat = axisAngleToQuaternion(F_AXIS, F_THETA);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_quat(i) - F_QUAT(i)), eps);
	}

	// double
	Eigen::Vector4d d_quat = axisAngleToQuaternion(D_AXIS, D_THETA);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(d_quat(i) - D_QUAT(i)), eps);
	}
}

BOOST_AUTO_TEST_CASE(TestQuaternionToAxisAngle)
{
	// float
	Eigen::Vector3f f_axis;
	float f_theta;
	quaternionToAxisAngle(F_QUAT, f_axis, f_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(f_axis(i) - F_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(f_theta - F_THETA), eps);

	// double
	Eigen::Vector3d d_axis;
	double d_theta;
	quaternionToAxisAngle(D_QUAT, d_axis, d_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(d_axis(i) - D_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(d_theta - D_THETA), eps);	
}

BOOST_AUTO_TEST_CASE(TestAxisAngleToEulerAngles)
{
	// float
	float f_rx;
	float f_ry;
	float f_rz;
	axisAngleToEulerAngles(F_AXIS, F_THETA, f_rx, f_ry, f_rz);
	BOOST_CHECK_LT(std::abs(f_rx - F_RX), eps);
	BOOST_CHECK_LT(std::abs(f_ry - F_RY), eps);
	BOOST_CHECK_LT(std::abs(f_rz - F_RZ), eps);

	// double
	double d_rx;
	double d_ry;
	double d_rz;
	axisAngleToEulerAngles(D_AXIS, D_THETA, d_rx, d_ry, d_rz);
	BOOST_CHECK_LT(std::abs(d_rx - D_RX), eps);
	BOOST_CHECK_LT(std::abs(d_ry - D_RY), eps);
	BOOST_CHECK_LT(std::abs(d_rz - D_RZ), eps);
}

BOOST_AUTO_TEST_CASE(TestEulerAnglesToAxisAngle)
{
	// float
	Eigen::Vector3f f_axis;
	float f_theta;
	eulerAnglesToAxisAngle(F_RX, F_RY, F_RZ, f_axis, f_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(f_axis(i) - F_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(f_theta - F_THETA), eps);

	// double
	Eigen::Vector3d d_axis;
	double d_theta;
	eulerAnglesToAxisAngle(D_RX, D_RY, D_RZ, d_axis, d_theta);
	for (int i = 0; i < 3; i++)
	{
		BOOST_CHECK_LT(std::abs(d_axis(i) - D_AXIS(i)), eps);
	}
	BOOST_CHECK_LT(std::abs(d_theta - D_THETA), eps);
}


BOOST_AUTO_TEST_CASE(TestRotationBetweenVectors)
{
	//float
	Eigen::Matrix3f f_rot_true;
	f_rot_true <<  0.577350269189626f, 0.577350269189626f, 0.577350269189626f,
			  -0.577350269189626f, 0.788675134594813f, -0.211324865405187f,
			  -0.577350269189626f, -0.211324865405187f, 0.788675134594813f;
	Eigen::Vector3f f_vec1(1,1,1);
	Eigen::Vector3f f_vec2(1,0,0);
	Eigen::Matrix3f f_rot_ret = rotationBetweenVectors(f_vec1, f_vec2);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot_true(j, i) - f_rot_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3f d_rot_true;
	d_rot_true <<  0.577350269189626, 0.577350269189626, 0.577350269189626,
			  -0.577350269189626, 0.788675134594813, -0.211324865405187,
			  -0.577350269189626, -0.211324865405187, 0.788675134594813;
	Eigen::Vector3f d_vec1(1,1,1);
	Eigen::Vector3f d_vec2(1,0,0);
	Eigen::Matrix3f d_rot_ret = rotationBetweenVectors(d_vec1, d_vec2);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot_true(j, i) - d_rot_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestIdentityQuaternion)
{
	// float
	Eigen::Vector4f f_iquat_true(1.0f, 0.0f, 0.0f, 0.0f);
	Eigen::Vector4f f_iquat_ret = identityQuaternion<float>();
	BOOST_CHECK_LT(std::abs(f_iquat_ret(0) - f_iquat_true(0)), eps);
	BOOST_CHECK_LT(std::abs(f_iquat_ret(1) - f_iquat_true(1)), eps);
	BOOST_CHECK_LT(std::abs(f_iquat_ret(2) - f_iquat_true(2)), eps);
	BOOST_CHECK_LT(std::abs(f_iquat_ret(3) - f_iquat_true(3)), eps);

	Eigen::Vector4d d_iquat_true(1.0, 0.0, 0.0, 0.0);
	Eigen::Vector4d d_iquat_ret = identityQuaternion<double>();
	BOOST_CHECK_LT(std::abs(d_iquat_ret(0) - d_iquat_true(0)), eps);
	BOOST_CHECK_LT(std::abs(d_iquat_ret(1) - d_iquat_true(1)), eps);
	BOOST_CHECK_LT(std::abs(d_iquat_ret(2) - d_iquat_true(2)), eps);
	BOOST_CHECK_LT(std::abs(d_iquat_ret(3) - d_iquat_true(3)), eps);
}

BOOST_AUTO_TEST_CASE(TestIdentityRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot_true;
	f_rot_true << 1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f;
	Eigen::Matrix3f f_rot_ret = identityRotationMatrix<float>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot_true(j, i) - f_rot_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot_true;
	d_rot_true << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
	Eigen::Matrix3d d_rot_ret = identityRotationMatrix<double>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot_true(j, i) - d_rot_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestXReflectionMatrix)
{
	// float
	Eigen::Matrix3f f_rot_true;
	f_rot_true << -1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f;
	Eigen::Matrix3f f_rot_ret = xReflectionMatrix<float>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot_true(j, i) - f_rot_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot_true;
	d_rot_true << -1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
	Eigen::Matrix3d d_rot_ret = xReflectionMatrix<double>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot_true(j, i) - d_rot_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestYReflectionMatrix)
{
	// float
	Eigen::Matrix3f f_rot_true;
	f_rot_true << 1.0f,0.0f,0.0f,0.0f,-1.0f,0.0f,0.0f,0.0f,1.0f;
	Eigen::Matrix3f f_rot_ret = yReflectionMatrix<float>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot_true(j, i) - f_rot_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot_true;
	d_rot_true << 1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,1.0;
	Eigen::Matrix3d d_rot_ret = yReflectionMatrix<double>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot_true(j, i) - d_rot_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestZReflectionMatrix)
{
	// float
	Eigen::Matrix3f f_rot_true;
	f_rot_true << 1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,-1.0f;
	Eigen::Matrix3f f_rot_ret = zReflectionMatrix<float>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot_true(j, i) - f_rot_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot_true;
	d_rot_true << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0;
	Eigen::Matrix3d d_rot_ret = zReflectionMatrix<double>();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot_true(j, i) - d_rot_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestNormalizeQuaternion)
{
	// float
	Eigen::Vector4f f_quat = Eigen::Vector4f::Random();
	Eigen::Vector4f f_n_quat = normalizeQuaternion(f_quat);
	float f_norm = f_n_quat.norm();
	BOOST_CHECK_LT(std::abs(f_norm - 1), eps);

	Eigen::Vector4f f_scale_n_quat = f_quat.norm() * f_n_quat;
	BOOST_CHECK_LT(std::abs(f_scale_n_quat(0) - f_quat(0)), eps);
	BOOST_CHECK_LT(std::abs(f_scale_n_quat(1) - f_quat(1)), eps);
	BOOST_CHECK_LT(std::abs(f_scale_n_quat(2) - f_quat(2)), eps);
	BOOST_CHECK_LT(std::abs(f_scale_n_quat(3) - f_quat(3)), eps);

	// double
	Eigen::Vector4d d_quat = Eigen::Vector4d::Random();
	Eigen::Vector4d d_n_quat = normalizeQuaternion(d_quat);
	float d_norm = d_n_quat.norm();
	BOOST_CHECK_LT(std::abs(d_norm - 1), eps);

	Eigen::Vector4d d_scale_n_quat = d_quat.norm() * d_n_quat;
	BOOST_CHECK_LT(std::abs(d_scale_n_quat(0) - d_quat(0)), eps);
	BOOST_CHECK_LT(std::abs(d_scale_n_quat(1) - d_quat(1)), eps);
	BOOST_CHECK_LT(std::abs(d_scale_n_quat(2) - d_quat(2)), eps);
	BOOST_CHECK_LT(std::abs(d_scale_n_quat(3) - d_quat(3)), eps);
}

BOOST_AUTO_TEST_CASE(TestInvertQuaternion)
{
	// float
	Eigen::Vector4f f_iquat(1.0f, 0.0f, 0.0f, 0.0f);
	Eigen::Vector4f f_iquat_ret = invertQuaternion(f_iquat);
	BOOST_CHECK_LT(std::abs(f_iquat_ret(0) - f_iquat(0)), eps);
	BOOST_CHECK_LT(std::abs(-f_iquat_ret(1) - f_iquat(1)), eps);
	BOOST_CHECK_LT(std::abs(-f_iquat_ret(2) - f_iquat(2)), eps);
	BOOST_CHECK_LT(std::abs(-f_iquat_ret(3) - f_iquat(3)), eps);

	// double
	Eigen::Vector4f d_iquat(1.0, 0.0, 0.0, 0.0);
	Eigen::Vector4f d_iquat_ret = invertQuaternion(d_iquat);
	BOOST_CHECK_LT(std::abs(d_iquat_ret(0) - d_iquat(0)), eps);
	BOOST_CHECK_LT(std::abs(-d_iquat_ret(1) - d_iquat(1)), eps);
	BOOST_CHECK_LT(std::abs(-d_iquat_ret(2) - d_iquat(2)), eps);
	BOOST_CHECK_LT(std::abs(-d_iquat_ret(3) - d_iquat(3)), eps);	
}

BOOST_AUTO_TEST_CASE(TestInvertRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Matrix3f f_rot_ret = invertRotationMatrix(f_rot);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_rot(j, i) - f_rot_ret(i, j)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_rot = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Matrix3d d_rot_ret = invertRotationMatrix(d_rot);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_rot(j, i) - d_rot_ret(i, j)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestComposeQuaternions)
{
	// float
	Eigen::Vector4f f_composed = composeQuaternions(F_QUAT, F_QUAT);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_composed(i) - F_QUAT_SQ(i)), eps);
	}

	// double
	Eigen::Vector4d d_composed = composeQuaternions(D_QUAT, D_QUAT);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(d_composed(i) - D_QUAT_SQ(i)), eps);
	}
}

BOOST_AUTO_TEST_CASE(TestComposeRotationMatrices)
{
	// float
	Eigen::Matrix3f f_composed_true;
	f_composed_true << 0.0f, 0.0f, 1.0f,
					   .707106781186547f, 0.707106781186548f, 0.0f,
					   -0.707106781186548f, 0.707106781186547f, 0.0f;
	Eigen::Matrix3f f_rot1;
	f_rot1 << 0.707106781186548f, -0.707106781186547f, 0.0f,
			  .707106781186547f, 0.707106781186548f, 0.0f,
			  0.0f, 0.0f, 1.0f;
	Eigen::Matrix3f f_rot2;
	f_rot2 << 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f;
	Eigen::Matrix3f f_composed_ret = composeRotationMatrices(f_rot1, f_rot2);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_composed_true(j, i) - f_composed_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix3d d_composed_true;
	d_composed_true << 0.0, 0.0, 1.0,
					   .707106781186547, 0.707106781186548, 0.0,
					   -0.707106781186548, 0.707106781186547, 0.0;
	Eigen::Matrix3d d_rot1;
	d_rot1 << 0.707106781186548, -0.707106781186547, 0.0,
			  .707106781186547, 0.707106781186548, 0.0,
			  0.0, 0.0, 1.0;
	Eigen::Matrix3d d_rot2;
	d_rot2 << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;

	Eigen::Matrix3d d_composed_ret = composeRotationMatrices(d_rot1, d_rot2);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_composed_true(j, i) - d_composed_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestConstructSimilarityTransform)
{
	// float
	Eigen::Matrix4f f_S_true;
	f_S_true <<  3.0f, -0.878679656440358f, 5.121320343559642f, 3.0f,
				 3.0f, 5.121320343559642f, -0.878679656440358f, 2.0f,
				 -4.242640687119285f, 3.0f, 3.0f, 1.0f,
				 0.0f, 0.0f, 0.0f, 1.0f;
	float f_scale = 6.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_scale);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(f_S_true(j, i) - f_S(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix4d d_S_true;
	d_S_true <<  3.0, -0.878679656440358, 5.121320343559642, 3.0,
				 3.0, 5.121320343559642, -0.878679656440358, 2.0,
				 -4.242640687119285, 3.0, 3.0, 1.0,
				 0.0, 0.0, 0.0, 1.0;
	double d_scale = 6.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_scale);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(d_S_true(j, i) - d_S(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestConstructSimilarityTransformNonUniformScale)
{
	// float
	Eigen::Matrix4f f_S_true;
	f_S_true << 3.0f, -0.585786437626905f, 1.707106781186547f, 3.0f,
				3.0f, 3.414213562373095f, -0.292893218813453f, 2.0f,
				-4.242640687119285f, 2.0f, 1.0f, 1.0f,
				0.0f, 0.0f, 0.0f, 1.0f;
	float f_xscale = 6.0f;
	float f_yscale = 4.0f;
	float f_zscale = 2.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_xscale, f_yscale, f_zscale);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(f_S_true(j, i) - f_S(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix4d d_S_true;
	d_S_true << 3.0, -0.585786437626905, 1.707106781186547, 3.0,
				3.0, 3.414213562373095, -0.292893218813453, 2.0,
				-4.242640687119285, 2.0, 1.0, 1.0,
				0.0, 0.0, 0.0, 1.0;
	double d_xscale = 6.0;
	double d_yscale = 4.0;
	double d_zscale = 2.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_xscale, d_yscale, d_zscale);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(d_S_true(j, i) - d_S(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestInvertSimilarityTransform)
{
	// float
	Eigen::Matrix4f f_S_inv_true;
	f_S_inv_true <<  0.083333333333333f, 0.083333333333333f, -0.117851130197758f, -0.298815536468909f,
				 -0.024407768234454f, 0.142258898432212f, 0.083333333333333f, -0.294627825494395f,
 				 0.142258898432212f, -0.024407768234454f, 0.083333333333333f, -0.461294492161061f,
				 0.0f, 0.0f, 0.0f, 1.0f;
	float f_scale = 6.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_scale);
	Eigen::Matrix4f f_S_inv_ret = invertSimilarityTransform(f_S);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(f_S_inv_true(j, i) - f_S_inv_ret(j, i)), eps);
		}
	}

	// double
	Eigen::Matrix4d d_S_inv_true;
	d_S_inv_true <<  0.083333333333333, 0.083333333333333, -0.117851130197758, -0.298815536468909,
				 -0.024407768234454, 0.142258898432212, 0.083333333333333, -0.294627825494395,
 				 0.142258898432212, -0.024407768234454, 0.083333333333333, -0.461294492161061,
				 0.0, 0.0, 0.0, 1.0;
	double d_scale = 6.0f;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_scale);
	Eigen::Matrix4d d_S_inv_ret = invertSimilarityTransform(d_S);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			BOOST_CHECK_LT(std::abs(d_S_inv_true(j, i) - d_S_inv_ret(j, i)), eps);
		}
	}
}

BOOST_AUTO_TEST_CASE(TestUniformScaleFromSimilarityTransform)
{
	// float
	float f_scale = 6.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_scale);
	float f_scale_ret = getScaleFromSimilarityTransform(f_S);
	BOOST_CHECK_LT(std::abs(f_scale - f_scale_ret), eps);

	// double
	double d_scale = 6.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_scale);
	double d_scale_ret = getScaleFromSimilarityTransform(d_S);
	BOOST_CHECK_LT(std::abs(d_scale - d_scale_ret), eps);
}

BOOST_AUTO_TEST_CASE(TestIndividualScalesFromSimilarityTransform)
{
	// float
	float f_xscale = 6.0f;
	float f_yscale = 4.0f;
	float f_zscale = 2.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_xscale, f_yscale, f_zscale);
	float f_xscale_ret = getScaleXFromSimilarityTransform(f_S);
	float f_yscale_ret = getScaleYFromSimilarityTransform(f_S);
	float f_zscale_ret = getScaleZFromSimilarityTransform(f_S);
	BOOST_CHECK_LT(std::abs(f_xscale - f_xscale_ret), eps);
	BOOST_CHECK_LT(std::abs(f_yscale - f_yscale_ret), eps);
	BOOST_CHECK_LT(std::abs(f_zscale - f_zscale_ret), eps);


	// double
	double d_xscale = 6.0;
	double d_yscale = 4.0;
	double d_zscale = 2.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_xscale, d_yscale, d_zscale);
	float d_xscale_ret = getScaleXFromSimilarityTransform(d_S);
	float d_yscale_ret = getScaleYFromSimilarityTransform(d_S);
	float d_zscale_ret = getScaleZFromSimilarityTransform(d_S);
	BOOST_CHECK_LT(std::abs(d_xscale - d_xscale_ret), eps);
	BOOST_CHECK_LT(std::abs(d_yscale - d_yscale_ret), eps);
	BOOST_CHECK_LT(std::abs(d_zscale - d_zscale_ret), eps);
}

BOOST_AUTO_TEST_CASE(TestDecomposeSimilarityTransform)
{
	// float
	Eigen::Matrix4f f_S_true;
	f_S_true <<  3.0f, -0.878679656440358f, 5.121320343559642f, 3.0f,
				 3.0f, 5.121320343559642f, -0.878679656440358f, 2.0f,
				 -4.242640687119285f, 3.0f, 3.0f, 1.0f,
				 0.0f, 0.0f, 0.0f, 1.0f;
	float f_scale = 6.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_scale);

	Eigen::Matrix3f f_R_ret = getRotationFromSimilarityTransform(f_S);
	Eigen::Vector3f f_t_ret = getTranslationFromSimilarityTransform(f_S);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(f_R_ret(j, i) - f_R(j, i)), eps);
		}
		BOOST_CHECK_LT(std::abs(f_t_ret(i) - f_t(i)), eps);
	}

	// double
	Eigen::Matrix4d d_S_true;
	d_S_true <<  3.0, -0.878679656440358, 5.121320343559642, 3.0,
				 3.0, 5.121320343559642, -0.878679656440358, 2.0,
				 -4.242640687119285, 3.0, 3.0, 1.0,
				 0.0, 0.0, 0.0, 1.0;
	double d_scale = 6.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_scale);
	Eigen::Matrix3d d_R_ret = getRotationFromSimilarityTransform(d_S);
	Eigen::Vector3d d_t_ret = getTranslationFromSimilarityTransform(d_S);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			BOOST_CHECK_LT(std::abs(d_R_ret(j, i) - d_R(j, i)), eps);
		}
		BOOST_CHECK_LT(std::abs(d_t_ret(i) - d_t(i)), eps);
	}
}

BOOST_AUTO_TEST_CASE(TestRotatePointByRotationMatrix)
{
	// float
	Eigen::Matrix3f f_rot = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_rot_pt = rotatePointByRotationMatrix(F_PT, f_rot);
	BOOST_CHECK_LT(std::abs(f_rot_pt(0) - F_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(1) - F_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(2) - F_ROT_PT(2)), eps);

	// double
	Eigen::Matrix3d d_rot = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_rot_pt = rotatePointByRotationMatrix(D_PT, d_rot);
	BOOST_CHECK_LT(std::abs(d_rot_pt(0) - D_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(1) - D_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(2) - D_ROT_PT(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestRotatePointByQuaternion)
{
	// float
	Eigen::Vector3f f_rot_pt = rotatePointByQuaternion(F_PT, F_QUAT);
	BOOST_CHECK_LT(std::abs(f_rot_pt(0) - F_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(1) - F_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(2) - F_ROT_PT(2)), eps);

	// double
	Eigen::Vector3d d_rot_pt = rotatePointByQuaternion(D_PT, D_QUAT);
	BOOST_CHECK_LT(std::abs(d_rot_pt(0) - D_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(1) - D_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(2) - D_ROT_PT(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestRotatePointByEulerAngles)
{
	// float
	Eigen::Vector3f f_rot_pt = rotatePointByEulerAngles(F_PT, F_RX, F_RY, F_RZ);
	BOOST_CHECK_LT(std::abs(f_rot_pt(0) - F_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(1) - F_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(2) - F_ROT_PT(2)), eps);

	// double
	Eigen::Vector3d d_rot_pt = rotatePointByEulerAngles(D_PT, D_RX, D_RY, D_RZ);
	BOOST_CHECK_LT(std::abs(d_rot_pt(0) - D_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(1) - D_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(2) - D_ROT_PT(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestRotatePointByAxisAngle)
{
	// float
	Eigen::Vector3f f_rot_pt = rotatePointByAxisAngle(F_PT, F_AXIS, F_THETA);
	BOOST_CHECK_LT(std::abs(f_rot_pt(0) - F_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(1) - F_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_rot_pt(2) - F_ROT_PT(2)), eps);

	// double
	Eigen::Vector3d d_rot_pt = rotatePointByAxisAngle(D_PT, D_AXIS, D_THETA);
	BOOST_CHECK_LT(std::abs(d_rot_pt(0) - D_ROT_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(1) - D_ROT_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_rot_pt(2) - D_ROT_PT(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestTransformPointBySimilarityTransform)
{
	// float
	Eigen::Vector3f f_pt(1.0f,0.0f,0.0f);
	Eigen::Vector3f f_t_pt_true = Eigen::Vector3f(6.0f, 5.0f, -3.242640687119285f);
	float f_scale = 6.0f;
	Eigen::Matrix3f f_R = Eigen::Map<Eigen::Matrix3f>(ARRAY_F_ROT);
	Eigen::Vector3f f_t(3.0f, 2.0f, 1.0f);
	Eigen::Matrix4f f_S = constructSimilarityTransform(f_R, f_t, f_scale);
	Eigen::Vector3f f_t_pt_ret = transformPointBySimilarityTransform(f_S, f_pt);
	BOOST_CHECK_LT(std::abs(f_t_pt_true(0) - f_t_pt_ret(0)), eps);
	BOOST_CHECK_LT(std::abs(f_t_pt_true(1) - f_t_pt_ret(1)), eps);
	BOOST_CHECK_LT(std::abs(f_t_pt_true(2) - f_t_pt_ret(2)), eps);

	// double
	Eigen::Vector3d d_pt(1.0,0.0,0.0);
	Eigen::Vector3d d_t_pt_true = Eigen::Vector3d(6.0, 5.0, -3.242640687119285);
	double d_scale = 6.0;
	Eigen::Matrix3d d_R = Eigen::Map<Eigen::Matrix3d>(ARRAY_D_ROT);
	Eigen::Vector3d d_t(3.0, 2.0, 1.0);
	Eigen::Matrix4d d_S = constructSimilarityTransform(d_R, d_t, d_scale);
	Eigen::Vector3d d_t_pt_ret = transformPointBySimilarityTransform(d_S, d_pt);
	BOOST_CHECK_LT(std::abs(d_t_pt_true(0) - d_t_pt_ret(0)), eps);
	BOOST_CHECK_LT(std::abs(d_t_pt_true(1) - d_t_pt_ret(1)), eps);
	BOOST_CHECK_LT(std::abs(d_t_pt_true(2) - d_t_pt_ret(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestReflectPoint)
{
	// float
	Eigen::Vector3f f_reflected = reflectPoint(F_PT, 0);
	BOOST_CHECK_LT(std::abs(-f_reflected(0) - F_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_reflected(1) - F_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_reflected(2) - F_PT(2)), eps);

	f_reflected = reflectPoint(F_PT, 1);
	BOOST_CHECK_LT(std::abs(f_reflected(0) - F_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(-f_reflected(1) - F_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(f_reflected(2) - F_PT(2)), eps);

	f_reflected = reflectPoint(F_PT, 2);
	BOOST_CHECK_LT(std::abs(f_reflected(0) - F_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(f_reflected(1) - F_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(-f_reflected(2) - F_PT(2)), eps);

	// double
	Eigen::Vector3d d_reflected = reflectPoint(D_PT, 0);
	BOOST_CHECK_LT(std::abs(-d_reflected(0) - D_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_reflected(1) - D_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_reflected(2) - D_PT(2)), eps);

	d_reflected = reflectPoint(D_PT, 1);
	BOOST_CHECK_LT(std::abs(d_reflected(0) - D_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(-d_reflected(1) - D_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(d_reflected(2) - D_PT(2)), eps);

	d_reflected = reflectPoint(D_PT, 2);
	BOOST_CHECK_LT(std::abs(d_reflected(0) - D_PT(0)), eps);
	BOOST_CHECK_LT(std::abs(d_reflected(1) - D_PT(1)), eps);
	BOOST_CHECK_LT(std::abs(-d_reflected(2) - D_PT(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestHamiltonianProduct)
{
	// float
	Eigen::Vector4f f_product = hamiltonianProduct(F_QUAT, F_QUAT);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(f_product(i) - F_QUAT_SQ(i)), eps);
	}

	// double
	Eigen::Vector4d d_product = hamiltonianProduct(D_QUAT, D_QUAT);
	for (int i = 0; i < 4; i++)
	{
		BOOST_CHECK_LT(std::abs(d_product(i) - D_QUAT_SQ(i)), eps);
	}
}