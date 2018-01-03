#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE "MathUtils"

#include <boost/test/unit_test.hpp>

#include "../math_util.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

#define eps 1e-6
#define eps2 1e-4


BOOST_AUTO_TEST_CASE(TestDegreesToRadians)
{
	// double
	double d_degrees = 180.0;
	double d_radians = Util::Math::degreesToRadians(d_degrees);
	BOOST_CHECK_LT(std::abs(d_radians - PI), eps);

	// float
	float f_degrees = 180.0f;
	float f_radians = Util::Math::degreesToRadians(f_degrees);
	BOOST_CHECK_LT(std::abs(f_radians - PI), eps);
}

BOOST_AUTO_TEST_CASE(TestRadiansToDegrees)
{
	// double
	double d_radians = PI;
	double d_degrees = Util::Math::radiansToDegrees(d_radians);
	BOOST_CHECK_LT(std::abs(d_degrees - 180.0), eps);

	// float
	float f_radians = PI;
	float f_degrees = Util::Math::radiansToDegrees(f_radians);
	BOOST_CHECK_LT(std::abs(f_degrees - 180.0f), eps);
}

BOOST_AUTO_TEST_CASE(TestSquare)
{
	// int
	int i_val = 2;
	BOOST_CHECK_LT(std::abs(Util::Math::square(i_val) - 4), eps);

	// float
	float f_val = 2.0f;
	BOOST_CHECK_LT(std::abs(Util::Math::square(f_val) - 4.0f), eps);

	// double
	double d_val = 2.0;
	BOOST_CHECK_LT(std::abs(Util::Math::square(d_val) - 4.0), eps);
}

BOOST_AUTO_TEST_CASE(TestCrossProductMatrix)
{
	// int
	Eigen::Vector3i i_e0(2,0,0);
	Eigen::Vector3i i_e1(0,2,0);
	Eigen::Matrix3i i_e0_mat = Util::Math::crossProductMatrix(i_e0);

	Eigen::Vector3i i_out_mat = i_e0_mat * i_e1;
	Eigen::Vector3i i_out_vec = i_e0.cross(i_e1);

	BOOST_CHECK_LT(std::abs(i_out_mat(0) - i_out_vec(0)), eps);
	BOOST_CHECK_LT(std::abs(i_out_mat(1) - i_out_vec(1)), eps);
	BOOST_CHECK_LT(std::abs(i_out_mat(2) - i_out_vec(2)), eps);

	// float
	Eigen::Vector3f f_e0(1.5,0,0);
	Eigen::Vector3f f_e1(0,1.5,0);
	Eigen::Matrix3f f_e0_mat = Util::Math::crossProductMatrix(f_e0);

	Eigen::Vector3f f_out_mat = f_e0_mat * f_e1;
	Eigen::Vector3f f_out_vec = f_e0.cross(f_e1);

	BOOST_CHECK_LT(std::abs(f_out_mat(0) - f_out_vec(0)), eps);
	BOOST_CHECK_LT(std::abs(f_out_mat(1) - f_out_vec(1)), eps);
	BOOST_CHECK_LT(std::abs(f_out_mat(2) - f_out_vec(2)), eps);

	// double
	Eigen::Vector3d d_e0(1.5,0,0);
	Eigen::Vector3d d_e1(0,1.5,0);
	Eigen::Matrix3d d_e0_mat = Util::Math::crossProductMatrix(d_e0);

	Eigen::Vector3d d_out_mat = d_e0_mat * d_e1;
	Eigen::Vector3d d_out_vec = d_e0.cross(d_e1);

	BOOST_CHECK_LT(std::abs(d_out_mat(0) - d_out_vec(0)), eps);
	BOOST_CHECK_LT(std::abs(d_out_mat(1) - d_out_vec(1)), eps);
	BOOST_CHECK_LT(std::abs(d_out_mat(2) - d_out_vec(2)), eps);
}

BOOST_AUTO_TEST_CASE(TestProjectPointToPlane)
{
	// int
	Eigen::Vector3i i_pt(3,2,1);
	Eigen::Vector3i i_plane_pt(1,0,0);
	Eigen::Vector3i i_normal(0,1,0);
	Eigen::Vector3i i_res = Util::Math::projectPointToPlane(i_normal, i_plane_pt, i_pt);

	BOOST_CHECK_LT(std::abs(i_res(0) - 3), eps);
	BOOST_CHECK_LT(std::abs(i_res(1) - 0), eps);
	BOOST_CHECK_LT(std::abs(i_res(2) - 1), eps);

	// float
	Eigen::Vector3f f_pt(3.0f,2.0f,1.0f);
	Eigen::Vector3f f_plane_pt(1.0f,0,0);
	Eigen::Vector3f f_normal(0,1.0f,0);
	Eigen::Vector3f f_res = Util::Math::projectPointToPlane(f_normal, f_plane_pt, f_pt);

	BOOST_CHECK_LT(std::abs(f_res(0)) - 3.0f, eps);
	BOOST_CHECK_LT(std::abs(f_res(1)) - 0, eps);
	BOOST_CHECK_LT(std::abs(f_res(2)) - 1.0f, eps);

	// double
	Eigen::Vector3d d_pt(3.0,2.0,1.0);
	Eigen::Vector3d d_plane_pt(1.0,0,0);
	Eigen::Vector3d d_normal(0,1.0,0);
	Eigen::Vector3d d_res = Util::Math::projectPointToPlane(d_normal, d_plane_pt, d_pt);

	BOOST_CHECK_LT(std::abs(d_res(0) - 3.0), eps);
	BOOST_CHECK_LT(std::abs(d_res(1) - 0), eps);
	BOOST_CHECK_LT(std::abs(d_res(2) - 1.0), eps);
}

BOOST_AUTO_TEST_CASE(TestPointsToPlaneNormal)
{
	// float
	Eigen::Matrix3f f_pts = Eigen::Matrix3f::Identity();
	Eigen::Vector3f f_normal = Util::Math::pointsToPlaneNormal(f_pts);

	BOOST_CHECK_LT(std::abs(f_normal(0) - 0.577350269f), eps);
	BOOST_CHECK_LT(std::abs(f_normal(1) - 0.577350269f), eps);
	BOOST_CHECK_LT(std::abs(f_normal(2) - 0.577350269f), eps);

	// double
	Eigen::Matrix3d d_pts = Eigen::Matrix3d::Identity();
	Eigen::Vector3d d_normal = Util::Math::pointsToPlaneNormal(d_pts);

	BOOST_CHECK_LT(std::abs(d_normal(0) - 0.577350269), eps);
	BOOST_CHECK_LT(std::abs(d_normal(1) - 0.577350269), eps);
	BOOST_CHECK_LT(std::abs(d_normal(2) - 0.577350269), eps);
}

BOOST_AUTO_TEST_CASE(TestCollinearityCheck)
{
	// float
	Eigen::Matrix3f f_pts0;
	f_pts0 << 1.0f, 1.0f, 5.0f,
			 0.0f, 4.0f, 4.0f,
			 0.0f, 4.0f, 4.0f;
	bool res = Util::Math::arePointsCollinear(f_pts0, 0.1745); // 10 degree threshod
	// Should be false
	BOOST_CHECK(!res);

	Eigen::Matrix3f f_pts1;
	f_pts1 << 1.0f, 2.0f, 3.0f,
			  2.0f, 4.0f, 6.0f,
			  3.0f, 6.0f, 9.5f;
	res = Util::Math::arePointsCollinear(f_pts1, 0.1745); // 10 degree threshod
	// Should be true
	BOOST_CHECK(res);

	// double
	Eigen::Matrix3d d_pts0;
	d_pts0 << 1.0, 1.0, 5.0,
			 0.0, 4.0, 4.0,
			 0.0, 4.0, 4.0;
	res = Util::Math::arePointsCollinear(d_pts0, 0.1745); // 10 degree threshod
	// Should be false
	BOOST_CHECK(!res);

	Eigen::Matrix3d d_pts1;
	d_pts1 << 1.0, 2.0, 3.0,
			  2.0, 4.0, 6.0,
			  3.0, 6.0, 9.5;
	res = Util::Math::arePointsCollinear(d_pts1, 0.1745); // 10 degree threshod
	// Should be true
	BOOST_CHECK(res);
}


BOOST_AUTO_TEST_CASE(TestPDist2)
{
	// float
	Eigen::MatrixXf f_X = Eigen::MatrixXf(3, 5);
	f_X << 8.147236863931790f, 9.133758561390193f, 2.784982188670484f, 9.648885351992766f, 9.571669482429456f,
		   9.057919370756192f, 6.323592462254095f, 5.468815192049838f, 1.576130816775483f, 4.853756487228412f,
		   1.269868162935061f, 0.975404049994095f, 9.575068354342976f, 9.705927817606156f, 8.002804688888002f;

	Eigen::MatrixXf f_Y = Eigen::MatrixXf(3, 4);
	f_Y << 1.418863386272153f, 7.922073295595544f, 0.357116785741896f, 6.787351548577734f,
		   4.217612826262750f, 9.594924263929030f, 8.491293058687772f, 7.577401305783335f,
		   9.157355251890671f, 6.557406991565868f, 9.339932477575505f, 7.431324681249162f;

	Eigen::MatrixXf f_D_gt = Eigen::MatrixXf(5, 4);
	f_D_gt << 130.9120296779811f, 28.2971397520759f, 126.1329742525787f, 42.0047682387019f,
			  130.8990832604867f, 43.3285497797096f, 151.6937025496513f, 48.7565736829524f,
			  3.6062721783778f  , 52.5207612142901f, 15.0851917495970f , 25.0607328280839f,
			  75.0116206225443f , 77.1961116182409f, 134.2903845096613f, 49.3774426182485f,
			  68.2059130014562f , 27.2890139698654f, 99.9275644320166f , 15.4972568535403f;

	Eigen::MatrixXf f_D_est = Util::Math::pdist2(f_X, f_Y);
	
	BOOST_CHECK_EQUAL(f_D_est.rows(), f_X.cols());
	BOOST_CHECK_EQUAL(f_D_est.cols(), f_Y.cols());
	for (int i = 0; i < f_D_gt.cols(); i++)
	{
		for (int j = 0; j < f_D_gt.rows(); j++)
		{
			BOOST_CHECK_LT(std::abs(f_D_gt(j, i) - f_D_est(j, i)), eps2);
		}
	}

	// double
	Eigen::MatrixXd d_X = Eigen::MatrixXd(3, 5);
	d_X << 8.147236863931790, 9.133758561390193, 2.784982188670484, 9.648885351992766, 9.571669482429456,
		   9.057919370756192, 6.323592462254095, 5.468815192049838, 1.576130816775483, 4.853756487228412,
		   1.269868162935061, 0.975404049994095, 9.575068354342976, 9.705927817606156, 8.002804688888002;

	Eigen::MatrixXd d_Y = Eigen::MatrixXd(3, 4);
	d_Y << 1.418863386272153, 7.922073295595544, 0.357116785741896, 6.787351548577734,
		   4.217612826262750, 9.594924263929030, 8.491293058687772, 7.577401305783335,
		   9.157355251890671, 6.557406991565868, 9.339932477575505, 7.431324681249162;

	Eigen::MatrixXd d_D_gt = Eigen::MatrixXd(5, 4);
	d_D_gt << 130.9120296779811, 28.2971397520759, 126.1329742525787, 42.0047682387019,
			  130.8990832604867, 43.3285497797096, 151.6937025496513, 48.7565736829524,
			  3.6062721783778  , 52.5207612142901, 15.0851917495970 , 25.0607328280839,
			  75.0116206225443 , 77.1961116182409, 134.2903845096613, 49.3774426182485,
			  68.2059130014562 , 27.2890139698654, 99.9275644320166 , 15.4972568535403;

	Eigen::MatrixXd d_D_est = Util::Math::pdist2(d_X, d_Y);

	BOOST_CHECK_EQUAL(d_D_est.rows(), d_X.cols());
	BOOST_CHECK_EQUAL(d_D_est.cols(), d_Y.cols());
	for (int i = 0; i < d_D_gt.cols(); i++)
	{
		for (int j = 0; j < d_D_gt.rows(); j++)
		{
			BOOST_CHECK_LT(std::abs(d_D_gt(j, i) - d_D_est(j, i)), eps);
		}
	}
}