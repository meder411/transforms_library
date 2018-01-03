#ifndef UTIL_LIB_MATH_H_
#define UTIL_LIB_MATH_H_

#include <Eigen/Core>

#include <cmath>

#include "assert.h"

const static double PI = 3.14159265358979323846;

namespace Eigen
{
	template <typename T, int dim>
	using MatrixDT = Eigen::Matrix<T, dim, dim>;

	template <typename T, int dim>
	using MatrixDXT = Eigen::Matrix<T, dim, Eigen::Dynamic>;

	template <typename T, int dim>
	using VectorDT = Eigen::Matrix<T, dim, 1>;

	template <typename T>
	using MatrixXT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
}


namespace Util
{
	namespace Math
	{
		// Only defined for float and double
		template <typename T>
		const T degreesToRadians(const T degrees) { return degrees * PI / 180.0; }

		// Only defined for float and double
		template <typename T>
		const T radiansToDegrees(const T radians) { return radians * 180.0 / PI; }

		// Defined for any type T
		template <typename T>
		const T square(const T val) { return val * val; }

		template <typename T>
		const Eigen::MatrixDT<T, 3> crossProductMatrix(const Eigen::VectorDT<T, 3>& cross)
		{
			Eigen::MatrixDT<T, 3> cross_mat;
			cross_mat <<	0,			-cross(2), 	cross(1),
							cross(2),	0,			-cross(0),
							-cross(1), 	cross(0), 	0;
			return cross_mat;
		}

		/** 
		 * Distance of a 3D point to a plane specified by a normal and a point on
		 * the plane.
		 *
		 * @param plane_normal						3D plane normal
		 * @param plane_pt							3D point on the plane
		 * @param pt								3D point to project
		 *
		 * @return 		Distance from point to plane
		 */
		template <typename T>
		const T pointToPlaneDistance(const Eigen::VectorDT<T, 3> &plane_normal,
			const Eigen::VectorDT<T, 3> &plane_pt, const Eigen::VectorDT<T, 3> &pt)
		{
			const Eigen::VectorDT<T, 3> vec = pt - plane_pt;
			return std::abs(vec.dot(plane_normal));
		}

		/** 
		 * Projects a 3D point onto a plane specified by a normal and a point on
		 * the plane.
		 *
		 * @param plane_normal						3D plane normal
		 * @param plane_pt							3D point on the plane
		 * @param pt								3D point to project
		 *
		 * @return 		Projection of the point on plane
		 */
		template <typename T>
		const Eigen::VectorDT<T, 3> projectPointToPlane(const Eigen::VectorDT<T, 3> &plane_normal,
			const Eigen::VectorDT<T, 3> &plane_pt, const Eigen::VectorDT<T, 3> &pt)
		{
			const Eigen::VectorDT<T, 3> vec = pt - plane_pt;
			const T dist = vec.dot(plane_normal);
			return pt - dist * plane_normal;
		}

		/** 
		 * Intersection of a 3D directional vector and a plane specified by a normal and a point on
		 * the plane.
		 *
		 * @param plane_normal						3D plane normal
		 * @param plane_pt							3D point on the plane
		 * @param vec								Directional 3D vector
		 * @param loc								Origin of directional vector
		 *
		 * @return 		Intersection point on plane
		 */
		template <typename T>
		const Eigen::VectorDT<T, 3> vectorPlaneIntersection(const Eigen::VectorDT<T, 3> &plane_normal,
			const Eigen::VectorDT<T, 3> &plane_pt, const Eigen::VectorDT<T, 3> &vec, const Eigen::VectorDT<T, 3> &loc)
		{
			const Eigen::VectorDT<T, 3> line = plane_pt - loc;
			const T dist = line.dot(plane_normal) / vec.dot(plane_normal);
			return loc + dist * vec;
		}

		/** 
		 * Computes the normal of a plane a parameterized by 3 points. Each column of the 
		 * input matrix is a 3D point.
		 *
		 * @param points							3x3 matrix where each column is a 3D point
		 *											on the plane
		 *
		 * @return 		3D plane normal
		 */
		template <typename T>
		const Eigen::VectorDT<T, 3> pointsToPlaneNormal(const Eigen::MatrixDT<T, 3> &points)
		{
			// Compute plane normal
			const Eigen::VectorDT<T, 3> line1 = (points.col(1) - points.col(0)).normalized();
			const Eigen::VectorDT<T, 3> line2 = (points.col(2) - points.col(0)).normalized();
			const Eigen::VectorDT<T, 3> plane_normal = line1.cross(line2);
			return plane_normal.normalized();
		}

		/** Determines whether 3 N-dimensional points are collinear
		*
		* @param points 							Nx3 matrix of points to check
		* @param threshold 							Angle (rad) beneath which points are considered collinear
		* 
		* @return 		True if collinear, false otherwise
		*/
		template <typename T, int D>
		const bool arePointsCollinear(const Eigen::Matrix<T, D, 3> &points, const double threshold)
		{
			const Eigen::VectorDT<T, D> line1 = (points.col(1) - points.col(0)).normalized();
			const Eigen::VectorDT<T, D> line2 = (points.col(2) - points.col(0)).normalized();
			const double angle12 = std::asin(line1.cross(line2).norm());

			// If angle between lines are a multiple of PI, they are collinear
			// If angle is nan, return true as well
			return fmod(std::abs(angle12), PI) < threshold || std::isnan(std::abs(angle12));
		}

		/**
		 * @brief Computes the all-pairs distance matrix between two sets of observations
		 * @details Outputs a (N x K) matrix containing the squared L2 norm between all pairs of observations
		 * 			between two sets. X and Y. X is a (M x N) matrix of N observations, each with M dimensions.
		 * 			Y is a (M x K) matrix of K observations, each with M dimensions. Only accurate to within 0.0001
		 * 			for float data. Double precision to at least 0.000001.
		 * 
		 * @param X 								M x N matrix of N observations each with M dimensions
		 * @param Y 								M x K matrix of K observations each with M dimensions
		 *
		 * @return 	N x K matrix of squared Euclidean (L2) distances
		 */
		template <typename T>
		const Eigen::MatrixXT<T> pdist2(const Eigen::MatrixXT<T> &X, const Eigen::MatrixXT<T> &Y)
		{
			ASSERT(X.rows() == Y.rows(), "Input observations must have same number of rows (" + 
				std::to_string(X.rows()) + "!=" + std::to_string(Y.rows()) + ")");

			const Eigen::MatrixXT<T> dists = X.colwise().squaredNorm().transpose() * Eigen::MatrixXT<T>::Ones(1, Y.cols())+
				Eigen::MatrixXT<T>::Ones(X.cols(), 1) * Y.colwise().squaredNorm() -
				2 * X.transpose() * Y;

			return dists;
		}
	}
}


#endif // UTIL_LIB_MATH_H_