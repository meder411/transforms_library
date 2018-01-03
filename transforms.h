/**
 * @file transforms.h
 */

#ifndef UTIL_LIB_TRANSFORMS_H_
#define UTIL_LIB_TRANSFORMS_H_

#include "math_util.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <random>
#include <cmath>
#include <string>

#define EPS 1e-6

namespace Util
{
	namespace Transforms
	{
		////////////////////
		// CONVERSIONS
		////////////////////

		/**
		 * @brief Decomposes a rotation matrix into rotation about the x, y, and z axes
		 * @details Extracts rotation around the X, Y, and Z axes from a given rotation matrix.
		 * 			This code assumes the rotations are in XYZ order. It checks for orthonormality
		 * 			and adjusts the matrix if necessary.
		 * 
		 * @param rot 	3x3 rotation matrix
		 * @param rx 	Rotation about the X-axis
		 * @param ry	Rotation about the Y-axis
		 * @param rz	Rotation about the Z-axis
		 */
		template<typename T>
		void rotationMatrixToEulerAngles(const Eigen::MatrixDT<T, 3> &rot, T &rx, T &ry, T &rz);

		/**
		 * @brief Converts a rotation matrix to a unit quaternion representation
		 * @details Extracts rotation around the X, Y, and Z axes from a given rotation matrix.
		 * 			This code assumes the rotations are in XYZ order. It checks for orthonormality
		 * 			and adjusts the matrix if necessary.
		 * 
		 * @param rot	3x3 Rotation matrix
		 * @return 		4x1 Unit quaternion (w, x, y, z)
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> rotationMatrixToQuaternion(const Eigen::MatrixDT<T, 3> &rot);
		
		/**
		 * @brief Converts a rotation matrix to an axis-angle representation
		 * @details Extracts rotation around the X, Y, and Z axes from a given rotation matrix.
		 * 			This code assumes the rotations are in XYZ order. It checks for orthonormality
		 * 			and adjusts the matrix if necessary.
		 * 
		 * @param rot	3x3 Rotation matrix
		 * @param axis	3x1 Unit vector (x, y, z)
		 * @param axis	Angle of CCW rotation around axis
		 */
		template<typename T>
		void rotationMatrixToAxisAngle(const Eigen::MatrixDT<T, 3> &rot, Eigen::VectorDT<T, 3> &axis, T &theta);

		/**
		 * @brief Converts a quaternion to rotation
		 * @details 
		 * 
		 * @param quat	4x1 Quaternion (w, x, y, z)
		 * @return 		3x3 Rotation matrix s.t. x' = R * X (i.e. row major)
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 3> quaternionToRotationMatrix(const Eigen::VectorDT<T, 4> &quat);

		template<typename T>
		void quaternionToAxisAngle(const Eigen::VectorDT<T, 4> &quat, Eigen::VectorDT<T, 3> &axis, T &theta);

		template<typename T>
		void quaternionToEulerAngles(const Eigen::VectorDT<T, 4> &quat, T &rx, T &ry, T &rz);

		template<typename T>
		const Eigen::MatrixDT<T, 3> eulerAnglesToRotationMatrix(const T rx, const T ry, const T rz);

		template<typename T>
		const Eigen::VectorDT<T, 4> eulerAnglesToQuaternion(const T rx, const T ry, const T rz);

		template<typename T>
		void eulerAnglesToAxisAngle(const T rx, const T ry, const T rz, Eigen::VectorDT<T, 3> &axis, T &theta);

		template<typename T>
		const Eigen::MatrixDT<T, 3> axisAngleToRotationMatrix(const Eigen::VectorDT<T, 3> &axis, const T theta);

		template<typename T>
		const Eigen::VectorDT<T, 4> axisAngleToQuaternion(const Eigen::VectorDT<T, 3> &axis, const T &theta);

		template<typename T>
		void axisAngleToEulerAngles(const Eigen::VectorDT<T, 3> &axis, const T theta, T &rx, T &ry, T &rz);

		// Rotation from vec1 to vec2
		template<typename T>
		const Eigen::MatrixDT<T, 3> rotationBetweenVectors(const Eigen::VectorDT<T, 3> &vec1, const Eigen::VectorDT<T, 3> &vec2);


		////////////////////
		// PRESETS
		////////////////////
		template<typename T>
		const Eigen::VectorDT<T, 4> identityQuaternion();

		template<typename T>
		const Eigen::MatrixDT<T, 3> identityRotationMatrix();

		template<typename T>
		const Eigen::MatrixDT<T, 3> xReflectionMatrix();

		template<typename T>
		const Eigen::MatrixDT<T, 3> yReflectionMatrix();

		template<typename T>
		const Eigen::MatrixDT<T, 3> zReflectionMatrix();


		////////////////////
		// RANDOMS
		////////////////////

		/**
		 * @brief Generates a random axis-angle rotation
		 * @details Initializes the parameters to be a random unit vector and a uniformly random values on the closed
		 * interval [0, 2*PI]
		 * 
		 * @param axis 	3-Vector parameter to take the rotation axis
		 * @param angle Parameter to take the rotation angle
		 */
		template<typename T>
		void randomAxisAngle(Eigen::VectorDT<T, 3> &axis, T &angle);
		
		/**
		 * @brief Generates a random Euler-angle rotation
		 * @details Initializes the parameters to be uniformly random values on the closed interval [0, 2*PI]
		 * 
		 * @param rx Parameter to take the rotation angle about the X-axis
		 * @param ry Parameter to take the rotation angle about the Y-axis
		 * @param rz Parameter to take the rotation angle about the Z-axis
		 */
		template<typename T>
		void randomEulerAngles(T &rx, T &ry, T &rz);

		/**
		 * @brief Generates a random unit quaternion rotation
		 * @details Uniformly randomly generates a normalized 4-vector
		 * 
		 * @return Random unit quaternion
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> randomQuaternion();

		/**
		 * @brief Generates a random orthonormal rotation matrix
		 * @details Generates two random 3-vectors X and Y_tmp. Determines an orthogonal 3-vector Y = cross(X, Y_tmp). Then determines
		 * a 3-vector Z that is orthogonal to both and and Y as Z = cross(X, Z). Finally normalizes each vector and composes the
		 * rotatiom matrix R = [X, Y, Z] where each vector is a column of the rotation matrix
		 * 
		 * @return Random rotation matrix
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 3> randomRotationMatrix();


		////////////////////
		// MANIPULATIONS
		////////////////////

		/**
		 * @brief Normalizes a quaternion
		 * @details Returns the quaternion divided by the L2 norm. In the case where the norm(q) is 0, this function
		 * returns q' = [1, imag(q)]
		 *
		 * @param quat Quaternion
		 * 
		 * @return Normalized quaternion
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> normalizeQuaternion(const Eigen::VectorDT<T, 4> &quat);

		/** 
		 * Averages two quaternions according to the SLERP algorithm (mean when t=0.5)
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> averageTwoQuaternions(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2, const T t);

		/**
		 * @brief Composes two rotations represented by quaternions
		 * @details Computes the hamiltonion product between the two quaternions
		 * 
		 * @param quat1
		 * @param quat2
		 * @return quat2 * quat1
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> composeQuaternions(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2);

		/**
		 * @brief Composes two rotations represented by rotation matrices
		 * @details Computes product of the rotation matrices
		 * 
		 * @param rot1
		 * @param rot2
		 * @return rot2 * rot1
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 3> composeRotationMatrices(const Eigen::MatrixDT<T, 3> &rot1, const Eigen::MatrixDT<T, 3> &rot2);

		/**
		 * @brief Inverts a quaternion
		 * @details  Flips the sign of the imaginary components. This has the effect
		 * of providing the rotation in the opposite direction.
		 * 
		 * @return Quaternion representing the inverse rotation
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> invertQuaternion(const Eigen::VectorDT<T, 4> &quat);

		/**
		 * @brief Inverts a rotation matrix
		 * @details  Transposes the rotation matrix. As the matrix must by definition be orthonormal,
		 * the transpose operation is equivalent to an inversion.
		 * 
		 * @return Rotation matrix representing the inverse rotation
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 3> invertRotationMatrix(const Eigen::MatrixDT<T, 3> &rot);

		/**
		 * @brief Constructs a 4x4 similarity transform with uniform scaling
		 * @details First applies a scaling transform, followed by rotation, followed by a translational shift
		 * 
		 * 	\f[
		 * 		T = \left[
		 * 			\begin{array}{cc}
		 * 				RS & t \\
		 * 				0  &  1
		 * 			\end{array}
		 * 			\right],
		 * 			
		 * 			\text{ where }
		 * 		S = \left[
		 * 			\begin{array}{ccc}
		 * 				s & 0 & 0 \\
		 * 				0 & s & 0 \\
		 * 				0 & 0 & s 
		 * 			\end{array}
		 * 			\right],
		 * 
		 * 	\f]
		 * 
		 * @param rot 	3x3 rotation matrix
		 * @param trans 3D vector defining translation
		 * @param scale scale factor
		 * 
		 * @return 4x4 similarity transform T such that X' = T * X = R*S*X + t where R is the rotation, S is a diagonal scaling matrix, 
		 * and t is a translation vector
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 4> constructSimilarityTransform(const Eigen::MatrixDT<T, 3> &rot, const Eigen::VectorDT<T, 3> &trans, const T scale);

		/**
		 * @brief Constructs a 4x4 similarity transform with uniform scaling
		 * @details First applies a scaling transform, followed by rotation, followed by a translational shift
		 * 
		 * 		S = diag(sx, sy, sz)
		 * 
		 * 		T =	[ RS | t ]
		 * 			[  0 | 1 ]
		 * 
		 * @param rot 	3x3 rotation matrix
		 * @param trans 3D vector defining translation
		 * @param sx	scale factor along x-direction
		 * @param sy	scale factor along y-direction
		 * @param sz	scale factor along z-direction
		 * 
		 * @return 4x4 similarity transform T such that X' = T * X = R*S*X + t where R is the rotation, S is a diagonal scaling matrix, 
		 * and t is a translation vector
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 4> constructSimilarityTransform(const Eigen::MatrixDT<T, 3> &rot, const Eigen::VectorDT<T, 3> &trans,
			const T sx, const T sy, const T sz);

		/**
		 * @brief Inverts a 4x4 similarity transform
		 * @details Analytical inverse of 4x4 similarity matrix that works regardless of uniform scaling
		 * 
		 * 		T^-1 =	[ S^-1 * R^-1 | - S^-1 * R^-1 * t ]
		 * 				[      0	  |         1         ]
		 * 
		 * @param similarity	4x4 similarity transform
		 * 
		 * @return 4x4 similarity transform T^-1 such that X = T^-1 * X' = S^-1 * R^-1 * X' - S^-1 * R^-1 * t
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 4> invertSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		* @brief Composes two similarity transformations
		* @details Left multiplies similarity2 by similarity1  as in S_new = S1 * S2
		* 
		* @param similarity1 Operative similarity transform
		* @param similarity2 Similarity transform to be transformed
		* @return Composed transform S1*S2
		*/
		template<typename T>
		const Eigen::MatrixDT<T, 4> composeSimilarityTransforms(const Eigen::MatrixDT<T, 4> &similarity1, const Eigen::MatrixDT<T, 4> &similarity2);

		/**
		 * @brief Finds the transformation from one similarity transfom to another
		 * @details Computes the transformation *from* similarity1 *to* similarity2 such that S_12 = S_2 * inv(S_1)
		 * 
		 * @param similarity1 Source transform
		 * @param similarity2 Destination transform
		 * @return Similarity transform T that transforms similarity1 to similarity2 as S2 = T * S1
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 4> getTransformBetweenSimilarityTransforms(const Eigen::MatrixDT<T, 4> &similarity1, const Eigen::MatrixDT<T, 4> &similarity2);

		template<typename T>
		void composeQuaternionTransformations(const Eigen::VectorDT<T, 4> &in_quat1, const Eigen::VectorDT<T, 3> &in_trans1,
			const Eigen::VectorDT<T, 4> &in_quat2, const Eigen::VectorDT<T, 3> &in_trans2,
			Eigen::VectorDT<T, 4> &out_quat, Eigen::VectorDT<T, 3> &out_trans);

		/**
		 * @brief Extracts a uniform scale factor from the similarity transform
		 * @details Returns the norm of the first column of the similarity transform. In the case of uniform scaling, this norm will
		 * be the scale factor. If there is non-uniform scaling, use the getScale*FromSimilarityTransform functions instead.
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Uniform scale factor in the similarity transform
		 */
		template<typename T>
		const T getScaleFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Extracts a the scale factor in the Y direction from the similarity transform
		 * @details Returns the norm of the first column of the similarity transform.
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Y scale factor in the similarity transform
		 */
		template<typename T>
		const T getScaleXFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Extracts a the scale factor in the Y direction from the similarity transform
		 * @details Returns the norm of the second column of the similarity transform.
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Y scale factor in the similarity transform
		 */
		template<typename T>
		const T getScaleYFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Extracts a the scale factor in the Z direction from the similarity transform
		 * @details Returns the norm of the third column of the similarity transform.
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Z scale factor in the similarity transform
		 */
		template<typename T>
		const T getScaleZFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Extracts a the rotation from the similarity transform
		 * @details Returns the orthonormal rotationm matrix from the similarity transform with any scale factors removed
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Scale-less rotation matrix
		 */
		template<typename T>
		const Eigen::MatrixDT<T, 3> getRotationFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Extracts a the translation vector from the similarity transform
		 * @details Returns the fourht column of the similarity transform.
		 * 
		 * @param similarity Similarity transform
		 *
		 * @return Translation vector
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> getTranslationFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity);

		/**
		 * @brief Inverts a quaternion and translation vector pair in place
		 * @details Given a rotation expressed as a quaternion and a 3x1 translation vector s.t. X' = qX + t,
		 * returns q' and t' s.t. X = q'X' + t'
		 * 
		 * @param quat 4x1 quaternion (w, x, y, z)
		 * @param trans 3x1 translation vector
		 */
		template<typename T>
		void getInverseRotationAndTranslation(Eigen::VectorDT<T, 4> &quat, Eigen::VectorDT<T, 3> &trans);


		//////////////////////
		// TRANSFORMATIONS
		//////////////////////

		/**
		 * @brief Rotates a 3D point by a quaternion
		 * @details Performs a rotation on a point by computing the Hamiltonian between the quaternion and the
		 * homogeneous representation of the point
		 * 
		 * @param pt 3D points
		 * @param quat 4x1 quaternion (w, x, y, z)
		 *
		 * @return Rotated point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByQuaternion(const Eigen::VectorDT<T, 3> &pt, const Eigen::VectorDT<T, 4> &quat);

		/**
		 * @brief Rotates a 3D point by a rotation matrix
		 * @details Performs a rotation on a point by left multiplying the point by the rotation matrix.
		 * 
		 * @param pt 3D points
		 * @param rot 3x3 orthonormal rotation matrix
		 *
		 * @return Rotated point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByRotationMatrix(const Eigen::VectorDT<T, 3> &pt, const Eigen::MatrixDT<T, 3> &rot);

		/**
		 * @brief Rotates a 3D point given a set of Euler angles
		 * @details Performs a rotation on a point around the X, Y, and Z axes, in that order, by the angles provided. First
		 * forms a rotation matrix using the provided angles and subsequently left-multiplies the point.
		 * 
		 * @param pt 3D points
		 * @param rx Rotation about the X axis
		 * @param ry Rotation about the Y axis
		 * @param rz Rotation about the Z axis
		 *
		 * @return Rotated point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByEulerAngles(const Eigen::VectorDT<T, 3> &pt, const T rx, const T ry, const T rz);

		/**
		 * @brief Rotates a point around a given axis k by angle theta
		 * @details Rotates a point using the Rodrigues rotation formula given as
		 *
		 * 			X' =  X*cos(theta) + cross(k, X)*sin(theta) + k*(dot(k, X))(1-cos(theta))
		 * 
		 * @param axis 	axis of rotation
		 * @param theta angle of rotation around the axis according to the right hand rule
		 * 
		 * @return The rotated point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByAxisAngle(const Eigen::VectorDT<T, 3> &pt, const Eigen::VectorDT<T, 3> &axis, const T theta);

		/**
		 * @brief Transforms a point by applying a similarity transformation to it
		 * @details Transforms a 3D point in homogeneous space as
		 *
		 * 			X' = S * X
		 * 
		 * @param similarity 	Rigid body similarity transformation
		 * @param pt 			Original point
		 * 
		 * @return The transformed 3D point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> transformPointBySimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity, const Eigen::VectorDT<T, 3> &pt);

		/**
		 * @brief Transforms a set of points by applying a similarity transformation to it
		 * @details Transforms a set of 3D points as a 4xN homogeneous matrix as
		 *
		 * 			X' = S * X
		 * 
		 * @param similarity 	Rigid body similarity transformation
		 * @param pt 			3xN matrix of points
		 * 
		 * @return The transformed 3D points as a 3xN matrix
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> transformPointSetBySimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity, const Eigen::MatrixDXT<T, 3> &pts);

		/**
		 * @brief Reflect a point across an axis
		 * @details Reflects a point across the axis specified by dim by negating that dimension
		 *
		 * @param pt 	3D point
		 * @param dim 	Dimension to reflect
		 * 
		 * @return Reflected point
		 */
		template<typename T>
		const Eigen::VectorDT<T, 3> reflectPoint(const Eigen::VectorDT<T, 3> &pt, const size_t dim);

		/**
		 * @brief Computes the hamiltonion product between quaternions such that q_ret = quat1 * quat2;
		 * 
		 * @param quat1
		 * @param quat2
		 * @return quat1 * quat2
		 */
		template<typename T>
		const Eigen::VectorDT<T, 4> hamiltonianProduct(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2);



		// *************************************************** //


		/////////////////////
		// IMPLEMENTATIONS
		/////////////////////

		template<typename T>
		void rotationMatrixToEulerAngles(const Eigen::MatrixDT<T, 3> &rot, T &rx, T &ry, T &rz)
		{
			Eigen::MatrixDT<T, 3> n_rot = rot;
			const double det = rot.determinant();
			if (det != 1)
			{
				ASSERT(det >= EPS, "Invalid rotation matrix (det~=0)");
				Eigen::VectorDT<T, 3> scale(rot.template block<3,1>(0,0).norm(), rot.template block<3,1>(0,1).norm(), rot.template block<3,1>(0,2).norm());
				n_rot = scale.asDiagonal() * n_rot;
			}

			rx = std::atan2(n_rot(2, 1), n_rot(2, 2));
			ry = std::atan2(-n_rot(2, 0), std::sqrt(Util::Math::square(n_rot(2, 1)) + Util::Math::square(n_rot(2, 2))));
			rz = std::atan2(n_rot(1, 0), n_rot(0, 0));

			rx = std::isnan(rx) ? 0 : rx;
			ry = std::isnan(ry) ? 0 : ry;
			rz = std::isnan(rz) ? 0 : rz;
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> rotationMatrixToQuaternion(const Eigen::MatrixDT<T, 3> &rot)
		{
			Eigen::MatrixDT<T, 3> n_rot = rot;
			const double det = rot.determinant();
			if (det != 1)
			{
				ASSERT(det >= EPS, "Invalid rotation matrix (det~=0)");
				Eigen::VectorDT<T, 3> scale(rot.template block<3,1>(0,0).norm(), rot.template block<3,1>(0,1).norm(), rot.template block<3,1>(0,2).norm());
				n_rot = scale.asDiagonal() * rot;
			}

			Eigen::Quaternion<T> quat(n_rot);
			return Eigen::VectorDT<T, 4>(quat.w(), quat.x(), quat.y(), quat.z());
		}
		
		template<typename T>
		void rotationMatrixToAxisAngle(const Eigen::MatrixDT<T, 3> &rot, Eigen::VectorDT<T, 3> &axis, T &theta)
		{
			Eigen::MatrixDT<T, 3> n_rot = rot;
			const double det = rot.determinant();
			if (det != 1)
			{
				ASSERT(det >= EPS, "Invalid rotation matrix (det~=0)");
				const Eigen::VectorDT<T, 3> scale(rot.template block<3,1>(0,0).norm(), rot.template block<3,1>(0,1).norm(), rot.template block<3,1>(0,2).norm());
				n_rot = scale.asDiagonal() * rot;
			}

			theta = std::acos((n_rot(0,0) + n_rot(1,1) + n_rot(2,2) - 1) / 2);
			
			// Handle case of 0 rotation
			if (std::abs(theta) < EPS)
			{
				theta = 0.0;
				axis = Eigen::VectorDT<T, 3>(1.0, 0.0, 0.0); // Arbitrary vector
			}
			// Handle case of 180 degree rotation
			// Simplest to just go through quaternion
			else if (std::abs(std::abs(theta) - PI) < EPS)
			{
				quaternionToAxisAngle(rotationMatrixToQuaternion(n_rot), axis, theta);
			}
			axis(0) = (n_rot(2,1) - n_rot(1,2));
			axis(1) = (n_rot(0,2) - n_rot(2,0));
			axis(2) = (n_rot(1,0) - n_rot(0,1));
			axis = axis.normalized();
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> quaternionToRotationMatrix(const Eigen::VectorDT<T, 4> &quat)
		{
			const Eigen::VectorDT<T, 4> n_quat = normalizeQuaternion(quat);
			const Eigen::Quaternion<T> eig_quat(n_quat(0), n_quat(1), n_quat(2), n_quat(3));
			return eig_quat.toRotationMatrix();
		}

		template<typename T>
		void quaternionToAxisAngle(const Eigen::VectorDT<T, 4> &quat, Eigen::VectorDT<T, 3> &axis, T &theta)
		{
			const Eigen::VectorDT<T, 4> n_quat = normalizeQuaternion(quat);
			
			theta = 2 * std::atan2(n_quat.template tail<3>().norm(), n_quat(0));

			// Singularity at 0 degrees
			if (std::abs(theta) < EPS)
			{
				theta = 0.0;
				axis = Eigen::VectorDT<T, 3>(1.0, 0.0, 0.0); // Arbitrary vector
			}
			// Singularity at 180 degrees
			else if(std::abs(theta - PI) < EPS)
			{
				theta = PI;
				axis = n_quat.template tail<3>();
			}
			// General case
			else
			{
				axis = n_quat.template tail<3>() / std::sin(theta / 2);
			}
			axis = axis.normalized();
		}

		template<typename T>
		void quaternionToEulerAngles(const Eigen::VectorDT<T, 4> &quat, T &rx, T &ry, T &rz)
		{
			const Eigen::VectorDT<T, 4> n_quat = normalizeQuaternion(quat);

			// Straight up singularity
			if (std::abs(n_quat(1)*n_quat(2) + n_quat(3)*n_quat(0) - 0.5) < EPS)
			{
				rx = 0;
				ry = std::asin(2*n_quat(1)*n_quat(2) + 2*n_quat(3)*n_quat(0));
				rz = 2 * atan2(n_quat(1), n_quat(0));
			}
			// Straight down singularity
			else if (std::abs(n_quat(1)*n_quat(2) + n_quat(3)*n_quat(0) + 0.5) < EPS)
			{
				rx = 0;
				ry = std::asin(2*n_quat(1)*n_quat(2) + 2*n_quat(3)*n_quat(0));
				rz = -2 * atan2(n_quat(1), n_quat(0));

			}
			// General case
			else
			{
				rx = std::atan2(2*n_quat(1)*n_quat(0) + 2*n_quat(2)*n_quat(3), 1 - 2*Util::Math::square(n_quat(1)) - 2*Util::Math::square(n_quat(2)));
				ry = std::asin(2*n_quat(0)*n_quat(2) - 2*n_quat(3)*n_quat(1));
				rz = std::atan2(2*n_quat(0)*n_quat(3) + 2*n_quat(1)*n_quat(2), 1 - 2*Util::Math::square(n_quat(2)) - 2*Util::Math::square(n_quat(3)));
			}
		}


		template<typename T>
		const Eigen::MatrixDT<T, 3> eulerAnglesToRotationMatrix(const T rx, const T ry, const T rz)
		{
			Eigen::MatrixDT<T, 3> Rx;
			Rx << 1, 0, 0, 0, std::cos(rx), -std::sin(rx), 0, std::sin(rx), std::cos(rx);
			Eigen::MatrixDT<T, 3> Ry;
			Ry << std::cos(ry), 0, std::sin(ry), 0, 1, 0, -std::sin(ry), 0, std::cos(ry);
			Eigen::MatrixDT<T, 3> Rz;
			Rz << std::cos(rz), -std::sin(rz), 0, std::sin(rz), std::cos(rz), 0, 0, 0, 1;

			// XYZ order
			return Rz * Ry * Rx;
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> eulerAnglesToQuaternion(const T rx, const T ry, const T rz)
		{
			const double t0 = std::cos(rz * 0.5);
			const double t1 = std::sin(rz * 0.5);
			const double t2 = std::cos(rx * 0.5);
			const double t3 = std::sin(rx * 0.5);
			const double t4 = std::cos(ry * 0.5);
			const double t5 = std::sin(ry * 0.5);

			Eigen::VectorDT<T, 4> quat;

			quat(0) = t0 * t2 * t4 + t1 * t3 * t5;
			quat(1) = t0 * t3 * t4 - t1 * t2 * t5;
			quat(2) = t0 * t2 * t5 + t1 * t3 * t4;
			quat(3) = t1 * t2 * t4 - t0 * t3 * t5;

			return quat.normalized();
		}

		template<typename T>
		void eulerAnglesToAxisAngle(const T rx, const T ry, const T rz, Eigen::VectorDT<T, 3> &axis, T &theta)
		{
			const Eigen::MatrixDT<T, 3> rot = eulerAnglesToRotationMatrix(rx, ry, rz);
			rotationMatrixToAxisAngle(rot, axis, theta);
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> axisAngleToRotationMatrix(const Eigen::VectorDT<T, 3> &axis, const T theta)
		{
			const Eigen::VectorDT<T, 3> n_axis = axis.normalized();
			const T s = std::sin(theta);
			const T t = 1 - std::cos(theta);
			const Eigen::MatrixDT<T, 3> crossMat = Util::Math::crossProductMatrix(n_axis);
			const Eigen::MatrixDT<T, 3> rot = Eigen::MatrixDT<T, 3>::Identity() + s * crossMat + t * crossMat * crossMat;
			return rot;
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> axisAngleToQuaternion(const Eigen::VectorDT<T, 3> &axis, const T &theta)
		{
			Eigen::VectorDT<T, 4> quat;
			const T half_theta = theta / 2;
			const T s = std::sin(half_theta);
			quat(0) = std::cos(half_theta);
			quat(1) = axis(0)*s;
			quat(2) = axis(1)*s;
			quat(3) = axis(2)*s;

			return quat;
		}

		template<typename T>
		void axisAngleToEulerAngles(const Eigen::VectorDT<T, 3> &axis, const T theta, T &rx, T &ry, T &rz)
		{
			const Eigen::MatrixDT<T, 3> rot = axisAngleToRotationMatrix(axis, theta);
			rotationMatrixToEulerAngles(rot, rx, ry, rz);
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> rotationBetweenVectors(const Eigen::VectorDT<T, 3> &vec1, const Eigen::VectorDT<T, 3> &vec2)
		{
			// Make sure the vectors are unit vectors
			const Eigen::VectorDT<T, 3> n_vec1 = vec1.normalized();
			const Eigen::VectorDT<T, 3> n_vec2 = vec2.normalized();

			// Compute the rotation matrix that rotates vec1 to vec2 using the Rodrigues formula
			const Eigen::VectorDT<T, 3> cross = n_vec1.cross(n_vec2);

			// If the cross product gives a vector with norm of 0, then the vectors are
			// already equal, so return identity
			if (cross.squaredNorm() == 0)
			{
				return Eigen::MatrixDT<T, 3>::Identity();
			}
			const T dot = n_vec1.dot(n_vec2);

			const Eigen::MatrixDT<T, 3> cross_mat = Util::Math::crossProductMatrix(cross);

			const Eigen::MatrixDT<T, 3> R = Eigen::MatrixDT<T, 3>::Identity(3, 3) + cross_mat + cross_mat * cross_mat * (( 1 - dot) / cross.squaredNorm());
			return R;
		}



		////////////////////
		// PRESETS
		////////////////////
		template<typename T>
		const Eigen::VectorDT<T, 4> identityQuaternion()
		{
			return Eigen::VectorDT<T, 4>(1, 0, 0, 0);
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> identityRotationMatrix()
		{
			return Eigen::MatrixDT<T, 3>::Identity();
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> xReflectionMatrix()
		{
			Eigen::MatrixDT<T, 3> mat = Eigen::MatrixDT<T, 3>::Identity();
			mat(0,0) = -1;
			return mat;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> yReflectionMatrix()
		{
			Eigen::MatrixDT<T, 3> mat = Eigen::MatrixDT<T, 3>::Identity();
			mat(1,1) = -1;
			return mat;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> zReflectionMatrix()
		{
			Eigen::MatrixDT<T, 3> mat = Eigen::MatrixDT<T, 3>::Identity();
			mat(2,2) = -1;
			return mat;
		}



		////////////////////
		// RANDOMS
		////////////////////

		template<typename T>
		void randomAxisAngle(Eigen::VectorDT<T, 3> &axis, T &angle)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<T> dis(0, 2*M_PI);

			axis = Eigen::VectorDT<T, 3>::Random().normalized();
			angle = dis(gen);
		}

		template<typename T>
		void randomEulerAngles(T &rx, T &ry, T &rz)
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<T> dis(0, 2*M_PI);

			rx = dis(gen);
			ry = dis(gen);
			rz = dis(gen);
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> randomQuaternion()
		{
			Eigen::VectorDT<T, 4> quat = Eigen::VectorDT<T, 4>::Random();
			quat = normalizeQuaternion(quat);
			return quat;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> randomRotationMatrix()
		{
			const Eigen::VectorDT<T, 3> axis0 = Eigen::VectorDT<T, 3>::Random().normalized();
			const Eigen::VectorDT<T, 3> tmp_axis = Eigen::VectorDT<T, 3>::Random().normalize();

			const Eigen::VectorDT<T, 3> axis1 = axis0.cross(tmp_axis);

			const Eigen::VectorDT<T, 3> axis2 = axis0.cross(axis1);

			Eigen::MatrixDT<T, 3> rotation;
			rotation << axis0, axis1, axis2;

			return rotation;
		}


		////////////////////
		// MANIPULATIONS
		////////////////////
		template<typename T>
		const Eigen::VectorDT<T, 4> normalizeQuaternion(const Eigen::VectorDT<T, 4> &quat)
		{
			const T norm = quat.norm();
			if (norm == 0)
			{
				// We do not just use (1, 0, 0, 0) because that is a constant and when used
				// for automatic differentiation that would lead to a zero derivative.
				return Eigen::VectorDT<T, 4>(1.0, quat(1), quat(2), quat(3));
			}
			else
			{
				const T inv_norm = 1.0 / norm;
				return inv_norm * quat;
			}
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> averageTwoQuaternions(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2, const T t)
		{
			const Eigen::VectorDT<T, 4> n_quat1 = normalizeQuaternion(quat1);
			Eigen::VectorDT<T, 4> n_quat2 = normalizeQuaternion(quat2);

			// Compute the cosine of the angle between the two vectors.
			float dot = n_quat1.dot(n_quat2);

			// If the quaternions are very similar, just linearly interpolate under the small angle assumption
			if (std::abs(dot) > 0.999)
			{
				Eigen::VectorDT<T, 4> interp_quat = n_quat1 + t * (n_quat2 - n_quat1);
				return normalizeQuaternion(interp_quat);
			}

			// If the dot product is negative, the quaternions adjust one quaternion so they have same handed-ness
			if (dot < 0.0f)
			{
				n_quat2 = -n_quat2;
				dot = -dot;
			}  

			if (dot > 1.0f) { dot = 1.0f; }
			else if (dot < -1.0f) { dot = -1.0f; }

			const float init_theta = std::acos(dot);  // Original angle between input quaternions
			const float theta = init_theta * t;    // Desired angle between quat1 and quat_avg

			// Find orthogonal basis of input quaternions
			Eigen::VectorDT<T, 4> quat_ortho = n_quat2 - n_quat1 * dot;
			quat_ortho.normalize();

			// Returns the quaternion representing the average
			const Eigen::VectorDT<T, 4> quat_avg = n_quat1*std::cos(theta) + quat_ortho*std::sin(theta);
			return normalizeQuaternion(quat_avg);
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> composeQuaternions(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2)
		{
			return hamiltonianProduct(quat2, quat1);
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> composeRotationMatrices(const Eigen::MatrixDT<T, 3> &rot1, const Eigen::MatrixDT<T, 3> &rot2)
		{
			return rot2 * rot1;
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> invertQuaternion(const Eigen::VectorDT<T, 4> &quat)
		{
			Eigen::VectorDT<T, 4> n_quat = normalizeQuaternion(quat);
			n_quat.template tail<3>() = -n_quat.template tail<3>();
			return n_quat;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> invertRotationMatrix(const Eigen::MatrixDT<T, 3> &rot)
		{
			return rot.transpose();
		}

		template<typename T>
		const Eigen::MatrixDT<T, 4> constructSimilarityTransform(const Eigen::MatrixDT<T, 3> &rot, const Eigen::VectorDT<T, 3> &trans, const T scale)
		{
			Eigen::MatrixDT<T, 4> similarity = Eigen::MatrixDT<T, 4>::Identity();
			similarity.col(3).template head<3>() = trans;
			similarity.topLeftCorner(3, 3) = scale * rot;
			return similarity;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 4> constructSimilarityTransform(const Eigen::MatrixDT<T, 3> &rot, const Eigen::VectorDT<T, 3> &trans,
			const T sx, const T sy, const T sz)
		{
			Eigen::MatrixDT<T, 4> similarity = Eigen::MatrixDT<T, 4>::Identity();
			Eigen::MatrixDT<T, 3> scale = Eigen::MatrixDT<T, 3>::Identity();
			scale(0,0) = sx;
			scale(1,1) = sy;
			scale(2,2) = sz;
			similarity.col(3).template head<3>() = trans;
			similarity.topLeftCorner(3, 3) = rot * scale;
			return similarity;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 4> invertSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			// Get the scale factors
			const T sx = getScaleXFromSimilarityTransform(similarity);
			const T sy = getScaleYFromSimilarityTransform(similarity);
			const T sz = getScaleZFromSimilarityTransform(similarity);
			const Eigen::VectorDT<T, 3> s_inv = Eigen::VectorDT<T, 3>(1/sx, 1/sy, 1/sz); // Inverse scales vector (1/X, 1/Y, 1/Z)

			// Get the orthnormal rotation matrix by unscaling the top-left 3x3 matrix of the similarity transform
			const Eigen::MatrixDT<T, 3> R = similarity.template block<3,3>(0,0) * s_inv.asDiagonal();

			// Get translation vector as the 4th column
			const Eigen::VectorDT<T, 3> t = similarity.template col(3).template head<3>();

			// Construct the inverse transform
			Eigen::MatrixDT<T, 4> inverse = Eigen::MatrixDT<T, 4>::Identity();
			inverse.template block<3,3>(0,0) = (s_inv.asDiagonal() * R.transpose()); // Invert the top-left 3x3 scale-rotation partition
			inverse.template block<3,1>(0,3) = (-inverse.template block<3,3>(0,0)) * t; // Invert the right 3x1 translation partition
			return inverse;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 4> composeSimilarityTransforms(const Eigen::MatrixDT<T, 4> &similarity1, const Eigen::MatrixDT<T, 4> &similarity2)
		{
			const Eigen::MatrixDT<T, 4> composed = similarity1 * similarity2;
			return composed;
		}

		template<typename T>
		const Eigen::MatrixDT<T, 4> getTransformBetweenSimilarityTransforms(const Eigen::MatrixDT<T, 4> &similarity1, const Eigen::MatrixDT<T, 4> &similarity2)
		{
			const Eigen::MatrixDT<T, 4> similarity12 = composeSimilarityTransforms(similarity2, invertSimilarityTransform(similarity1));
			return similarity12;
		}

		template<typename T>
		void composeQuaternionTransformations(const Eigen::VectorDT<T, 4> &in_quat1, const Eigen::VectorDT<T, 3> &in_trans1,
			const Eigen::VectorDT<T, 4> &in_quat2, const Eigen::VectorDT<T, 3> &in_trans2,
			Eigen::VectorDT<T, 4> &out_quat, Eigen::VectorDT<T, 3> &out_trans)
		{
			out_quat = composeQuaternions(in_quat1, in_quat2); // in_quat2 * in_quat1
			out_trans = rotatePointByQuaternion(in_trans1, in_quat2) + in_trans2;
		}

		template<typename T>
		const T getScaleFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			return similarity.template block<3, 1>(0,0).norm();
		}

		template<typename T>
		const T getScaleXFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			return similarity.template block<3, 1>(0,0).norm();
		}


		template<typename T>
		const T getScaleYFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			return similarity.template block<3, 1>(0,1).norm();
		}

		template<typename T>
		const T getScaleZFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			return similarity.template block<3, 1>(0,2).norm();
		}

		template<typename T>
		const Eigen::MatrixDT<T, 3> getRotationFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			// Get the scale factors
			const T sx = getScaleXFromSimilarityTransform(similarity);
			const T sy = getScaleYFromSimilarityTransform(similarity);
			const T sz = getScaleZFromSimilarityTransform(similarity);
			const Eigen::VectorDT<T, 3> s_inv = Eigen::VectorDT<T, 3>(1/sx, 1/sy, 1/sz); // Inverse scales vector (1/X, 1/Y, 1/Z)

			// Get the orthnormal rotation matrix by unscaling the top-left 3x3 matrix of the similarity transform
			const Eigen::MatrixDT<T, 3> R = similarity.template block<3,3>(0,0) * s_inv.asDiagonal();
			return R;			
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> getTranslationFromSimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity)
		{
			const Eigen::VectorDT<T, 3> t = similarity.template block <3, 1>(0, 3);
			return t;
		}


		template<typename T>
		void getInverseRotationAndTranslation(Eigen::VectorDT<T, 4> &quat, Eigen::VectorDT<T, 3> &trans)
		{
			// Construct the similarity transform
			Eigen::MatrixDT<T, 3> R = quaternionToRotationMatrix(quat);
			Eigen::MatrixDT<T, 4> S = constructSimilarityTransform(R, trans, static_cast<T>(1));

			// Invert it
			S = invertSimilarityTransform(S);

			// Deconstruct it again
			R = getRotationFromSimilarityTransform(S);
			quat = rotationMatrixToQuaternion(R);
			trans = getTranslationFromSimilarityTransform(S);
		}



		//////////////////////
		// TRANSFORMATIONS
		//////////////////////

		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByQuaternion(const Eigen::VectorDT<T, 3> &pt, const Eigen::VectorDT<T, 4> &quat)
		{
			ASSERT((quat.norm() - 1.0) < EPS, "Norm of the quaternion must be 1 (norm(quat) = " + std::to_string(quat.norm()) + ")");

			const Eigen::VectorDT<T, 4> inv_quat = invertQuaternion(quat);
			Eigen::VectorDT<T, 4> pt_quat;
			pt_quat.template tail<3>() = pt;
			return hamiltonianProduct(hamiltonianProduct(quat, pt_quat), inv_quat).template tail<3>();
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByRotationMatrix(const Eigen::VectorDT<T, 3> &pt, const Eigen::MatrixDT<T, 3> &rot)
		{
			ASSERT(std::abs(rot.determinant() - 1.0) < EPS, "Determinant of the rotation matrix must be 1 (det(rot) = " + std::to_string(rot.determinant()) + ")");

			return rot * pt;
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByEulerAngles(const Eigen::VectorDT<T, 3> &pt, const T rx, const T ry, const T rz)
		{
			return eulerAnglesToRotationMatrix(rx, ry, rz) * pt;
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> rotatePointByAxisAngle(const Eigen::VectorDT<T, 3> &pt, const Eigen::VectorDT<T, 3> &axis, const T theta)
		{
			ASSERT(std::abs(axis.norm() - 1.0) < EPS, "Norm of the axis of rotation must be 1 (norm(axis) = " + std::to_string(axis.norm()) + ")");

			const T c = std::cos(theta);
			const T s = std::sin(theta);
			return c*pt + s*Util::Math::crossProductMatrix(axis)*pt + (1-c)*axis.dot(pt)*axis;
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> transformPointBySimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity, const Eigen::VectorDT<T, 3> &pt)
		{
			Eigen::VectorDT<T, 4> h_pt(pt(0), pt(1), pt(2), static_cast<T>(1));
			h_pt = similarity * h_pt;
			return h_pt.template head<3>();
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> transformPointSetBySimilarityTransform(const Eigen::MatrixDT<T, 4> &similarity, const Eigen::MatrixDXT<T, 3> &pts)
		{
			Eigen::MatrixDXT<T, 4> h_pts = Eigen::MatrixDXT<T, 4>::Ones(4, pts.cols());
			h_pts.template topRows<3>() = pts;
			h_pts = similarity * h_pts;
			return h_pts.template topRows<3>();
		}

		template<typename T>
		const Eigen::VectorDT<T, 3> reflectPoint(const Eigen::VectorDT<T, 3> &pt, const size_t dim)
		{
			Eigen::VectorDT<T, 3> new_pt = pt;
			new_pt(dim) = -new_pt(dim);
			return new_pt;
		}

		template<typename T>
		const Eigen::VectorDT<T, 4> hamiltonianProduct(const Eigen::VectorDT<T, 4> &quat1, const Eigen::VectorDT<T, 4> &quat2)
		{
			Eigen::VectorDT<T, 4> product;
			product(0) = quat1(0)*quat2(0) - quat1(1)*quat2(1) - quat1(2)*quat2(2) - quat1(3)*quat2(3);
			product(1) = quat1(0)*quat2(1) + quat1(1)*quat2(0) + quat1(2)*quat2(3) - quat1(3)*quat2(2);
			product(2) = quat1(0)*quat2(2) - quat1(1)*quat2(3) + quat1(2)*quat2(0) + quat1(3)*quat2(1);
			product(3) = quat1(0)*quat2(3) + quat1(1)*quat2(2) - quat1(2)*quat2(1) + quat1(3)*quat2(0);
			return product;
		}
	}
}

#endif // UTIL_LIB_TRANSFORMS_H_