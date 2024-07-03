
#pragma once

#include <opencv2/core/core.hpp>
#include <vector>
#include <Eigen/Eigen>
#include "GCRANSAC/estimator/estimator.h"

namespace gcransac
{
	class Model
	{
	public:
		Eigen::MatrixXd descriptor; // The descriptor of the current model

		Model(const Eigen::MatrixXd &descriptor_) : descriptor(descriptor_)
		{
		}

		Model()
		{
		}
	};

	class RigidTransformation : public Model
	{
	public:
		RigidTransformation() : Model(Eigen::MatrixXd(4, 4))
		{
		}
		RigidTransformation(const RigidTransformation &other)
		{
			descriptor = other.descriptor;
		}
	};

	class Line2D : public Model
	{
	public:
		Line2D() : Model(Eigen::MatrixXd(3, 1))
		{
		}
		Line2D(const Line2D &other)
		{
			descriptor = other.descriptor;
		}
	};

	// FW:
	class Plane3D : public Model
	{
	public:
		Plane3D() : Model(Eigen::MatrixXd(4, 1))
		{
		}
		Plane3D(const Plane3D &other)
		{
			descriptor = other.descriptor;
		}
	};

	class FundamentalMatrix : public Model
	{
	public:
		FundamentalMatrix() : Model(Eigen::MatrixXd(3, 3))
		{
		}
		FundamentalMatrix(const FundamentalMatrix &other)
		{
			descriptor = other.descriptor;
		}
	};

	class EssentialMatrix : public Model
	{
	public:
		EssentialMatrix() : Model(Eigen::MatrixXd(3, 3))
		{
		}
		EssentialMatrix(const EssentialMatrix &other)
		{
			descriptor = other.descriptor;
		}
	};

	class Pose6D : public Model
	{
	public:
		Pose6D() : Model(Eigen::MatrixXd(3, 4))
		{
		}
		Pose6D(const Pose6D &other_)
		{
			descriptor = other_.descriptor;
		}
	};

	class Homography : public Model
	{
	public:
		Homography() : Model(Eigen::MatrixXd(3, 3))
		{
		}

		Homography(const Homography &other)
		{
			descriptor = other.descriptor;
		}
	};
}