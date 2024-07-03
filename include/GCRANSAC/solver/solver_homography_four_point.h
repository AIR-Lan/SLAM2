
#pragma once

#include "GCRANSAC/solver/solver_engine.h"
#include "GCRANSAC/estimator/homography_estimator.h"

namespace gcransac
{
	namespace estimator
	{
		namespace solver
		{
			// This is the estimator class for estimating a homography matrix between two images. A model estimation method and error calculation method are implemented
			class HomographyFourPointSolver : public SolverEngine
			{
			public:
				HomographyFourPointSolver()
				{
				}

				~HomographyFourPointSolver()
				{
				}

				// Determines if there is a chance of returning multiple models
				// the function 'estimateModel' is applied.
				static constexpr bool returnMultipleModels()
				{
					return maximumSolutions() > 1;
				}

				static constexpr size_t maximumSolutions()
				{
					return 1;
				}

				// The minimum number of points required for the estimation
				static constexpr size_t sampleSize()
				{
					return 4;
				}

				// Estimate the model parameters from the given point sample
				// using weighted fitting if possible.
				OLGA_INLINE bool estimateModel(
					const cv::Mat &data_,					 // The set of data points
					const size_t *sample_,					 // The sample used for the estimation
					size_t sample_number_,					 // The size of the sample
					std::vector<Model> &models_,			 // The estimated model parameters
					const double *weights_ = nullptr) const; // The weight for each point
			};

			OLGA_INLINE bool HomographyFourPointSolver::estimateModel(
				const cv::Mat &data_,
				const size_t *sample_,
				size_t sample_number_,
				std::vector<Model> &models_,
				const double *weights_) const
			{
				constexpr size_t equation_number = 2;
				const size_t columns = data_.cols;
				const size_t row_number = equation_number * sample_number_;
				Eigen::MatrixXd coefficients(row_number, 8);
				Eigen::MatrixXd inhomogeneous(row_number, 1);

				const double *data_ptr = reinterpret_cast<double *>(data_.data);
				size_t row_idx = 0;
				double weight = 1.0;

				for (size_t i = 0; i < sample_number_; ++i)
				{
					const size_t idx =
						sample_ == nullptr ? i : sample_[i];

					const double *point_ptr =
						data_ptr + idx * columns;

					const double
						&x1 = point_ptr[0],
						&y1 = point_ptr[1],
						&x2 = point_ptr[2],
						&y2 = point_ptr[3];

					if (weights_ != nullptr)
						weight = weights_[idx];

					const double
						minus_weight_times_x1 = -weight * x1,
						minus_weight_times_y1 = -weight * y1,
						weight_times_x2 = weight * x2,
						weight_times_y2 = weight * y2;

					coefficients(row_idx, 0) = minus_weight_times_x1;
					coefficients(row_idx, 1) = minus_weight_times_y1;
					coefficients(row_idx, 2) = -weight;
					coefficients(row_idx, 3) = 0;
					coefficients(row_idx, 4) = 0;
					coefficients(row_idx, 5) = 0;
					coefficients(row_idx, 6) = weight_times_x2 * x1;
					coefficients(row_idx, 7) = weight_times_x2 * y1;
					inhomogeneous(row_idx) = -weight_times_x2;
					++row_idx;

					coefficients(row_idx, 0) = 0;
					coefficients(row_idx, 1) = 0;
					coefficients(row_idx, 2) = 0;
					coefficients(row_idx, 3) = minus_weight_times_x1;
					coefficients(row_idx, 4) = minus_weight_times_y1;
					coefficients(row_idx, 5) = -weight;
					coefficients(row_idx, 6) = weight_times_y2 * x1;
					coefficients(row_idx, 7) = weight_times_y2 * y1;
					inhomogeneous(row_idx) = -weight_times_y2;
					++row_idx;
				}

				Eigen::Matrix<double, 8, 1> h;

				// If we have a minimal sample, it is usually enough to solve the problem with not necessarily
				// the most accurate solver. Therefore, we use normal equations for this
				if (sample_number_ == sampleSize())
				{
					const Eigen::Matrix<double, 8, 8> coefficientsTransposed =
						coefficients.transpose();
					h = (coefficientsTransposed * coefficients).llt().solve(coefficientsTransposed * inhomogeneous);
				}
				else // Otherwise, we want the results to be very accurate.
					h = coefficients.colPivHouseholderQr().solve(inhomogeneous);

				Homography model;
				model.descriptor << h(0), h(1), h(2),
					h(3), h(4), h(5),
					h(6), h(7), 1.0;
				models_.emplace_back(model);
				return true;
			}
		}
	}
}