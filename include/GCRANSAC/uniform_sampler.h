

#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include "GCRANSAC/uniform_random_generator.h"
#include "GCRANSAC/sampler.h"

namespace gcransac
{
	namespace sampler
	{
		class UniformSampler : public Sampler<cv::Mat, size_t>
		{
		protected:
			std::unique_ptr<utils::UniformRandomGenerator<size_t>> random_generator;

		public:
			explicit UniformSampler(const cv::Mat *const container_)
				: Sampler(container_)
			{
				initialized = initialize(container_);
			}

			~UniformSampler() {}

			const std::string getName() const { return "Uniform Sampler"; }

			// Initializes any non-trivial variables and sets up sampler if
			// necessary. Must be called before sample is called.
			bool initialize(const cv::Mat *const container_)
			{
				random_generator = std::make_unique<utils::UniformRandomGenerator<size_t>>();
				random_generator->resetGenerator(0,
												 static_cast<size_t>(container_->rows));
				return true;
			}

			void reset()
			{
				random_generator->resetGenerator(0,
												 static_cast<size_t>(container->rows));
			}

			// Samples the input variable data and fills the std::vector subset with the
			// samples.
			OLGA_INLINE bool sample(const std::vector<size_t> &pool_,
									size_t *const subset_,
									size_t sample_size_);
		};

		OLGA_INLINE bool UniformSampler::sample(
			const std::vector<size_t> &pool_,
			size_t *const subset_,
			size_t sample_size_)
		{
			// If there are not enough points in the pool, interrupt the procedure.
			if (sample_size_ > pool_.size())
				return false;

			// Generate a unique random set of indices.
			random_generator->generateUniqueRandomSet(subset_,
													  sample_size_,
													  pool_.size() - 1);

			// Replace the temporary indices by the ones in the pool
			for (size_t i = 0; i < sample_size_; ++i)
				subset_[i] = pool_[subset_[i]];
			return true;
		}
	}
}