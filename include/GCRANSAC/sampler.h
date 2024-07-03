
#pragma once

#include <vector>

namespace gcransac
{
	namespace sampler
	{
		// Purely virtual class used for the sampling consensus methods (e.g. Ransac,
		// Prosac, MLESac, etc.)
		template <class _DataContainer, class _IndexType>
		class Sampler
		{
		protected:
			// The pointer of the container consisting of the data points from which
			// the neighborhood graph is constructed.
			const _DataContainer *const container;

			// A variable showing if the initialization was succesfull
			bool initialized;

		public:
			explicit Sampler(const _DataContainer *const container_) : container(container_),
																	   initialized(false)
			{
			}

			virtual ~Sampler() {}

			virtual const std::string getName() const = 0;

			virtual void reset() = 0;

			// Initializes any non-trivial variables and sets up sampler if
			// necessary. Must be called before sample is called.
			virtual bool initialize(const _DataContainer *const container_) = 0;

			// Samples the input variable data and fills the std::vector subset with the
			// samples.
#ifdef _MSC_EXTENSIONS
#define OLGA_INLINE __forceinline
#else
#define OLGA_INLINE inline
#endif
			OLGA_INLINE virtual bool sample(const std::vector<_IndexType> &pool_,
											_IndexType *const subset_,
											size_t sample_size_) = 0;

			bool isInitialized()
			{
				return initialized;
			}
		};
	}
}