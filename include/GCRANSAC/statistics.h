
#pragma once

#include <vector>
#include <numeric>

namespace gcransac
{
	namespace utils
	{
		struct RANSACStatistics
		{
			size_t graph_cut_number,
				local_optimization_number,
				iteration_number,
				neighbor_number,
				accepted_models,
				rejected_models;

			std::string main_sampler_name,
				local_optimizer_sampler_name;

			double processing_time;

			std::vector<size_t> inliers;

			RANSACStatistics() : graph_cut_number(0),
								 local_optimization_number(0),
								 iteration_number(0),
								 neighbor_number(0),
								 accepted_models(0),
								 rejected_models(0),
								 processing_time(0.0)
			{
			}
		};
	}
}