
#pragma once

#include <vector>
#include <numeric>

namespace gcransac
{
	namespace utils
	{

		struct Settings
		{
			bool do_final_iterated_least_squares, // Flag to decide a final iterated least-squares fitting is needed to polish the output model parameters.
				do_local_optimization,			  // Flag to decide if local optimization is needed
				do_graph_cut,					  // Flag to decide of graph-cut is used in the local optimization
				use_inlier_limit;				  // Flag to decide if an inlier limit is used in the local optimization to speed up the procedure

			size_t desired_fps; // The desired FPS

			size_t max_local_optimization_number,	// Maximum number of local optimizations
				min_iteration_number_before_lo,		// Minimum number of RANSAC iterations before applying local optimization
				min_iteration_number,				// Minimum number of RANSAC iterations
				max_iteration_number,				// Maximum number of RANSAC iterations
				max_unsuccessful_model_generations, // Maximum number of unsuccessful model generations
				max_least_squares_iterations,		// Maximum number of iterated least-squares iterations
				max_graph_cut_number,				// Maximum number of graph-cuts applied in each iteration
				core_number;						// Number of parallel threads

			double confidence,				// Required confidence in the result
				neighborhood_sphere_radius, // The radius of the ball used for creating the neighborhood graph
				threshold,					// The inlier-outlier threshold
				spatial_coherence_weight;	// The weight of the spatial coherence term

			Settings() : do_final_iterated_least_squares(true),
						 do_local_optimization(true),
						 do_graph_cut(true),
						 use_inlier_limit(false),
						 desired_fps(-1),
						 max_local_optimization_number(10),
						 max_graph_cut_number(10),
						 max_least_squares_iterations(10),
						 min_iteration_number_before_lo(20),
						 min_iteration_number(20),
						 neighborhood_sphere_radius(20),
						 max_iteration_number(std::numeric_limits<size_t>::max()),
						 max_unsuccessful_model_generations(100),
						 core_number(1),
						 spatial_coherence_weight(0.14),
						 threshold(2.0),
						 confidence(0.95)
			{
			}
		};
	}
}