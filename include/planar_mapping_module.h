

#ifndef ORB_SLAM3_MODULE_PLANAR_MAPPING_H
#define ORB_SLAM3_MODULE_PLANAR_MAPPING_H

//#include <yaml-cpp/yaml.h>
#include "types.h"

#include <mutex>
#include <atomic>
#include <memory>
#include "MapPoint.h"
#include "Converter.h"

namespace ORB_SLAM3
{
//    namespace data
//    {
    class Frame;
    class KeyFrame;
    class Map;

    class Plane;
    class MapPoint;
    class Converter;
    class Atlas;

//    }

    // FW: This module could be designed as the 4th thread in the SLAM system,
    //     but I find out it is not necessary (at least for now), so I merged it into mapping_module.
    class Planar_Mapping_module
    {
    public:
        Planar_Mapping_module(Atlas* pAtlas, const int sensor);
        ~Planar_Mapping_module();

        // Called in system/map initialization, works on initial keyframes
        bool initialize_map_with_plane(KeyFrame* pRefKF);

        // Those functions are used for estimating plane parameters
        bool process_new_kf(KeyFrame* pRefKF);                                                                 // Try to estimate new plane when there is a new keyframe detected
        void estimate_map_scale(KeyFrame* pRefKF);                                                             // (monocular) Calculate scale based on visible landmarks to this keyframe
        void estimate_map_scale();                                                                                   // (RGB-D) Calculate scale based on all the exiting 3D map points
        bool create_ColorToPlane(KeyFrame* pRefKF, eigen_alloc_unord_map<long, ORB_SLAM3::Plane *> &colorToPlanes); // Create map between potential plane and the map points
        bool create_new_plane(eigen_alloc_unord_map<long, ORB_SLAM3::Plane *> &colorToPlanes);                            // Create instance plane from segmentation mask and 3D points
        bool estimate_plane_sequential_RANSAC(ORB_SLAM3::Plane *plane);                                                   // Given some 3D points, estimate the best plane
        bool update_plane_via_RANSAC(ORB_SLAM3::Plane *plane);                                                            // Estimate plane paramters (e.g. after merge)

        // Graph-cut RANSAC
        bool estimate_plane_sequential_Graph_cut_RANSAC(ORB_SLAM3::Plane *plane);

        // Estimate plane based on some map_points(>=3) in RANSAC pipeline, with random indexes selected from all the map_points
        double estimate_plane_SVD(std::vector<ORB_SLAM3::MapPoint *> const &all_plane_points, std::vector<int> indexes,
                                  double &a, double &b, double &c, double &d);

        // Functions for refine existing planes
        void refinement();    // Try to refine planes when there are >= 2 planes exist in the map
        bool merge_planes();  // Merge planes which are geometrical close
        bool refine_planes(); // Update plane e.g. after merge
        bool refine_points(); // Minimize point2plane distance after merge and update
        int plSensor;
    private:
        std::mutex _mtxPlaneMutex;

        // Some thresholds (estimated on the fly)
        double _map_scale;
        double _planar_distance_thresh; // Geometric threshold for find close point to the specific plane
        double _final_error_thresh;     // Geometric threshold for find optimal plane parameters within RANSAC pipeline

//        data::map_database *_map_db = nullptr; // Used for accessing map database
//        Map* mpMap;
        Atlas* mpAtlas;
        //myplan
//        const bool _is_monocular;              // Camera mode (Mono/RGB-D), this will influence how we calculate map scale

        // ------------- Notice! all the parameters defined below are configured from an external YAML file (planar_mapping_parameters.yaml) -------------
        // ---
//        const std::string _cfg_path = "./src/planar_mapping_parameters.yaml";
        void load_configuration();

        double DOT_PRODUCT_THRESHOLD;  // Used when comparing the angle between two planes
        double OFFSET_DELTA_THRESHOLD; // Used when check two planes are close or not
        int _offset_delta_factor;

        // These two geometric thresholds have to be very strict to avoid detecting wrong plane, here we use it for both monocular and RGB-D mode
        double PLANAR_DISTANCE_CORRECTION; // Used to find close points to a certain plane, an empirical value is given as 0.02
        double FINAL_ERROR_CORRECTION;     // Used to end RANSAC estimation of a certain plane, an empirical value is given as 0.01

        // Parameters for Sequential RANSAC
        enum RansacMode
        {
            RANSAC_SMALLEST_DISTANCE_ERROR,
            RANSAC_HIGHEST_INLIER_RATIO // not used for now
        };

        RansacMode _ransacMode = RANSAC_SMALLEST_DISTANCE_ERROR;
        unsigned int _iterationsCount;                // an empirical value is given as 50
        double _inliers_ratio_thr;                    // an empirical value is given as 0.7
        unsigned int MIN_NUMBER_POINTS_BEFORE_RANSAC; // an empirical value is given as 12
        unsigned int POINTS_PER_RANSAC;               // an empirical value is given as 18

        // The semantic label of a pixel should be consistant with the label within a small window around it.
        bool _check_3x3_window; // optional

        // Print debug information in the terminal, if true
        bool _setVerbose; // optional

        // Use sequential RANSAC or Graph-cut RANSAC
        bool _use_graph_cut; // optional

        // Parameters for GC-RANSAC
        double _confidence;                    // The RANSAC confidence value
        int _fps;                              // The required FPS limit. If it is set to "-1" as un-limit
        double _inlier_outlier_threshold;      // The used adaptive inlier-outlier threshold in GC-RANSAC
        double _spatial_coherence_weight;      // The weight of the spatial coherence term in the graph-cut energy minimization (set to "0" to ignore spatial coherence).
        double _sphere_radius;                 // The radius of the sphere used for determining the neighborhood-graph
        double _minimum_inlier_ratio_for_sprt; // An assumption about the minimum inlier ratio used for the SPRT test

        // adaptive to dynamic threshold according to scale
        int adaptive_number_; // optional, if adaptive_number_ > 0
    };

} //

#endif