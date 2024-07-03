

#ifndef ORB_SLAM3_DATA_LANDMARK_PLANE_H
#define ORB_SLAM3_DATA_LANDMARK_PLANE_H

#include "types.h"
#include <opencv2/core/core.hpp>

#include <Thirdparty/g2o/config.h>
#include <Thirdparty/g2o/g2o/stuff/misc.h>
#include <Thirdparty/g2o/g2o/core/eigen_types.h>

#include <mutex>
//myplan
#include "MapPoint.h"
namespace ORB_SLAM3
{
//    namespace data
//    {

    class KeyFrame;
    class Map;
    class MapPoint;
    class Atlas;

    // FW: 3D plane structure fitted from 3D sparse point cloud
    class Plane
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Constructors
        // [1] a 3D plane instance is initialized with given instance segmentation, labeled as *invalid first
        // [2] only after successfully fitting (RANSAC) from point cloud, it will be labeled as *valid
        // [3] if a plane is merged, it will be labeled as *invalid again
        Plane(KeyFrame* pRefKF, Atlas* mpAtlas);

        // Setter and Getter
        void add_landmark(MapPoint *lm);
        std::vector<MapPoint *> get_landmarks() ;
        std::unordered_map<unsigned int, MapPoint *> get_landmarks_unordered_map();

        void set_landmarks(std::vector<MapPoint *> &lms);
        unsigned int get_num_landmarks();

        void set_equation(double a, double b, double c, double d);           // set the plane parameters
        void get_equation(double &a, double &b, double &c, double &d) ; // get the plane parameters

        bool is_valid(); // indicates if a plane is valid
        void set_valid();      // a plane is valid after fitting from point cloud
        void set_invalid();    // a plane is invalid if it is merged as a candidate

        bool need_refinement();  // indicates the plane parameters need to be refined (after merge)
        void set_refinement_is_done(); // the parameters are updated (after merge)
        void set_need_refinement();    // the parameters need to be updated (after merge)

        Vec3_t get_normal() ;      // get the normal vector
        double get_normal_norm(); // the the norm of normal
        double get_offset() ;

        void set_landmarks_ownership();
        void remove_landmarks_ownership();

        void set_best_error(double const &error); // this is the threshold used to stop RANSAC loop
        double get_best_error();

        double calculate_distance(const Vec3_t &pos_w); // calculate point-plane distance
        void merge(Plane *other);                             // merge two planes

        unsigned int _id;
        static std::atomic<unsigned int> _next_id;

        // Color for visualization
        bool _has_color = false;
        double _r;
        double _g;
        double _b;

        // Plane centroid
        void setCentroid(Vec3_t &centroid);
        Vec3_t getCentroid();

        // visualization
        // Vec3_t _base_vector1{0, 0, 0};
        // Vec3_t _base_vector2{0, 0, 0};
        // void update_base_vectors();

    private:
        // The classic Hessian form: π = (n^T, d)^T
        // !Notice here, normal n is not normalized
        Vec3_t _n;                                               // n = (a, b, c)^T, not normalized
        double _d;                                               // the distance from origin
        double _abs_n;                                           // |n|
        //无符号整数和地图点关联
        std::unordered_map<unsigned int, MapPoint *> _landmarks; // associated map points
        Vec3_t _plane_centroid;                                  // centroid 3D point which is calculated by the average of all map_points' coordinates.

        //! reference keyframe
        KeyFrame* _mpRefKF;

        //! map database
//            Map* _mpMap;
        Atlas* _mpAtlas;

        // Flags
        bool _valid;
        bool _needs_refinement;

        // Statistics
        double _best_error; // best error estimated from SVD -> residual

        std::mutex _mtx_position;
        std::mutex _mtx_observations;
    };

    // namespace data

}

#endif