
//myplan 整个函数
#include "landmark_plane.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Atlas.h"
//#include <spdlog/spdlog.h>

namespace ORB_SLAM3
{
//    namespace data
//    {
    std::atomic<unsigned int> Plane::_next_id{0};

    Plane::Plane(KeyFrame * pRefKF, Atlas *mpAtlas)
            : _id(_next_id++),
              _n({1.0, 0.0, 0.0}),
              _d(-1.0),
              _abs_n(1.0),
              _plane_centroid({0.0, 0.0, 0.0}),
              _mpRefKF(pRefKF),
              _mpAtlas(mpAtlas),
              _valid(false),
              _needs_refinement(true)
    {
        _best_error = std::numeric_limits<double>::max();
    }

    void Plane::add_landmark(MapPoint *lm)
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        if (!lm->isBad())
        {
            _landmarks[lm->mnId] = lm;
            lm->set_Owning_Plane(this);
        }
    }

    std::vector<MapPoint *> Plane::get_landmarks()
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);

        std::vector<MapPoint *> landmarks;
        for (auto &id_lm_pair : _landmarks)
        {
            landmarks.push_back(id_lm_pair.second);
        }

        return landmarks;
    }

    std::unordered_map<unsigned int, MapPoint *> Plane::get_landmarks_unordered_map()
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        return _landmarks;
    }

    void Plane::set_landmarks(std::vector<MapPoint *> &lms)
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        if (!lms.empty())
        {
            _landmarks.clear();
            for (auto &lm : lms)
            {
                if (!lm->isBad())
                {
                    _landmarks[lm->mnId] = lm;
                }
            }
        }
    }

    unsigned int Plane::get_num_landmarks()
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        return _landmarks.size();
    }

    void Plane::set_equation(double a, double b, double c, double d)
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        _n = {a, b, c};
        _d = d;
        _abs_n = _n.norm();
    }

    void Plane::get_equation(double &a, double &b, double &c, double &d)
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        a = _n(0);
        b = _n(1);
        c = _n(2);
        d = _d;
    }

    double Plane::calculate_distance(const Vec3_t &pos_w)
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        double dist = (_n.dot(pos_w) + _d) / _abs_n;
        return std::fabs(dist);
    }

    bool Plane::is_valid()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        return _valid;
    }

    void Plane::set_valid()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        _valid = true;
    }

    void Plane::set_invalid()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        _valid = false;
    }

    bool Plane::need_refinement()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        return _needs_refinement;
    }

    void Plane::set_refinement_is_done()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        _needs_refinement = false;
    }

    void Plane::set_need_refinement()
    {
        std::lock_guard<std::mutex> lock1(_mtx_observations);
        std::lock_guard<std::mutex> lock2(_mtx_position);
        _needs_refinement = true;
    }

    Vec3_t Plane::get_normal()
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        return _n;
    }

    double Plane::get_normal_norm()
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        return _abs_n;
    }

    double Plane::get_offset()
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        return _d;
    }

    void Plane::set_landmarks_ownership()
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        if (!_landmarks.empty())
        {
            for (auto &id_lm_pair : _landmarks)
            {
                id_lm_pair.second->set_Owning_Plane(this);
            }
        }
    }

    void Plane::remove_landmarks_ownership()
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);
        if (!_landmarks.empty())
        {
            for (auto &id_lm_pair : _landmarks)
            {
                id_lm_pair.second->set_Owning_Plane(nullptr);
            }
        }
    }

    void Plane::set_best_error(double const &error)
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        _best_error = error;
    }

    double Plane::get_best_error()
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        return _best_error;
    }

    void Plane::merge(Plane *other)
    {
        std::lock_guard<std::mutex> lock(_mtx_observations);

        if (_id > other->_id)
        { // keep the color of the earlier plane in visualization
            _r = other->_r;
            _g = other->_g;
            _b = other->_b;
        }

        auto landmarks = other->get_landmarks_unordered_map();
        other->remove_landmarks_ownership();

        for (auto &id_lm_pair : landmarks)
        {
            if (!id_lm_pair.second->isBad())
            {
                _landmarks[id_lm_pair.first] = id_lm_pair.second;
                id_lm_pair.second->set_Owning_Plane(this);
            }
        }

        _needs_refinement = true;

        // set other plane to be invalid, and it will be removed after merge
        other->set_invalid();
    }

    void Plane::setCentroid(Vec3_t &centroid)
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        _plane_centroid = centroid;
    }

    Vec3_t Plane::getCentroid()
    {
        std::lock_guard<std::mutex> lock(_mtx_position);
        return _plane_centroid;
    }

    // void Plane::update_base_vectors()
    // {

    //     Vec3_t normalized_normal;
    //     landmark *pt_1 = nullptr;
    //     landmark *pt_2 = nullptr;

    //     {
    //         std::lock_guard<std::mutex> lock(_mtx_observations);
    //         std::lock_guard<std::mutex> lock2(_mtx_position);

    //         for (auto &id_lm_pair : _landmarks)
    //         {
    //             if (id_lm_pair.second->will_be_erased())
    //             {
    //                 continue;
    //             }

    //             double dist = std::fabs((_n.dot(id_lm_pair.second->get_pos_in_world()) + _d) / _abs_n);

    //             if (dist > _best_error)
    //             {
    //                 continue;
    //             }

    //             if (!pt_1)
    //             {
    //                 pt_1 = id_lm_pair.second;
    //                 continue;
    //             }

    //             if (!pt_2)
    //             {
    //                 pt_2 = id_lm_pair.second;
    //                 continue;
    //             }

    //             if (pt_1 && pt_2)
    //             {
    //                 break;
    //             }
    //         }

    //         normalized_normal = _n.normalized();
    //     }

    //     // simply pick two map points to formulate the base vectors of a plane patch
    //     _base_vector1 = (pt_1->get_pos_in_world() - pt_2->get_pos_in_world()).normalized();
    //     _base_vector2 = _base_vector1.cross(normalized_normal);
    // }

//    } // namespace data

}