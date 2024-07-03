//
// Created by lzh on 10/07/23.
//

#ifndef ORB_SLAM3_MODELER_H
#define ORB_SLAM3_MODELER_H

#include <mutex>

#include <list>
#include <vector>
#include "CARV/Matrix.h"
#include "CARV/SFMTranscriptInterface_ORBSLAM.h"
#include "CARV/SFMTranscriptInterface_Delaunay.h"

class Cluster;

namespace ORB_SLAM3 {
    class KeyFrame;
    class Map;
    class MapPoint;
    class Atlas;
}

namespace ORB_SLAM3 {
    class Model {
    public:
        Model(const vector<dlovi::Matrix> &modelPoints, const list<dlovi::Matrix> &modelTris);

        vector<dlovi::Matrix> &GetPoints();

        list<dlovi::Matrix> &GetTris();

        void SetNotErase();

        void SetErase();

        void Release();

    private:
        std::mutex mMutexErase;
        bool mbNotErase;
        bool mbToBeErased;
        std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix>> mData;

    };

    class Atlas;

// interface class for surface reconstruction using CARV system
    class Modeler {
    public:
        Modeler(ORB_SLAM3::Atlas *pAtlas);

        void AddKeyFrameEntry(ORB_SLAM3::KeyFrame *pKF);

        void AddLineSegmentKeyFrameEntry(ORB_SLAM3::KeyFrame *pKF);

        bool CheckNewTranscriptEntry();

        void RunRemainder();

        void RunOnce();

        void UpdateModel();

        void WriteModel(std::string filename);

    public:
        //CARV interface
        SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
        //CARV runner instance
        dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
        SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.

        ORB_SLAM3::Map *mpMap;
        ORB_SLAM3::Atlas *mpAtlas;

        // This avoid that two transcript entries are created simultaneously in separate threads
        std::mutex mMutexTranscript;

        //number of lines in transcript last time checked
        int mnLastNumLines;

        bool mbFirstKeyFrame;

    };
}
#endif //ORB_SLAM3_MODELER_H
