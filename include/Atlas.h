

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

#include "Modeler.h"

namespace ORB_SLAM3
{
class Viewer;
class Map;
class MapPoint;
class MapLine;
class KeyFrame;
class KeyFrameDatabase;
class Frame;
class KannalaBrandt8;
class Pinhole;
class Object_Map;
//MYPLAN
class Plane;

//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

class Atlas
{
    friend class boost::serialization::access;
//
//    template<class Archive>
//    void serialize(Archive &ar, const unsigned int version)
//    {
//        //ar.template register_type<Pinhole>();
//        //ar.template register_type<KannalaBrandt8>();
//
//        // Save/load the set of maps, the set is broken in libboost 1.58 for ubuntu 16.04
//        //ar & mspMaps;
//        ar & mvpBackupMaps;
//        ar & mvpCameras;
//        //ar & mvpBackupCamPin;
//        //ar & mvpBackupCamKan;
//        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
//        ar & Map::nNextId;
//        ar & Frame::nNextId;
//        ar & KeyFrame::nNextId;
//        ar & MapPoint::nNextId;
//	//ar & MapLine::nNextId;
//        ar & GeometricCamera::nNextId;
//        ar & mnLastInitKFidMap;
//    }

public:
    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
    ~Atlas();

    //myplan
    void SetModeler(Modeler *pModeler);
    Model* GetModel();
    std::vector<Object_Map*> GetObjects();
    void UpdateModel(Model* pModel);

    void CreateNewMap();
    void ChangeMap(Map* pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer* pViewer);

    // Method for change components in the current map
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    //myline 加线
    void AddMapLine(MapLine* pML);
    //void EraseMapPoint(MapPoint* pMP);
    //void EraseKeyFrame(KeyFrame* pKF);

    void AddCamera(GeometricCamera* pCam);

    /* All methods without Map pointer work on current map */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    //myline 地图册加线
    void SetReferenceMapLines(const std::vector<MapLine*> &vpMLs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    long unsigned int MapPointsInMap();
    long unsigned int MapLinesInMap();
    long unsigned KeyFramesInMap();

    // Method for get data in current map
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapLine*> GetAllMapLines();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapLine*> GetReferenceMapLines();

    //myplan
    vector<Object_Map*> mvObjectMap;    // objects in the map.
        //MYPLAN
    void add_landmark_plane(Plane *pl);
    std::vector<Plane *> get_all_landmark_planes();
    unsigned int get_num_landmark_planes();
    void erase_landmark_plane(Plane *pl);


    vector<Map*> GetAllMaps();

    int CountMaps();

    void clearMap();

    void clearAtlas();

    Map* GetCurrentMap();

    void SetMapBad(Map* pMap);
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
//    void PreSave();
    void PostLoad();

    void SetKeyFrameDababase(KeyFrameDatabase* pKFDB);
    KeyFrameDatabase* GetKeyFrameDatabase();

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    ORBVocabulary* GetORBVocabulary();

    long unsigned int GetNumLivedKF();

    long unsigned int GetNumLivedMP();

protected:

    std::set<Map*> mspMaps;
    std::set<Map*> mspBadMaps;
    // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map*> mvpBackupMaps;
    Map* mpCurrentMap;

    std::vector<GeometricCamera*> mvpCameras;
    std::vector<KannalaBrandt8*> mvpBackupCamKan;
    std::vector<Pinhole*> mvpBackupCamPin;

    //Pinhole testCam;
    std::mutex mMutexAtlas;

    unsigned long int mnLastInitKFidMap;

    Viewer* mpViewer;
    bool mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    //myplan
    Modeler* mpModeler;
    Model* mpModel;
        //myplan
    std::unordered_map<unsigned int, Plane *> _landmarks_plane;


}; // class Atlas

} // namespace ORB_SLAM3

#endif // ATLAS_H
