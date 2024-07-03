

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
//myplan
#include "landmark_plane.h"
namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Chage to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

//myplan
void Atlas::SetModeler(Modeler *pModeler)
{
    mpModeler=pModeler;
}
Model* Atlas::GetModel()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpModel;
}

// BRIEF [EAO-SLAM]
//myplan 新增函数
vector<Object_Map*> Atlas::GetObjects()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return vector<Object_Map*>(mvObjectMap.begin(),mvObjectMap.end());
}

void Atlas::UpdateModel(Model* pModel)
{
    Model* pModelPrev;
    {
        unique_lock<mutex> lock(mMutexAtlas);
        pModelPrev = mpModel;
        mpModel = pModel;
    }
    // // TODO bug?
    // if(pModelPrev != NULL)
    //     pModelPrev->Release();
}


void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddMapLine(MapLine* pML)
{
    Map* pMapML = pML->GetMap();
    pMapML->AddMapLine(pML);
} 

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::SetReferenceMapLines(const std::vector<MapLine*> &vpMLs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapLines(vpMLs);
} 

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned int Atlas::MapLinesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapLinesInMap();
} 

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapLine*> Atlas::GetAllMapLines()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapLines();
} 

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}
//myline 获取参考地图线
std::vector<MapLine*> Atlas::GetReferenceMapLines()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapLines();
} 

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

//void Atlas::PreSave()
//{
//    if(mpCurrentMap){
//        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
//            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
//    }
//
//    struct compFunctor
//    {
//        inline bool operator()(Map* elem1 ,Map* elem2)
//        {
//            return elem1->GetId() < elem2->GetId();
//        }
//    };
//    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
//    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());
//
//    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
//    cout << "There are " << spCams.size() << " cameras in the atlas" << endl;
//    for(Map* pMi : mvpBackupMaps)
//    {
//        cout << "Pre-save of map " << pMi->GetId() << endl;
//        pMi->PreSave(spCams);
//    }
//    cout << "Maps stored" << endl;
//    for(GeometricCamera* pCam : mvpCameras)
//    {
//        cout << "Pre-save of camera " << pCam->GetId() << endl;
//        if(pCam->GetType() == pCam->CAM_PINHOLE)
//        {
//            mvpBackupCamPin.push_back((Pinhole*) pCam);
//        }
//        else if(pCam->GetType() == pCam->CAM_FISHEYE)
//        {
//            mvpBackupCamKan.push_back((KannalaBrandt8*) pCam);
//        }
//    }
//
//}

//void Atlas::PostLoad()
//{
//    mvpCameras.clear();
//    map<unsigned int,GeometricCamera*> mpCams;
//    for(Pinhole* pCam : mvpBackupCamPin)
//    {
//        //mvpCameras.push_back((GeometricCamera*)pCam);
//        mvpCameras.push_back(pCam);
//        mpCams[pCam->GetId()] = pCam;
//    }
//    for(KannalaBrandt8* pCam : mvpBackupCamKan)
//    {
//        //mvpCameras.push_back((GeometricCamera*)pCam);
//        mvpCameras.push_back(pCam);
//        mpCams[pCam->GetId()] = pCam;
//    }
//
//    mspMaps.clear();
//    unsigned long int numKF = 0, numMP = 0;
//    map<long unsigned int, KeyFrame*> mpAllKeyFrameId;
//    for(Map* pMi : mvpBackupMaps)
//    {
//        cout << "Map id:" << pMi->GetId() << endl;
//        mspMaps.insert(pMi);
//        map<long unsigned int, KeyFrame*> mpKeyFrameId;
//        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpKeyFrameId, mpCams);
//        mpAllKeyFrameId.insert(mpKeyFrameId.begin(), mpKeyFrameId.end());
//        numKF += pMi->GetAllKeyFrames().size();
//        numMP += pMi->GetAllMapPoints().size();
//    }
//
//    cout << "Number KF:" << numKF << "; number MP:" << numMP << endl;
//    mvpBackupMaps.clear();
//}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

//myplan 往地图里添加面
    void Atlas::add_landmark_plane(Plane *pl)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        _landmarks_plane[pl->_id] = std::move(pl);
    }

// FW:// myplan 往地图里添加面
    std::vector<Plane *> Atlas::get_all_landmark_planes()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        std::vector<Plane *> all_planes;
        if (!_landmarks_plane.empty())
        {
            all_planes.reserve(_landmarks_plane.size());
            for (auto id_plane : _landmarks_plane)
            {
                all_planes.push_back(id_plane.second);
            }
        }
        return all_planes;
    }

    void Atlas::erase_landmark_plane(Plane *pl)
    {
        unique_lock<mutex> lock(mMutexAtlas);
        _landmarks_plane.erase(pl->_id);
    }
    unsigned int Atlas::get_num_landmark_planes()
    {
        unique_lock<mutex> lock(mMutexAtlas);
        return _landmarks_plane.size();
    }//myplan end


} //namespace ORB_SLAM3
