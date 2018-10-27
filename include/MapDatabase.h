/**
* This file is part of the Collaborative Visual SLAM Extension for ORB-SLAM2:
* "Collaborative Visual SLAM using Compressed Feature Exchange"
* Copyright (C) 2017-2018 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*
* ORB-SLAM2 and the collaborative extension is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 and the collaborative extension is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with the ORB-SLAM2 collaborative extension. If not, see <http://www.gnu.org/licenses/>.
*
* The extension is built upon ORB-SLAM2:
* ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*/

#ifndef MAPDATABASE_H
#define MAPDATABASE_H


#include <mutex>
#include <map>
#include <thread>

#include "ORBVocabulary.h"
#include "Sensor.h"


namespace CORB_SLAM2
{

class System;
class Map;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;
class FrameDrawer;
class MapDrawer;
class Viewer;
class Tracking;
class MapMerging;
class Communication;
class Transmission;
class ORBextractor;
class SLAMConfig;


struct MapHolder
{
	int nAgentId;
	Map *pMap = NULL;
    LocalMapping *pLocalMapper = NULL;
    LoopClosing *pLoopCloser = NULL;
    KeyFrameDatabase *pKeyFrameDatabase = NULL;
    Tracking *pTracker = NULL;
    SLAMConfig *pConfig = NULL;

    FrameDrawer *pFrameDrawer = NULL;
    MapDrawer *pMapDrawer = NULL;
    std::thread *ptMapDrawer = NULL;
    std::thread *ptLocalMapping = NULL;
    std::thread *ptLoopClosing = NULL;
};



class MapDatabase
{
public:
	MapDatabase(System *pSystem, ORBVocabulary* pVocabulary );

    void AddMap(int nAgentId, SLAMConfig *config, Sensor sensor, bool bUseViewer = false);

    void RemoveMap(int nMapId );
    void RedirectMaps( int nSrcMapId, int nDstMapId );


    void StartMapMerging();
    void StopMapMerging();


    // Get map holder
	MapHolder *GetSingleMapHolderByMapId( int nMapId );
	std::vector<MapHolder *> GetMapHolderByMapId( int nMapId );
    MapHolder *GetMapHolderByAgentId( int nAgentId );
    std::vector<MapHolder *> GetMapHolders();


    void Shutdown();

    MapMerging *mpMapMerger;


protected:
    System *mpSystem;
    ORBVocabulary* mpVocabulary;

    // Map merging
    std::thread *mptMapMerger;

    // Hold maps
    std::mutex mMutexMap;
    std::map<int, MapHolder *> mmMaps;


};

} //namespace ORB_SLAM

#endif // MAP_H
