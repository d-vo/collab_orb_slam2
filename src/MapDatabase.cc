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

#include "MapDatabase.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "Viewer.h"
#include "MapMerging.h"
#include "MultiAgentInfo.h"

namespace CORB_SLAM2
{

MapDatabase::MapDatabase(System *pSystem, ORBVocabulary* pVocabulary )
: 	mpSystem(pSystem), mpVocabulary(pVocabulary)
{
	mpMapMerger = new MapMerging(this, mpVocabulary);
	mptMapMerger = NULL;
}



void MapDatabase::AddMap(int nAgentId, SLAMConfig *config, Sensor sensor, bool bUseViewer)
{
	std::unique_lock<std::mutex>(mMutexMap);

	if( mmMaps.count(nAgentId) > 0 )
		std::cout << "Map already present. Overwriting" << std::endl;

	if( MultiAgentInfo::gnMaxRobotId < nAgentId )
		MultiAgentInfo::gnMaxRobotId = nAgentId;


	MapHolder *pMapHolder = new MapHolder;


	// Setup config
	pMapHolder->pConfig = config;
	pMapHolder->nAgentId = nAgentId;
	pMapHolder->pMap = new Map(nAgentId);
	pMapHolder->pMapDrawer = new MapDrawer(pMapHolder->pMap, pMapHolder->pConfig , nAgentId);
	pMapHolder->pFrameDrawer = new FrameDrawer(pMapHolder->pMap);


	//Create KeyFrame Database
	pMapHolder->pKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


	// Setup mapper
	pMapHolder->pLocalMapper = new LocalMapping(pMapHolder->pMap, false, nAgentId);
	pMapHolder->pLoopCloser = new LoopClosing(pMapHolder->pMap, pMapHolder->pKeyFrameDatabase, mpVocabulary, this, true);
	pMapHolder->pLocalMapper->SetLoopCloser(pMapHolder->pLoopCloser );
	pMapHolder->pLocalMapper->SetMapMerging(mpMapMerger);
	pMapHolder->pLocalMapper->SetKeyFrameDatabase(pMapHolder->pKeyFrameDatabase);


	// Local loop closing
	pMapHolder->ptLocalMapping = new thread(&LocalMapping::Run,pMapHolder->pLocalMapper);
	pMapHolder->ptLoopClosing = new thread(&LoopClosing::Run, pMapHolder->pLoopCloser);


	// Tracking
	pMapHolder->pTracker = new Tracking(mpSystem, mpVocabulary, pMapHolder->pFrameDrawer,
			pMapHolder->pMapDrawer, pMapHolder->pMap, pMapHolder->pKeyFrameDatabase,
			pMapHolder->pConfig, sensor, nAgentId);

	pMapHolder->pTracker->SetLocalMapper(pMapHolder->pLocalMapper);
	pMapHolder->pTracker->SetLoopClosing(pMapHolder->pLoopCloser);


	// Setup visualization
	pMapHolder->pMapDrawer->SetTracker(pMapHolder->pTracker);

	if( bUseViewer )
	{
		if( pMapHolder->ptMapDrawer == NULL )
			pMapHolder->ptMapDrawer = new thread(&MapDrawer::Run, pMapHolder->pMapDrawer);
	}

	mmMaps[nAgentId] = pMapHolder;

	// Start map merging
	if( mmMaps.size() >= 2 )
		StartMapMerging();
}




void MapDatabase::RedirectMaps( int nSrcMapId, int nDstMapId )
{
	MapHolder *dstMapHolder = GetSingleMapHolderByMapId(nDstMapId);
	MapHolder *srcMapHolder = GetSingleMapHolderByMapId(nSrcMapId);

	Map *pDstMap = dstMapHolder->pMap;
	Map *pSrcMap = srcMapHolder->pMap;

	LoopClosing *pDstLoopCloser = dstMapHolder->pLoopCloser;
	KeyFrameDatabase *pDstKeyFrameDB = dstMapHolder->pKeyFrameDatabase;

	vector<KeyFrame*> vpSrcKeyFrames = pSrcMap->GetAllKeyFrames();

	// Merge Maps
	pDstMap->AddMap(pSrcMap);
	pSrcMap->empty();


	// Merge Keyframe Databases
	KeyFrameDatabase *pSrcKeyFrameDB = srcMapHolder->pKeyFrameDatabase;
	pDstKeyFrameDB->AddMap(pSrcKeyFrameDB);


	// Get all maps with same map id of src robot
	for( auto itSrcHolder : mmMaps )
	{
		MapHolder *pSrcHolder = itSrcHolder.second;
		if( pSrcHolder->pMap->GetMapId() == nSrcMapId )
		{
			// Change old tracker to new map
			Tracking *pSrcTracker = pSrcHolder->pTracker;
			pSrcTracker->ChangeMap(pDstMap, pDstLoopCloser, pDstKeyFrameDB, pSrcHolder->pMapDrawer);

			// Change src drawer to new map
			MapDrawer *pSrcMapDrawer = pSrcHolder->pMapDrawer;
			pSrcMapDrawer->ChangeMap(pDstMap);


			// Replace old pointers
			pSrcHolder->pLocalMapper->ChangeMap(pDstMap);
			pSrcHolder->pLocalMapper->SetLoopCloser(pDstLoopCloser);
			pSrcHolder->pLoopCloser->RequestFinish();


			pSrcHolder->pLoopCloser= pDstLoopCloser;
			pSrcHolder->pKeyFrameDatabase = pDstKeyFrameDB;
			pSrcHolder->pMap = pDstMap;
		}
	}


}


void MapDatabase::StartMapMerging()
{
	if( mptMapMerger != NULL )
	{
		std::cout << "Map merging already running" << std::endl;
		return;
	}

	mptMapMerger = new thread(&MapMerging::Run,mpMapMerger);
	std::cout << "Map merging started" << std::endl;
}


void MapDatabase::StopMapMerging()
{
	mpMapMerger->RequestFinish();
	while(!mpMapMerger->isFinished())
		usleep(5000);

	delete mptMapMerger;
	mptMapMerger = NULL;
}


MapHolder *MapDatabase::GetSingleMapHolderByMapId( int nMapId )
{
	std::vector<MapHolder *> vMapHolder;
	for( auto it : mmMaps )
	{
		if( it.second->pMap->GetMapId() == nMapId )
			return it.second;
	}

	return NULL;
}


std::vector<MapHolder *> MapDatabase::GetMapHolderByMapId( int nMapId )
{
	std::vector<MapHolder *> vMapHolder;
	for( auto it : mmMaps )
	{
		if( it.second->pMap->GetMapId() == nMapId )
			vMapHolder.push_back(it.second);
	}

	return vMapHolder;
}


MapHolder *MapDatabase::GetMapHolderByAgentId( int nAgentId )
{
	return mmMaps.at(nAgentId);
}


std::vector<MapHolder *> MapDatabase::GetMapHolders()
{
	std::vector<MapHolder *> tmp;
	for( auto itHolder : mmMaps )
		tmp.push_back(itHolder.second);

	return tmp;
}


void MapDatabase::Shutdown()
{
	StopMapMerging();


	for( std::map<int, MapHolder *>::iterator it = mmMaps.begin(); it != mmMaps.end(); it++)
	{
		it->second->pLocalMapper->RequestFinish();
		it->second->pLoopCloser->RequestFinish();
		it->second->pMapDrawer->RequestFinish();
	}

	for( std::map<int, MapHolder *>::iterator it = mmMaps.begin(); it != mmMaps.end(); it++)
	{
		// Wait until all thread have effectively stopped
		while(!it->second->pLocalMapper->isFinished()
				|| !it->second->pLoopCloser->isFinished()
				|| it->second->pLoopCloser->isRunningGBA())
		{
			usleep(5000);
		}
	}
}




}
