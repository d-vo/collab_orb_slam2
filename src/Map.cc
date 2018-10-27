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

#include "Map.h"

#include<mutex>

namespace CORB_SLAM2
{

Map::Map(int nMapId)
: mnMapId(nMapId), mnMaxKFid(0),mnBigChangeIdx(0)
{
	mnMaxRobotId = nMapId;
	msnRobotIds.insert(nMapId);
}


void Map::AddMap(Map *pMap)
{
	vector<KeyFrame*> vSrcKeyFrames = pMap->GetAllKeyFrames();
	vector<MapPoint*> vSrcMapPoints = pMap->GetAllMapPoints();


	for( KeyFrame *pKF : vSrcKeyFrames )
	{
		pKF->UpdateMap(this);
		AddKeyFrame(pKF);
	}

	for( MapPoint *pMapPoint : vSrcMapPoints )
	{
		pMapPoint->UpdateMap(this);
		AddMapPoint(pMapPoint);
	}

	msnRobotIds.insert(pMap->GetMapId());

	if( pMap->GetMapId() > mnMaxRobotId )
		mnMaxRobotId = pMap->GetMapId();
}

int Map::GetMapId()
{
	return mnMapId;
}



void Map::AddKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexMap);
	if( mspKeyFrames.size() == 0)
		pKF->SetFirst();

	mspKeyFrames.insert(pKF);
	if(pKF->mnId>mnMaxKFid)
		mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
	unique_lock<mutex> lock(mMutexMap);
	mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
	unique_lock<mutex> lock(mMutexMap);
	mspMapPoints.erase(pMP);

	// TODO: This only erase the pointer.
	// Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexMap);
	mspKeyFrames.erase(pKF);

	// TODO: This only erase the pointer.
	// Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
	unique_lock<mutex> lock(mMutexMap);
	mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
	unique_lock<mutex> lock(mMutexMap);
	mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
	unique_lock<mutex> lock(mMutexMap);
	return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
	unique_lock<mutex> lock(mMutexMap);
	return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
	unique_lock<mutex> lock(mMutexMap);
	return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
	unique_lock<mutex> lock(mMutexMap);
	return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
	unique_lock<mutex> lock(mMutexMap);
	return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
	unique_lock<mutex> lock(mMutexMap);
	return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
	unique_lock<mutex> lock(mMutexMap);
	return mnMaxKFid;
}


void Map::clear()
{
	for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
		delete *sit;

	for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
		delete *sit;

	mspMapPoints.clear();
	mspKeyFrames.clear();
	mnMaxKFid = 0;
	mvpReferenceMapPoints.clear();
	mvpKeyFrameOrigins.clear();
}


void Map::empty()
{
	mspMapPoints.clear();
	mspKeyFrames.clear();
	mnMaxKFid = 0;
	mvpReferenceMapPoints.clear();
	mvpKeyFrameOrigins.clear();
}

int Map::GetMaxRobotId()
{
	return mnMaxRobotId;
}



void Map::SanityCheck()
{
#ifdef DEBUG
	vector<KeyFrame*> vpKeyFrames = GetAllKeyFrames();
	vector<MapPoint*> vpMapPoints = GetAllMapPoints();


	if( vpKeyFrames.size() < 1 )
		return;

	std::set<long unsigned int> sKfIds;
	for( KeyFrame *pKF : vpKeyFrames )
	{
		if( pKF->isBad() )
			continue;

		std::set<long unsigned int> sMapPointIds;
		vector<MapPoint*> vpMapPointMatches = pKF->GetMapPointMatches();
		for( int i = 0; i < (int) vpMapPointMatches.size(); i++)
		{
			MapPoint* pMP = vpMapPointMatches[i];
			if( pMP )
			{
				if( pMP->isBad() )
					continue;

				int index = pMP->GetIndexInKeyFrame(pKF);

				if( index != i )
					std::cout << "Index: " << index << " " << i << std::endl;

				assert( index == i );

				// No duplicate matches
				assert(sMapPointIds.count(pMP->mnId) == 0);
				sMapPointIds.insert(pMP->mnId);
			}
		}


		// No duplicate KF
		assert(sKfIds.count(pKF->mnId) == 0);
		sKfIds.insert(pKF->mnId);
	}

	std::set<long unsigned int> sMapPointIds;
	for( MapPoint *pMP : vpMapPoints )
	{
		if( pMP->isBad() )
			continue;

		map<KeyFrame*, size_t> vObservations = pMP->GetObservations();
		for( auto obs : vObservations )
		{
			KeyFrame* pKF = obs.first;
			MapPoint *pCurrentMP = pKF->GetMapPoint(obs.second);

			if( pKF->isBad() || pMP->isBad() || pCurrentMP->isBad() )
				continue;

			assert(pCurrentMP == pMP);
		}


		assert( vObservations.size() > 0);

		//		// Check if reference keyframe is here
		KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
		assert(pRefKF);


		// No duplicate KF
		assert(sMapPointIds.count(pMP->mnId) == 0);
		sMapPointIds.insert(pMP->mnId);
	}

	if( vpKeyFrames.empty() || vpMapPoints.empty() )
		return;


	// Check essential graph
	KeyFrame *pOriginKF = *(mvpKeyFrameOrigins.begin());
	list<KeyFrame*> lpKFtoCheck;
	lpKFtoCheck.push_back(pOriginKF);


	bool bOriginFound = false;
	std::set<long unsigned int> sKfGrtaphIds;
	while(!lpKFtoCheck.empty())
	{
		KeyFrame* pKF = lpKFtoCheck.front();

		const set<KeyFrame*> sChilds = pKF->GetChilds();
		for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
		{
			KeyFrame* pChild = *sit;
			lpKFtoCheck.push_back(pChild);
		}


		if( (pKF->GetParent() == NULL) )
		{
			assert(!bOriginFound);
			bOriginFound = true;
		}

		// No duplicate KF
		assert(sKfGrtaphIds.count(pKF->mnId) == 0);
		sKfGrtaphIds.insert(pKF->mnId);

		lpKFtoCheck.pop_front();
	}

	assert(sKfGrtaphIds.size() == sKfIds.size());
	assert( bOriginFound );

	if( !bOriginFound )
		std::cout << "Origin not found" << std::endl;

#endif
}




} //namespace ORB_SLAM
