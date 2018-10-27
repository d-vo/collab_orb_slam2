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


#ifndef MAPMERGING_H
#define MAPMERGING_H

#include <thread>
#include <mutex>

#include "KeyFrame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"



namespace CORB_SLAM2
{

class MapPoint;
class Map;
class KeyFrame;
class Frame;
class Tracking;
class KeyFrameDatabase;
class LocalMapping;
class LoopClosing;
class MapDrawer;
class FrameDrawer;
class Communication;
class MapDatabase;

class MapMerging
{
public:
	typedef pair<set<KeyFrame*>,int> ConsistentGroup;
	typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
			Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;


	struct MergeContext
	{
	    std::vector<ConsistentGroup> mvConsistentGroups;
	    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
	};


public:
	MapMerging(MapDatabase *pMapDatabase, ORBVocabulary* pORBVocabulary);

	// Main loop
	void Run();

	void InsertKeyFrame(KeyFrame* pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF, const int dstMapId);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    void RequestFinish();

    bool isFinished();

    // Thread Synch
    void RequestStop();
    bool Stop();
    void Release();
    bool isStopped();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	bool DetectOverlap();

	bool ComputeSim3();

	void SearchAndFuse(const int nMapId, const KeyFrameAndPose &CorrectedPosesMap);

	void MergeMaps();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

	MapDatabase *mpMapDatabase;
	ORBVocabulary* mpORBVocabulary;

	std::list<KeyFrame*> mlpLoopKeyFrameQueue;

	std::mutex mMutexStop;
    std::mutex mMutexLoopQueue;


    bool mbStopped;
    bool mbStopRequested;


    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;

    std::map<int, MergeContext> mvContext;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    int mnCurrentMapCandidate;


    cv::Mat mScw;
    g2o::Sim3 mg2oScw;


    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;
};

} //namespace ORB_SLAM

#endif
