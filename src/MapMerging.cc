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

#include "MapMerging.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"


#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "MapDrawer.h"

#include <mutex>
#include <thread>

using namespace std;

namespace CORB_SLAM2
{

MapMerging::MapMerging(MapDatabase *pMapDatabase, ORBVocabulary* pORBVocabulary)
: mbFinishRequested(false), mbFinished(true), mpMapDatabase(pMapDatabase),
  mpORBVocabulary(pORBVocabulary), mpMatchedKF(NULL), mbRunningGBA(false), mpThreadGBA(NULL), mbFixScale(true)
{
	mnCovisibilityConsistencyTh = 3;
	mbStopped = false;
	mbStopRequested = false;
}


void MapMerging::Run()
{
	mbFinished = false;
	while(1)
	{
		// Detect loop candidates and check covisibility consistency
		if(DetectOverlap())
		{
			// Compute similarity transformation [sR|t]
			// In the stereo/RGBD case s=1
			if(ComputeSim3())
			{
				// Perform loop fusion and pose graph optimization
				MergeMaps();
			}
		}
		else if(Stop())
		{
			// Safe area to stop
			while(isStopped() )
			{
				usleep(3000);
				if(CheckFinish())
					break;
			}
		}


		if(CheckFinish())
			break;

		ResetIfRequested();

		usleep(3000);

		// If a Global Bundle Adjustment is running, wait
		while(isRunningGBA())
		{
			usleep(5000);
		}
	}

	mbFinished = true;
}

void MapMerging::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	unique_lock<mutex> lock2(mMutexLoopQueue);

	mbStopRequested = true;
	std::cout << "Loop Closing Request Stop" << std::endl;
}

bool MapMerging::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if(mbStopRequested )
	{
		mbStopped = true;
		return true;
	}

	return false;
}

bool MapMerging::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void MapMerging::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	unique_lock<mutex> lock2(mMutexFinish);
	if(mbFinished)
		return;
	mbStopped = false;
	mbStopRequested = false;
}



void MapMerging::InsertKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexLoopQueue);
	if(!pKF->IsFirst() && ((pKF->mnId % MAP_IDX_OFFSET) > 5) )
		mlpLoopKeyFrameQueue.push_back(pKF);
}



bool MapMerging::DetectOverlap()
{
	{
		unique_lock<mutex> lock(mMutexLoopQueue);

		// Priority
		if( !mlpLoopKeyFrameQueue.empty() )
		{
			mpCurrentKF = mlpLoopKeyFrameQueue.front();
			mlpLoopKeyFrameQueue.pop_front();
		}
		else
			return false;

		// Avoid that a keyframe can be erased while it is being process by this thread
		mpCurrentKF->SetNotErase();
	}



	// Compute reference BoW similarity score
	// This is the lowest score to a connected keyframe in the covisibility graph
	// We will impose loop candidates to have a higher similarity than this
	const int nKfMapId = mpCurrentKF->GetMapId();


	const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
	const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
	float minScore = 1;
	for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
	{
		KeyFrame* pKF = vpConnectedKeyFrames[i];

		if(pKF->isBad())
			continue;

		const DBoW2::BowVector &BowVec = pKF->mBowVec;
		float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

		if(score<minScore)
			minScore = score;
	}


	std::vector<MapHolder *> vMapHolders = mpMapDatabase->GetMapHolders();
	for( int i = 0; i < (int) vMapHolders.size(); i++)
	{
		int nMapId = vMapHolders[i]->pMap->GetMapId();

		// Do not search in own database
		if( nMapId == nKfMapId)
			continue;

		// Query the database imposing the minimum score
		KeyFrameDatabase *mpKeyFrameDB = vMapHolders[i]->pKeyFrameDatabase;
		vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

		// If there are no loop candidates, just add new keyframe and return false
		if(vpCandidateKFs.empty())
		{
			mvContext[nMapId].mvConsistentGroups.clear();
			continue;
		}

		// For each loop candidate check consistency with previous loop candidates
		// Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
		// A group is consistent with a previous group if they share at least a keyframe
		// We must detect a consistent loop in several consecutive keyframes to accept it
		mvContext[nMapId].mvpEnoughConsistentCandidates.clear();

		vector<ConsistentGroup> vCurrentConsistentGroups;
		vector<bool> vbConsistentGroup(mvContext[nMapId].mvConsistentGroups.size(),false);
		for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
		{
			KeyFrame* pCandidateKF = vpCandidateKFs[i];

			set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
			spCandidateGroup.insert(pCandidateKF);

			bool bEnoughConsistent = false;
			bool bConsistentForSomeGroup = false;
			for(size_t iG=0, iendG= mvContext[nMapId].mvConsistentGroups.size(); iG<iendG; iG++)
			{
				set<KeyFrame*> sPreviousGroup = mvContext[nMapId].mvConsistentGroups[iG].first;

				bool bConsistent = false;
				for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
				{
					if(sPreviousGroup.count(*sit))
					{
						bConsistent=true;
						bConsistentForSomeGroup=true;
						break;
					}
				}

				if(bConsistent)
				{
					int nPreviousConsistency = mvContext[nMapId].mvConsistentGroups[iG].second;
					int nCurrentConsistency = nPreviousConsistency + 1;
					if(!vbConsistentGroup[iG])
					{
						ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
						vCurrentConsistentGroups.push_back(cg);
						vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
					}
					if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
					{
						mvContext[nMapId].mvpEnoughConsistentCandidates.push_back(pCandidateKF);
						bEnoughConsistent=true; //this avoid to insert the same candidate more than once
					}
				}
			}

			// If the group is not consistent with any previous group insert with consistency counter set to zero
			if(!bConsistentForSomeGroup)
			{
				ConsistentGroup cg = make_pair(spCandidateGroup,0);
				vCurrentConsistentGroups.push_back(cg);
			}
		}

		// Update Covisibility Consistent Groups
		mvContext[nMapId].mvConsistentGroups = vCurrentConsistentGroups;


		if( !mvContext[nMapId].mvpEnoughConsistentCandidates.empty())
		{
			mnCurrentMapCandidate = nMapId;
			return true;
		}

	}

	mpCurrentKF->SetErase();

	return false;
}


bool MapMerging::ComputeSim3()
{
	// For each consistent loop candidate we try to compute a Sim3
	const int nInitialCandidates = mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates.size();

	// We compute first ORB matches for each candidate
	// If enough matches are found, we setup a Sim3Solver
	ORBmatcher matcher(0.75,true);

	vector<Sim3Solver*> vpSim3Solvers;
	vpSim3Solvers.resize(nInitialCandidates);

	vector<vector<MapPoint*> > vvpMapPointMatches;
	vvpMapPointMatches.resize(nInitialCandidates);

	vector<bool> vbDiscarded;
	vbDiscarded.resize(nInitialCandidates);

	int nCandidates=0; //candidates with enough matches

	for(int i=0; i<nInitialCandidates; i++)
	{
		KeyFrame* pKF = mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i];

		// avoid that local mapping erase it while it is being processed in this thread
		pKF->SetNotErase();

		if(pKF->isBad())
		{
			vbDiscarded[i] = true;
			continue;
		}

		int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

		if(nmatches<15)
		{
			vbDiscarded[i] = true;
			continue;
		}
		else
		{
			Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
			pSolver->SetRansacParameters(0.95,10,500);
			vpSim3Solvers[i] = pSolver;
		}

		nCandidates++;
	}

	bool bMatch = false;

	// Perform alternatively RANSAC iterations for each candidate
	// until one is succesful or all fail
	while(nCandidates>0 && !bMatch)
	{
		for(int i=0; i<nInitialCandidates; i++)
		{
			if(vbDiscarded[i])
				continue;

			KeyFrame* pKF = mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i];

			// Perform 5 Ransac Iterations
			vector<bool> vbInliers;
			int nInliers;
			bool bNoMore;

			Sim3Solver* pSolver = vpSim3Solvers[i];
			cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

			// If Ransac reachs max. iterations discard keyframe
			if(bNoMore)
			{
				vbDiscarded[i]=true;
				nCandidates--;
			}

			// If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
			if(!Scm.empty())
			{
				vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
				for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
				{
					if(vbInliers[j])
						vpMapPointMatches[j]=vvpMapPointMatches[i][j];
				}

				cv::Mat R = pSolver->GetEstimatedRotation();
				cv::Mat t = pSolver->GetEstimatedTranslation();
				const float s = pSolver->GetEstimatedScale();
				matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

				g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
				const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

				// If optimization is succesful stop ransacs and continue
				if(nInliers>=20)
				{
					bMatch = true;
					mpMatchedKF = pKF;
					g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
					mg2oScw = gScm*gSmw;
					mScw = Converter::toCvMat(mg2oScw);

					mvpCurrentMatchedPoints = vpMapPointMatches;
					break;
				}
			}
		}
	}

	if(!bMatch)
	{
		for(int i=0; i<nInitialCandidates; i++)
			mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i]->SetErase();
		mpCurrentKF->SetErase();
		return false;
	}

	// Retrieve MapPoints seen in Loop Keyframe and neighbors
	vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
	vpLoopConnectedKFs.push_back(mpMatchedKF);
	mvpLoopMapPoints.clear();
	for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
	{
		KeyFrame* pKF = *vit;
		vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
		for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
		{
			MapPoint* pMP = vpMapPoints[i];
			if(pMP)
			{
				if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
				{
					mvpLoopMapPoints.push_back(pMP);
					pMP->mnLoopPointForKF=mpCurrentKF->mnId;
				}
			}
		}
	}

	// Find more matches projecting with the computed Sim3
	matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints,
			mvpCurrentMatchedPoints,10);

	// If enough matches accept Loop
	int nTotalMatches = 0;
	for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
	{
		if(mvpCurrentMatchedPoints[i])
			nTotalMatches++;
	}

	if(nTotalMatches>=30)
	{
		for(int i=0; i<nInitialCandidates; i++)
			if(mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
				mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i]->SetErase();
		return true;
	}
	else
	{
		for(int i=0; i<nInitialCandidates; i++)
			mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[i]->SetErase();
		mpCurrentKF->SetErase();
		return false;
	}
}



void MapMerging::MergeMaps()
{
	// Copy all Keyframes and Map Points into single Map
	const int srcMapId = mpCurrentKF->GetMapId();
	const int dstMapId = mvContext[mnCurrentMapCandidate].mvpEnoughConsistentCandidates[0]->GetMapId();

	cout << "Overlap detected: Merging Map " << srcMapId << " into Map " << dstMapId << endl;


	std::vector<MapHolder *> vpSrcMapHoldder = mpMapDatabase->GetMapHolderByMapId(srcMapId);
	std::vector<MapHolder *> vpDstMapHoldder = mpMapDatabase->GetMapHolderByMapId(dstMapId);


	Map *pSrcMap = mpMapDatabase->GetSingleMapHolderByMapId(srcMapId)->pMap;
	Map *pDstMap =  mpMapDatabase->GetSingleMapHolderByMapId(dstMapId)->pMap;


	// Stop everything
	for( MapHolder *pSrcMapHoldder : vpSrcMapHoldder )
	{
		pSrcMapHoldder->pMapDrawer->RequestStop();
		pSrcMapHoldder->pLocalMapper->RequestStop();
		pSrcMapHoldder->pLoopCloser->RequestFinish();
		pSrcMapHoldder->pTracker->Pause();
		while( !pSrcMapHoldder->pLocalMapper->isStopped()
				|| !pSrcMapHoldder->pLoopCloser->isFinished()
				|| !pSrcMapHoldder->pMapDrawer->isStopped())
		{
			usleep(1000);
		}
	}


	for( MapHolder *pDstMapHoldder : vpDstMapHoldder )
	{
		pDstMapHoldder->pMapDrawer->RequestStop();
		pDstMapHoldder->pLocalMapper->RequestStop();
		pDstMapHoldder->pLoopCloser->RequestStop();
		pDstMapHoldder->pTracker->Pause();
		while( !pDstMapHoldder->pLocalMapper->isStopped()
				|| !pDstMapHoldder->pLoopCloser->isStopped()
				|| !pDstMapHoldder->pMapDrawer->isStopped())
		{
			usleep(1000);
		}
	}



	// Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
	mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
	mvpCurrentConnectedKFs.push_back(mpCurrentKF);

	KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
	CorrectedSim3[mpCurrentKF]=mg2oScw;
	cv::Mat Twc = mpCurrentKF->GetPoseInverse();

	// Ensure current keyframe is updated
	mpCurrentKF->UpdateConnections();


	{
		mpMapDatabase->RedirectMaps(srcMapId, dstMapId);

		for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(),
				vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
		{
			KeyFrame* pKFi = *vit;

			cv::Mat Tiw = pKFi->GetPose();

			if(pKFi!=mpCurrentKF)
			{
				cv::Mat Tic = Tiw*Twc;
				cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
				cv::Mat tic = Tic.rowRange(0,3).col(3);
				g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
				g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
				//Pose corrected with the Sim3 of the loop closure
				CorrectedSim3[pKFi]=g2oCorrectedSiw;
			}

			cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
			cv::Mat tiw = Tiw.rowRange(0,3).col(3);
			g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
			//Pose without correction
			NonCorrectedSim3[pKFi]=g2oSiw;
		}

		// Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
		for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
		{
			KeyFrame* pKFi = mit->first;
			g2o::Sim3 g2oCorrectedSiw = mit->second;
			g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

			g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

			vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
			for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
			{
				MapPoint* pMPi = vpMPsi[iMP];
				if(!pMPi)
					continue;
				if(pMPi->isBad())
					continue;
				if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
					continue;

				// Project with non-corrected pose and project back with corrected pose
				cv::Mat P3Dw = pMPi->GetWorldPos();
				Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
				Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

				cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
				pMPi->SetWorldPos(cvCorrectedP3Dw);
				pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
				pMPi->mnCorrectedReference = pKFi->mnId;
				pMPi->UpdateNormalAndDepth();
			}

			// Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
			Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
			Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
			double s = g2oCorrectedSiw.scale();

			eigt *=(1./s); //[R t/s;0 1]

			cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

			pKFi->SetPose(correctedTiw);

			// Make sure connections are updated
			pKFi->UpdateConnections();
		}

		// Start map merging
		// Update matched map points and replace if duplicated
		{
			boost::unique_lock<boost::shared_mutex> lockKF(mpCurrentKF->mFeaturesSharedMutex);
			for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
			{
				if(mvpCurrentMatchedPoints[i])
				{
					MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
					MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
					if(pCurMP)
						pCurMP->Replace(pLoopMP);
					else
					{
						pLoopMP->AddObservation(mpCurrentKF,i);
						mpCurrentKF->AddMapPoint(pLoopMP,i);
						pLoopMP->ComputeDistinctiveDescriptors();
					}
				}
			}
		}


		// Project MapPoints observed in the neighborhood of the loop keyframe
		// into the current keyframe and neighbors using corrected poses.
		// Fuse duplications.
		SearchAndFuse(dstMapId, CorrectedSim3);


		// After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides
		map<KeyFrame*, set<KeyFrame*> > LoopConnections;

		for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
		{
			KeyFrame* pKFi = *vit;
			vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

			// Update connections. Detect new links.
			pKFi->UpdateConnections();
			LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
			for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
			{
				LoopConnections[pKFi].erase(*vit_prev);
			}
			for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
			{
				LoopConnections[pKFi].erase(*vit2);
			}
		}



		// Optimize graph
		Optimizer::OptimizeEssentialGraph(pDstMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);
	}






	// Attach the two essential graphs
	KeyFrame *srcOrigin = *(pSrcMap->mvpKeyFrameOrigins.begin());
	KeyFrame *dstOrigin = *(pDstMap->mvpKeyFrameOrigins.begin());
	srcOrigin->AddParent(dstOrigin);


	// Restart everything
	for( MapHolder *pSrcMapHoldder : vpSrcMapHoldder )
	{
		pSrcMapHoldder->pLocalMapper->Release();
		pSrcMapHoldder->pTracker->Resume();
		pSrcMapHoldder->pMapDrawer->Release();
	}


	for( MapHolder *pDstMapHoldder : vpDstMapHoldder )
	{
		pDstMapHoldder->pLocalMapper->Release();
		pDstMapHoldder->pTracker->Resume();
		pDstMapHoldder->pMapDrawer->Release();
	}


	// Launch a new thread to perform Global Bundle Adjustment
	mbRunningGBA = true;
	mpThreadGBA = new thread(&MapMerging::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId, dstMapId);


	std::cout << "Merge complete" << std::endl;
}


void MapMerging::SearchAndFuse(const int nMapId, const KeyFrameAndPose &CorrectedPosesMap)
{
	ORBmatcher matcher(0.8);
	for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
	{
		KeyFrame* pKF = mit->first;

		g2o::Sim3 g2oScw = mit->second;
		cv::Mat cvScw = Converter::toCvMat(g2oScw);

		vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
		matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

		const int nLP = mvpLoopMapPoints.size();
		for(int i=0; i<nLP;i++)
		{
			MapPoint* pRep = vpReplacePoints[i];
			if(pRep)
			{
				pRep->Replace(mvpLoopMapPoints[i]);
			}
		}
	}
}


void MapMerging::RequestReset()
{
	{
		unique_lock<mutex> lock(mMutexReset);
		mbResetRequested = true;
	}

	while(1)
	{
		{
			unique_lock<mutex> lock2(mMutexReset);
			if(!mbResetRequested)
				break;
		}
		usleep(5000);
	}
}

void MapMerging::ResetIfRequested()
{
	unique_lock<mutex> lock(mMutexReset);
	if(mbResetRequested)
	{
		mlpLoopKeyFrameQueue.clear();
		mbResetRequested=false;
	}
}



void MapMerging::RunGlobalBundleAdjustment(unsigned long nLoopKF, const int dstMapId)
{
	cout << "Starting Global Bundle Adjustment" << endl;

	Map *pDstMap = mpMapDatabase->GetSingleMapHolderByMapId(dstMapId)->pMap;

	// We do not allow to stop the GBA on map fusion
	bool bStopGBA = false;
	Optimizer::GlobalBundleAdjustemnt(pDstMap,20,&bStopGBA,nLoopKF,false);



	// Update all MapPoints and KeyFrames
	// Local Mapping was active during BA, that means that there might be new keyframes
	// not included in the Global BA and they are not consistent with the updated map.
	// We need to propagate the correction through the spanning tree
	{
		unique_lock<mutex> lock(mMutexGBA);
		cout << "Global Bundle Adjustment finished" << endl;
		cout << "Updating map ..." << endl;

		std::vector<MapHolder *> vpMapHolder = mpMapDatabase->GetMapHolderByMapId(dstMapId);

		for( MapHolder *pMapHolder : vpMapHolder )
			pMapHolder->pLocalMapper->RequestStop();


		for( MapHolder *pMapHolder : vpMapHolder )
		{
			pMapHolder->pTracker->Pause();
			while( !pMapHolder->pLocalMapper->isStopped())
			{
				usleep(1000);
			}
		}


		// Correct keyframes starting at map first keyframe
		list<KeyFrame*> lpKFtoCheck(pDstMap->mvpKeyFrameOrigins.begin(),pDstMap->mvpKeyFrameOrigins.end());

		while(!lpKFtoCheck.empty())
		{
			KeyFrame* pKF = lpKFtoCheck.front();
			const set<KeyFrame*> sChilds = pKF->GetChilds();
			cv::Mat Twc = pKF->GetPoseInverse();
			for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
			{
				KeyFrame* pChild = *sit;
				if(pChild->mnBAGlobalForKF!=nLoopKF)
				{
					cv::Mat Tchildc = pChild->GetPose()*Twc;
					pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
					pChild->mnBAGlobalForKF=nLoopKF;
				}
				lpKFtoCheck.push_back(pChild);
			}

			pKF->mTcwBefGBA = pKF->GetPose();
			pKF->SetPose(pKF->mTcwGBA);
			lpKFtoCheck.pop_front();
		}

		std::cout << "Keyframes corrected" << std::endl;

		// Correct MapPoints
		const vector<MapPoint*> vpMPs = pDstMap->GetAllMapPoints();
		for(size_t i=0; i<vpMPs.size(); i++)
		{
			MapPoint* pMP = vpMPs[i];

			if(pMP->isBad())
				continue;

			if(pMP->mnBAGlobalForKF==nLoopKF)
			{
				// If optimized by Global BA, just update
				pMP->SetWorldPos(pMP->mPosGBA);
			}
			else
			{
				// Update according to the correction of its reference keyframe
				KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

				if(pRefKF->mnBAGlobalForKF!=nLoopKF)
					continue;

				// Map to non-corrected camera
				cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
				cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
				cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

				// Backproject using corrected camera
				cv::Mat Twc = pRefKF->GetPoseInverse();
				cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
				cv::Mat twc = Twc.rowRange(0,3).col(3);

				pMP->SetWorldPos(Rwc*Xc+twc);
			}
		}

		cout << "Map updated!" << endl;
		mbRunningGBA = false;

		for( MapHolder *pMapHolder : vpMapHolder )
		{
			pMapHolder->pLocalMapper->Release();
			pMapHolder->pTracker->Resume();
		}

		mpMapDatabase->GetSingleMapHolderByMapId(dstMapId)->pLoopCloser->Release();
	}


}

void MapMerging::RequestFinish()
{
	unique_lock<mutex> lock(mMutexFinish);
	mbFinishRequested = true;
}

bool MapMerging::CheckFinish()
{
	unique_lock<mutex> lock(mMutexFinish);
	return mbFinishRequested;
}

void MapMerging::SetFinish()
{
	unique_lock<mutex> lock(mMutexFinish);
	mbFinished = true;
}

bool MapMerging::isFinished()
{
	unique_lock<mutex> lock(mMutexFinish);
	return mbFinished;
}


} //namespace ORB_SLAM
