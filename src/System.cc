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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace CORB_SLAM2
{

System::System(const string &strVocFile)
: mbReset(false),mbActivateLocalizationMode(false),
  mbDeactivateLocalizationMode(false)
{
	// Output welcome message
	cout << endl <<
			"ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
			"This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
			"This is free software, and you are welcome to redistribute it" << endl <<
			"under certain conditions. See LICENSE.txt." << endl << endl;



	//Load ORB Vocabulary
	cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

	mpVocabulary = new ORBVocabulary();
	bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	if(!bVocLoad)
	{
		cerr << "Wrong path to vocabulary. " << endl;
		cerr << "Falied to open at: " << strVocFile << endl;
		exit(-1);
	}
	cout << "Vocabulary loaded!" << endl << endl;



	// MULTI ROBOT
	mpMapDatabase = new MapDatabase(this, mpVocabulary);
}

void System::InitAgent( int nRobotId, std::string strSettings, Sensor sensor, bool bUseViewer )
{
	SLAMConfig *pSlamConfig = new SLAMConfig;
	pSlamConfig->Load(strSettings);
	mpMapDatabase->AddMap(nRobotId, pSlamConfig, sensor, bUseViewer);
}


cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, int nRobotId)
{
	Tracking *pTracker = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pTracker;
	LocalMapping *pLocalMapper = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pLocalMapper;

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			pLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while(!pLocalMapper->isStopped())
			{
				usleep(1000);
			}

			pTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if(mbDeactivateLocalizationMode)
		{
			pTracker->InformOnlyTracking(false);
			pLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			pTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = pTracker->GrabImageStereo(imLeft,imRight,timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = pTracker->mState;
	mTrackedMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = pTracker->mCurrentFrame.mvKeysUn;
	return Tcw;
}

cv::Mat System::TrackStereoCompressed(const FrameInfo &info, const std::vector<cv::KeyPoint> &keyPointsLeft,
		const cv::Mat &descriptorLeft, const std::vector<unsigned int> &visualWords,
		const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight,
		const double &timestamp, int nRobotId)
{
	Tracking *pTracker = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pTracker;
	LocalMapping *pMapper = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pLocalMapper;

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			pMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while(!pMapper->isStopped())
			{
				usleep(1000);
			}

			pTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if(mbDeactivateLocalizationMode)
		{
			pTracker->InformOnlyTracking(false);
			pMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			pTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = pTracker->GrabImageStereoCompressed(info, keyPointsLeft,descriptorLeft, visualWords,
			keyPointsRight,descriptorRight,timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = pTracker->mState;
	return Tcw;
}




cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, int nRobotId)
{
	if(mSensor!=RGBD)
	{
		cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
		exit(-1);
	}

	Tracking *pTracker = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pTracker;
	LocalMapping *pLocalMapper = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pLocalMapper;

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			pLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while(!pLocalMapper->isStopped())
			{
				usleep(1000);
			}

			pTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if(mbDeactivateLocalizationMode)
		{
			pTracker->InformOnlyTracking(false);
			pLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			pTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = pTracker->GrabImageRGBD(im,depthmap,timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = pTracker->mState;
	mTrackedMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = pTracker->mCurrentFrame.mvKeysUn;
	return Tcw;
}


cv::Mat System::TrackRGBDCompressed(const FrameInfo &info, const std::vector<cv::KeyPoint> &keypoints,
		const cv::Mat &descriptors, const std::vector<unsigned int> &visualWords,
		const std::vector<float> &vfDepthValues, const double &timestamp, int nRobotId)
{
	Tracking *pTracker = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pTracker;
	LocalMapping *pMapper = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pLocalMapper;

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			pMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while(!pMapper->isStopped())
			{
				usleep(1000);
			}

			pTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if(mbDeactivateLocalizationMode)
		{
			pTracker->InformOnlyTracking(false);
			pMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			pTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = pTracker->GrabImageRGBDCompressed(info, keypoints, descriptors, visualWords, vfDepthValues, timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = pTracker->mState;
	return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, int nRobotId)
{
	if(mSensor!=MONOCULAR)
	{
		cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
		exit(-1);
	}

	Tracking *pTracker = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pTracker;
	LocalMapping *pLocalMapper = mpMapDatabase->GetMapHolderByAgentId(nRobotId)->pLocalMapper;

	// Check mode change
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			pLocalMapper->RequestStop();

			// Wait until Local Mapping has effectively stopped
			while(!pLocalMapper->isStopped())
			{
				usleep(1000);
			}

			pTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		if(mbDeactivateLocalizationMode)
		{
			pTracker->InformOnlyTracking(false);
			pLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}

	// Check reset
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			pTracker->Reset();
			mbReset = false;
		}
	}

	cv::Mat Tcw = pTracker->GrabImageMonocular(im,timestamp);

	unique_lock<mutex> lock2(mMutexState);
	mTrackingState = pTracker->mState;
	mTrackedMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
	mTrackedKeyPointsUn = pTracker->mCurrentFrame.mvKeysUn;

	return Tcw;
}

void System::ActivateLocalizationMode()
{
	unique_lock<mutex> lock(mMutexMode);
	mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
	unique_lock<mutex> lock(mMutexMode);
	mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
	return false;
}

void System::Reset()
{
	unique_lock<mutex> lock(mMutexReset);
	mbReset = true;
}

void System::Shutdown()
{
	mpMapDatabase->Shutdown();
}

void System::SaveTrajectoryEuroC(const string &folder)
{
	cout << endl << "Saving camera trajectory to " << folder << " ..." << endl;

	std::vector<MapHolder *> vpHolder = mpMapDatabase->GetMapHolders();
	for( MapHolder *pHolder : vpHolder)
	{
		Map *pMap = pHolder->pMap;
		vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		if( vpKFs.empty() )
		{
			std::cout << "No keyframes" << std::endl;
			continue;
		}

		cv::Mat Two = vpKFs[0]->GetPoseInverse();


		std::string filename = folder + "/Robot_" + std::to_string(pHolder->nAgentId) + "_Trajectory.txt";

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.
		Tracking *pTracker = pHolder->pTracker;

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<KeyFrame*>::iterator lRit = pTracker->mlpReferences.begin();
		list<double>::iterator lT = pTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = pTracker->mlbLost.begin();
		for(list<cv::Mat>::iterator lit=pTracker->mlRelativeFramePoses.begin(),
				lend=pTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
		{
			if(*lbL)
				continue;

			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while(pKF->isBad())
			{
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			unsigned long t = round(*lT * 1e9);

			f << t << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();



		cout << endl << "trajectory saved!" << endl;

	}

}

void System::SaveTrajectoryKITTI(const string &folder)
{
	cout << endl << "Saving camera trajectory to " << folder << " ..." << endl;
	if(mSensor==MONOCULAR)
	{
		cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
		return;
	}


	std::vector<MapHolder *> vpHolder = mpMapDatabase->GetMapHolders();
	for( MapHolder *pHolder : vpHolder)
	{
		Map *pMap = pHolder->pMap;
		Tracking *pTracker = pHolder->pTracker;

		vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
		sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

		if( vpKFs.empty() )
		{
			std::cout << "No keyframes" << std::endl;
			continue;
		}

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		std::string filename = folder + "/KITTI_Robot_" + std::to_string(pHolder->nAgentId) + "_Trajectory.txt";

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<KeyFrame*>::iterator lRit = pTracker->mlpReferences.begin();
		list<double>::iterator lT = pTracker->mlFrameTimes.begin();
		for(list<cv::Mat>::iterator lit=pTracker->mlRelativeFramePoses.begin(), lend=pTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
		{
			KeyFrame* pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

			while(pKF->isBad())
			{
				//  cout << "bad parent" << endl;
				Trw = Trw*pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw*pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

			f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
					Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
					Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}
}

int System::GetTrackingState()
{
	unique_lock<mutex> lock(mMutexState);
	return mTrackingState;
}







} //namespace ORB_SLAM
