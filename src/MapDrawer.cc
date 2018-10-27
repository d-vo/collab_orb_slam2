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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <pangolin/display/display_internal.h>
#include <mutex>


std::mutex gGuiMutex;
namespace CORB_SLAM2
{


typedef struct {
	double r;
	double g;
	double b;
} rgb;

typedef struct {
	double h;
	double s;
	double v;
} hsv;


static rgb hsv2rgb(hsv in);

rgb hsv2rgb(hsv in)
{
	// Partially from https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
	double      hh, p, q, t, ff;
	long        i;
	rgb         out;

	assert(in.h <= 1.0 && in.h >= 0.0);
	assert(in.s <= 1.0 && in.s >= 0.0);
	assert(in.v <= 1.0 && in.v >= 0.0);

	if(in.s <= 0.0) {
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}
	hh = in.h;
	hh *= 6;
	i = (long)hh;
	ff = hh - i;
	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch(i) {
	case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	case 5:
	default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	}
	return out;
}



MapDrawer::MapDrawer(Map* pMap, SLAMConfig *pSlamConfig, int nAgentId)
: mpMap(pMap), mpTracker(NULL), mpSlamConfig(pSlamConfig), mnAgentId(nAgentId), mbFinishRequested(false),
  mbFinished(true), mbStopRequested(false), mbStopped(true)
{
	float fps = pSlamConfig->mfps;
	if(fps<1)
		fps=30;
	mT = 1e3/fps;

	mViewpointX = pSlamConfig->mViewpointX;
	mViewpointY = pSlamConfig->mViewpointY;
	mViewpointZ = pSlamConfig->mViewpointZ;
	mViewpointF = pSlamConfig->mViewpointF;

	// Custom color for evaluation
	mvColorPalette.push_back(cv::Point3f(0, 0.4470, 0.7410));
	mvColorPalette.push_back(cv::Point3f(0.8500, 0.3250, 0.0980));
	mvColorPalette.push_back(cv::Point3f(0.9290, 0.6940, 0.1250));

	// Generate pleasant colors
	const float golden_ratio_c= 0.618033988749895;
	float h = 0;
	for( unsigned int i = 0; i < 256; i++ )
	{
		hsv hsvColor;
		hsvColor.h = h;
		hsvColor.s = 0.5;
		hsvColor.v = 0.95;

		rgb col = hsv2rgb(hsvColor);

		// Generate palette
		mvColorPalette.push_back(cv::Point3f(col.r, col.g, col.b));

		h += golden_ratio_c;
		while( h >= 1.0)
			h -= 1.0;
	}
}


void MapDrawer::SetTracker(Tracking* pTracker)
{
	mpTracker = pTracker;
}


void MapDrawer::ChangeMap(Map *pMap)
{
	unique_lock<mutex>(mMutexMap);
	mpMap = pMap;
}


void MapDrawer::Run()
{
	// I admit, this is somehow ugly, but you should not generate multiple GUIs simultaneously.
	gGuiMutex.lock();
	std::string windowName = "Remote ORB-SLAM2: Map Viewer - Robot: " + std::to_string(mnAgentId);
	std::string menuString = "menu" + std::to_string(mnAgentId);


	pangolin::CreateWindowAndBind(windowName.c_str(),1024,768);
	pangolin::CreatePanel(menuString).SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));


	// 3D Mouse handler requires depth testing to be enabled
	glEnable(GL_DEPTH_TEST);

	// Issue specific OpenGl we might need
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);



	pangolin::Var<bool> menuFollowCamera(menuString + ".Follow Camera",false,true);
	pangolin::Var<bool> menuShowPoints(menuString + ".Show Points",true,true);
	pangolin::Var<bool> menuShowKeyFrames(menuString + ".Show KeyFrames",true,true);
	pangolin::Var<bool> menuShowGraphCov(menuString + ".Show Cov Graph",true,true);
	pangolin::Var<bool> menuShowGraphMST(menuString + ".Show MST Graph",false,true);
	pangolin::Var<bool> menuShowGraphLoops(menuString + ".Show Loops",true,true);
	pangolin::Var<bool> menuShowReferencePts(menuString + ".Show Reference Pts",false,true);
	pangolin::Var<bool> menuShowObservations(menuString + ".Show Observations",false,true);
	pangolin::Var<bool> menuReset(menuString + ".Reset",false,false);
	pangolin::Var<int> 	menuMapId(menuString +".Map Id",mpMap->GetMapId(),0, 0);
	pangolin::Var<int> 	menuMapPointSize(menuString +".Map Points",mpMap->MapPointsInMap(),0, 0);
	pangolin::Var<int> 	menuMapKeyFrameSize(menuString +".Map KeyFrames",mpMap->MapPointsInMap(),0, 0);

	gGuiMutex.unlock();


	// Define Camera Render Object (for view / scene browsing)
	pangolin::OpenGlRenderState s_cam(
			pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000),
			pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
	);

	// Add named OpenGL viewport to window and provide 3D Handler
	pangolin::View& d_cam = pangolin::CreateDisplay()
	.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
	.SetHandler(new pangolin::Handler3D(s_cam));

	pangolin::OpenGlMatrix Twc;
	Twc.SetIdentity();


	{
		unique_lock<mutex> lock(mMutexFinish);
		mbFinished = false;
	}

	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
	}


	bool bFollow = true;
	while(1)
	{
		std::unique_lock<std::mutex> lock(mMutexDrawing);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		{
			unique_lock<mutex> lock(mMutexCamera);
			ConvertToOpenGLCameraMatrix(mmTcw[mnAgentId], Twc);
		}



		if(menuFollowCamera && bFollow)
		{
			s_cam.Follow(Twc);
		}
		else if(menuFollowCamera && !bFollow)
		{
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
			s_cam.Follow(Twc);
			bFollow = true;
		}
		else if(!menuFollowCamera && bFollow)
		{
			bFollow = false;
		}


		d_cam.Activate(s_cam);
		glClearColor(1.0f,1.0f,1.0f,1.0f);


		{
			unique_lock<mutex> lock(mMutexCamera);

			for( auto it : mmTcw  )
			{
				pangolin::OpenGlMatrix Twc;
				ConvertToOpenGLCameraMatrix(it.second, Twc);
				DrawCamera(it.first, Twc);
			}
		}


		if(menuShowKeyFrames || menuShowGraphCov || menuShowGraphMST || menuShowGraphLoops)
			DrawKeyFrames(menuShowKeyFrames, menuShowGraphCov, menuShowGraphMST, menuShowGraphLoops);
		if(menuShowPoints)
			DrawMapPoints(menuShowObservations, menuShowReferencePts);

		pangolin::FinishFrame();

		std::this_thread::sleep_for(std::chrono::milliseconds(mT));

		if(menuReset)
		{
			menuShowGraphCov = true;
			menuShowGraphMST = true;
			menuShowKeyFrames = true;
			menuShowPoints = true;
			bFollow = true;
			menuFollowCamera = true;
			menuReset = false;
		}

		menuMapId = mpMap->GetMapId();
		menuMapPointSize = mpMap->MapPointsInMap();
		menuMapKeyFrameSize = mpMap->KeyFramesInMap();


		if(Stop())
		{
			// Safe area to stop
			while(isStopped() )
			{
				usleep(3000);
			}
		}


		if(CheckFinish())
			break;
	}

	unique_lock<mutex> lock(mMutexFinish);
	mbFinished = true;
	pangolin::DestroyWindow(windowName.c_str());
}

bool MapDrawer::CheckFinish()
{
	unique_lock<mutex> lock(mMutexFinish);
	return mbFinishRequested;
}

void MapDrawer::RequestFinish()
{
	unique_lock<mutex> lock(mMutexFinish);
	mbFinishRequested = true;
}

bool MapDrawer::isFinished()
{
	unique_lock<mutex> lock(mMutexFinish);
	return mbFinished;
}



void MapDrawer::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
	std::cout << "Map Drawing Request Stop" << std::endl;
}

bool MapDrawer::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if( mbStopRequested )
	{
		mbStopped = true;
		return true;
	}

	return false;
}

bool MapDrawer::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void MapDrawer::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	unique_lock<mutex> lock2(mMutexFinish);
	if(mbFinished)
		return;

	mbStopped = false;
	mbStopRequested = false;

	std::cout << "Map Drawer Release" << std::endl;
}

void MapDrawer::DrawMapPoints(bool bDrawObservations, bool bDrawReferencePoints)
{
	vector<MapPoint*> vpMPs;
	vector<MapPoint*> vpRefMPs;
	vector<MapPoint*> vpTrackedMPs;
	{
		unique_lock<mutex>(mMutexMap);
		vpMPs = mpMap->GetAllMapPoints();

		if( bDrawReferencePoints )
			vpRefMPs = mpTracker->GetReferenceMapPoints();

		if( bDrawObservations )
			vpTrackedMPs = mpTracker->GetTrackedMapPoints();
	}


	set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

	if(vpMPs.empty())
		return;

	glPointSize(mpSlamConfig->mPointSize);
	glBegin(GL_POINTS);
	glColor3f(0.0,0.0,0.0);

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		if(vpMPs[i]->isBad())
			continue;

		if( bDrawReferencePoints && spRefMPs.count(vpMPs[i]) )
			continue;


		const int nAgentId = vpMPs[i]->GetAgentId();




		cv::Point3f color =  mvColorPalette.at(nAgentId);
		glColor3f(color.x,color.y,color.z);

		cv::Mat pos = vpMPs[i]->GetWorldPos();
		if( !pos.empty() )
			glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	}
	glEnd();




	if( bDrawReferencePoints )
	{
		glPointSize(mpSlamConfig->mPointSize);
		glBegin(GL_POINTS);
		glColor3f(1.0,0.0,0.0);

		for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
		{
			if((*sit)->isBad())
				continue;
			cv::Mat pos = (*sit)->GetWorldPos();
			glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

		}

		glEnd();
	}



	if( bDrawObservations )
	{
		glBegin(GL_LINES);

		for(MapPoint *pMP : vpTrackedMPs )
		{
			if(pMP->isBad() )
				continue;

			cv::Mat pos = pMP->GetWorldPos();

			const int nPointAgentId = pMP->GetAgentId();
			cv::Point3f color =  mvColorPalette.at(nPointAgentId);
			glColor4f(color.x,color.y,color.z, 0.2);

			cv::Mat Tcw = mmTcw[mnAgentId];

			cv::Mat Rwc(3,3,CV_32F);
			cv::Mat twc(3,1,CV_32F);

			Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			twc = -Rwc*Tcw.rowRange(0,3).col(3);

			const float &x = twc.at<float>(0);
			const float &y = twc.at<float>(1);
			const float &z = twc.at<float>(2);



			glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
			glVertex3f(x,y,z);
		}
		glEnd();
	}

}

void MapDrawer::DrawKeyFrames(const bool bDrawKF,  const bool bDrawGraphCovisibility, const bool bDrawMSTGraph,
		const bool bDrawLoop )
{
	const float &w = mpSlamConfig->mKeyFrameSize;
	const float h = w*0.75;
	const float z = w*0.6;

	vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();


	if(bDrawKF)
	{
		for(size_t i=0; i<vpKFs.size(); i++)
		{
			KeyFrame* pKF = vpKFs[i];
			cv::Mat Twc = pKF->GetPoseInverse().t();

			glPushMatrix();

			glMultMatrixf(Twc.ptr<GLfloat>(0));

			glLineWidth(mpSlamConfig->mKeyFrameLineWidth);

			int nAgentId = pKF->GetAgentId();
			cv::Point3f color =  mvColorPalette.at(nAgentId);
			glColor3f(color.x,color.y,color.z);

			glBegin(GL_LINES);
			glVertex3f(0,0,0);
			glVertex3f(w,h,z);
			glVertex3f(0,0,0);
			glVertex3f(w,-h,z);
			glVertex3f(0,0,0);
			glVertex3f(-w,-h,z);
			glVertex3f(0,0,0);
			glVertex3f(-w,h,z);

			glVertex3f(w,h,z);
			glVertex3f(w,-h,z);

			glVertex3f(-w,h,z);
			glVertex3f(-w,-h,z);

			glVertex3f(-w,h,z);
			glVertex3f(w,h,z);

			glVertex3f(-w,-h,z);
			glVertex3f(w,-h,z);
			glEnd();

			glPopMatrix();
		}
	}

	for(size_t i=0; i<vpKFs.size(); i++)
	{
		cv::Mat Ow = vpKFs[i]->GetCameraCenter();

		if(bDrawGraphCovisibility)
		{
			// Covisibility Graph
			glLineWidth(mpSlamConfig->mGraphLineWidth);
			glColor4f(0.0f,1.0f,0.0f,0.6f);
			glBegin(GL_LINES);
			const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);

			if(!vCovKFs.empty())
			{
				for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
				{
					if((*vit)->mnId<vpKFs[i]->mnId)
						continue;
					cv::Mat Ow2 = (*vit)->GetCameraCenter();
					glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
					glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
				}
			}
			glEnd();
		}

		if( bDrawMSTGraph)
		{
			// Spanning tree
			glLineWidth(mpSlamConfig->mGraphLineWidth*3);
			KeyFrame* pParent = vpKFs[i]->GetParent();

			if(pParent)
			{
				glColor4f(1.0f,0.4f,0.0f,0.6f);

				glBegin(GL_LINES);
				cv::Mat Owp = pParent->GetCameraCenter();
				glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
				glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
				glEnd();
			}

		}

		if( bDrawLoop )
		{
			// Loops
			glColor4f(0.0f,0.4f,1.0f,0.6f);
			glBegin(GL_LINES);
			set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
			for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
			{
				if((*sit)->mnId<vpKFs[i]->mnId)
					continue;
				cv::Mat Owl = (*sit)->GetCameraCenter();
				glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
				glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
			}
			glEnd();
		}
	}
}



void MapDrawer::DrawCamera(int nAgentId, pangolin::OpenGlMatrix &Twc)
{
	const float &w = mpSlamConfig->mCameraSize;
	const float h = w*0.75;
	const float z = w*0.6;

	glPushMatrix();

#ifdef HAVE_GLES
	glMultMatrixf(Twc.m);
#else
	glMultMatrixd(Twc.m);
#endif

	glLineWidth(mpSlamConfig->mCameraLineWidth);
	cv::Point3f color =  mvColorPalette.at(nAgentId);
	glColor3f(color.x,color.y,color.z);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(w,h,z);
	glVertex3f(0,0,0);
	glVertex3f(w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,h,z);

	glVertex3f(w,h,z);
	glVertex3f(w,-h,z);

	glVertex3f(-w,h,z);
	glVertex3f(-w,-h,z);

	glVertex3f(-w,h,z);
	glVertex3f(w,h,z);

	glVertex3f(-w,-h,z);
	glVertex3f(w,-h,z);
	glEnd();

	glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(int nAgentId, const cv::Mat &Tcw)
{
	unique_lock<mutex> lock(mMutexCamera);
	mmTcw[nAgentId] = Tcw.clone();
}


void MapDrawer::ConvertToOpenGLCameraMatrix(const cv::Mat &Tcw,  pangolin::OpenGlMatrix &mTwc)
{
	if(!Tcw.empty())
	{

		cv::Mat Rwc(3,3,CV_32F);
		cv::Mat twc(3,1,CV_32F);

		Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
		twc = -Rwc*Tcw.rowRange(0,3).col(3);


		mTwc.m[0] = Rwc.at<float>(0,0);
		mTwc.m[1] = Rwc.at<float>(1,0);
		mTwc.m[2] = Rwc.at<float>(2,0);
		mTwc.m[3]  = 0.0;

		mTwc.m[4] = Rwc.at<float>(0,1);
		mTwc.m[5] = Rwc.at<float>(1,1);
		mTwc.m[6] = Rwc.at<float>(2,1);
		mTwc.m[7]  = 0.0;

		mTwc.m[8] = Rwc.at<float>(0,2);
		mTwc.m[9] = Rwc.at<float>(1,2);
		mTwc.m[10] = Rwc.at<float>(2,2);
		mTwc.m[11]  = 0.0;

		mTwc.m[12] = twc.at<float>(0);
		mTwc.m[13] = twc.at<float>(1);
		mTwc.m[14] = twc.at<float>(2);
		mTwc.m[15]  = 1.0;
	}
	else
		mTwc.SetIdentity();

}



} //namespace ORB_SLAM
