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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include"System.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace CORB_SLAM2
{
class Tracking;
class SLAMConfig;
class MapDrawer
{
public:
    MapDrawer(Map* pMap, SLAMConfig *pSlamConfig, int nRobotId = -1);

    void SetTracker(Tracking* pTracker);
    void ChangeMap(Map *pMap);

    void DrawMapPoints(bool bDrawObservations, bool bDrawReferencePoints);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraphCovisibility, const bool bDrawMSTGraph, const bool bDrawLoops);
    void DrawCamera(int nRobotId, pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(int nRobotId, const cv::Mat &Tcw);

    void SetReferenceKeyFrame(KeyFrame *pKF);
    void ConvertToOpenGLCameraMatrix(const cv::Mat &cameraPose,  pangolin::OpenGlMatrix &M);

    void Run();

    void RequestFinish();
    bool CheckFinish();
    bool isFinished();



    void RequestStop();
    void Release();

    bool isStopped();
    bool Stop();


private:
    std::map<int, cv::Mat> mmTcw;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    int mT;

    std::vector<cv::Point3f> mvColorPalette;

    std::string mWwindowName;

    Map* mpMap;
    Tracking* mpTracker;
    SLAMConfig *mpSlamConfig;
    std::vector<Tracking *> mvpTracker;

    int mnAgentId;


    bool mbFinishRequested;
    bool mbFinished;

    bool mbStopRequested;
    bool mbStopped;

    std::mutex mMutexMap;
    std::mutex mMutexCamera;
    std::mutex mMutexUpdate;
    std::mutex mMutexFinish;
    std::mutex mMutexStop;
    std::mutex mMutexDrawing;

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
