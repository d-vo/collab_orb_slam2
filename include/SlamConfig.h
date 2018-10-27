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


#ifndef SLAMCONFIG_H
#define SLAMCONFIG_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

namespace CORB_SLAM2
{


class SLAMConfig
{
public:
	void Load( std::string strSettingPath )
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		mfx = fSettings["Camera.fx"];
		mfy = fSettings["Camera.fy"];
		mcx = fSettings["Camera.cx"];
		mcy = fSettings["Camera.cy"];

		mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
		mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
		mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
		mPointSize = fSettings["Viewer.PointSize"];
		mCameraSize = fSettings["Viewer.CameraSize"];
		mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

		mViewpointX = fSettings["Viewer.ViewpointX"];
		mViewpointY = fSettings["Viewer.ViewpointY"];
		mViewpointZ = fSettings["Viewer.ViewpointZ"];
		mViewpointF = fSettings["Viewer.ViewpointF"];

		mfps = fSettings["Camera.fps"];

		mDistCoef = cv::Mat(4,1,CV_32F);
		mDistCoef.at<float>(0) = fSettings["Camera.k1"];
		mDistCoef.at<float>(1) = fSettings["Camera.k2"];
		mDistCoef.at<float>(2) = fSettings["Camera.p1"];
		mDistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if(k3!=0)
		{
			mDistCoef.resize(5);
			mDistCoef.at<float>(4) = k3;
		}

		mbf = fSettings["Camera.bf"];
		mnRGB = fSettings["Camera.RGB"];

		mnFeatures = fSettings["ORBextractor.nFeatures"];
		mfScaleFactor = fSettings["ORBextractor.scaleFactor"];
		mnLevels = fSettings["ORBextractor.nLevels"];
		mfIniThFAST = fSettings["ORBextractor.iniThFAST"];
		mfMinThFAST = fSettings["ORBextractor.minThFAST"];

		mThDepth = fSettings["ThDepth"];
		mDepthMapFactor = fSettings["DepthMapFactor"];
	}


public:
	float mfx;
	float mfy;
	float mcx;
	float mcy;

	int mnRGB;

	int mnFeatures;
	float mfScaleFactor;
	int mnLevels;
	int mfIniThFAST;
	int mfMinThFAST;

	float mThDepth;
	float mDepthMapFactor;

	int width;
	int height;
	int scaleLevels;
	float scaleFactor;

	cv::Mat mK;
	cv::Mat mDistCoef;
	float mbf;
	float thDepth;
	ORBextractor *mpORBextractorLeft;

	float mfps;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    int mT;
};

}

#endif
