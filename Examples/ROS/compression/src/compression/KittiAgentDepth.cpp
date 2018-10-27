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

#include "boost/program_options.hpp"

#include "ros/ros.h"
#include "compression/msg_features.h"

#include "System.h"
#include "feature_coder.h"



namespace po = boost::program_options;


std::shared_ptr<CORB_SLAM2::ORBextractor> mpORBextractorLeft;
std::shared_ptr<CORB_SLAM2::ORBextractor> mpORBextractorRight;

int nFeatures;
float fScaleFactor;
int nLevels;
int fIniThFAST;
int fMinThFAST;
cv::Mat K0;
cv::Mat DistCoef0;
cv::Mat M1l,M2l;
cv::Mat M1r,M2r;
float mBaseline;
float mFocalLength;



void ExtractORB(int flag, const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &descriptors)
{
    if(flag==0)
    {
        (*mpORBextractorLeft)(im,cv::Mat(),vKeys,descriptors);
    }
    else
    {
        (*mpORBextractorRight)(im,cv::Mat(),vKeys,descriptors);
    }

}

void LoadImagesKittiStereo(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
		std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps)
{
	std::ifstream fTimes;
	std::string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while(!fTimes.eof())
	{
		std::string s;
		getline(fTimes,s);
		if(!s.empty())
		{
			std::stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	std::string strPrefixLeft = strPathToSequence + "/image_0/";
	std::string strPrefixRight = strPathToSequence + "/image_1/";

	const int nTimes = vTimestamps.size();
	vstrImageLeft.resize(nTimes);
	vstrImageRight.resize(nTimes);

	for(int i=0; i<nTimes; i++)
	{
		std::stringstream ss;
		ss << std::setfill('0') << std::setw(6) << i;
		vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
		vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
	}
}


void loadSettings(const std::string &settingsFile)
{
	// Load camera parameters from settings file
	cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];

	K0 = cv::Mat::eye(3,3,CV_32F);
	K0.at<float>(0,0) = fx;
	K0.at<float>(1,1) = fy;
	K0.at<float>(0,2) = cx;
	K0.at<float>(1,2) = cy;

	DistCoef0 = cv::Mat(4,1,CV_32F);
	DistCoef0.at<float>(0) = fsSettings["Camera.k1"];
	DistCoef0.at<float>(1) = fsSettings["Camera.k2"];
	DistCoef0.at<float>(2) = fsSettings["Camera.p1"];
	DistCoef0.at<float>(3) = fsSettings["Camera.p2"];
	const float k3 = fsSettings["Camera.k3"];
	if(k3!=0)
	{
		DistCoef0.resize(5);
		DistCoef0.at<float>(4) = k3;
	}

	float bf = fsSettings["Camera.bf"];
	mBaseline = bf / fx;
	mFocalLength = fx;

	cout << endl << "Camera Parameters: " << endl;
	cout << "- fx: " << fx << endl;
	cout << "- fy: " << fy << endl;
	cout << "- cx: " << cx << endl;
	cout << "- cy: " << cy << endl;
	cout << "- k1: " << DistCoef0.at<float>(0) << endl;
	cout << "- k2: " << DistCoef0.at<float>(1) << endl;
	if(DistCoef0.rows==5)
		cout << "- k3: " << DistCoef0.at<float>(4) << endl;
	cout << "- p1: " << DistCoef0.at<float>(2) << endl;
	cout << "- p2: " << DistCoef0.at<float>(3) << endl;


	// Load ORB parameters
	nFeatures = fsSettings["ORBextractor.nFeatures"];
	fScaleFactor = fsSettings["ORBextractor.scaleFactor"];
	nLevels = fsSettings["ORBextractor.nLevels"];
	fIniThFAST = fsSettings["ORBextractor.iniThFAST"];
	fMinThFAST = fsSettings["ORBextractor.minThFAST"];


	cout << endl  << "ORB Extractor Parameters: " << endl;
	cout << "- Number of Features: " << nFeatures << endl;
	cout << "- Scale Levels: " << nLevels << endl;
	cout << "- Scale Factor: " << fScaleFactor << endl;
	cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
	cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}




int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()
						("help", "produce help message")
						("voc,v", po::value<std::string>(), "Vocabulary path")
						("input,i", po::value<std::string>(), "Image path")
						("coding,c", po::value<std::string>(), "settings path")
						("settings,s", po::value<std::string>(), "ORB SLAM settings path")
						("robotid,r", po::value<int>(), "agent id");


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);


	// Load settings
	std::string strSettingPath = vm["settings"].as<std::string>();
	loadSettings(strSettingPath);


	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	ORBVocabulary voc;
	std::cout << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);


	// Load coding statistics
	std::string settings_path = vm["coding"].as<std::string>();
	std::cout << "Loading statistics from " << settings_path << std::endl;
	LBFC2::CodingStats codingModel;
	codingModel.load(settings_path );


	// Load images
	std::string image_path = vm["input"].as<std::string>();
	std::cout << "Loading image list from " << image_path << std::endl;
	std::vector<double> vTimestamps;
	std::vector<std::string> vCam0, vCam1;
	LoadImagesKittiStereo(image_path, vCam0, vCam1, vTimestamps);


	size_t nImages = vCam0.size();

	// Setup encoder
	int imgWidth = 1241;
	int imgHeight = 376;
	int bufferSize = 1;
	bool inter = true;
	bool stereo = false;
	bool depth = true;
	LBFC2::FeatureCoder encoder(voc, codingModel,imgWidth, imgHeight, nLevels, 32, bufferSize, inter, stereo, depth, mFocalLength, mBaseline);


	// Setup features
    mpORBextractorLeft = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
    mpORBextractorRight = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));


    // Setup ROS
    int nRobotId = vm["robotid"].as<int>();
  	std::string bitstreamTopic = "/featComp/bitstream" + std::to_string(nRobotId);

  	std::string name = "agent" + std::to_string(nRobotId);
  	ros::init(argc, argv, name.c_str());
  	ros::NodeHandle n;

  	ros::Publisher bitstream_pub = n.advertise<compression::msg_features>(bitstreamTopic, 1000, true);



	std::vector<cv::Mat> vImgLeft, vImgRight;
	for( size_t imgId = 0; imgId < nImages; imgId++ )
	{
		// Read left and right images from file
		cv::Mat imLeftDist = cv::imread(vCam0[imgId],cv::IMREAD_GRAYSCALE);
		cv::Mat imRightDist = cv::imread(vCam1[imgId],cv::IMREAD_GRAYSCALE);


		vImgLeft.push_back(imLeftDist);
		vImgRight.push_back(imRightDist);

		if( imgId % 64 == 0)
			std::cout << "Finished loading image " << imgId << std::endl;
	}

   ros::Rate poll_rate(100);
	while(bitstream_pub.getNumSubscribers() == 0)
		poll_rate.sleep();

	std::cout << "Start" << std::endl;



	for( size_t imgId= 0; imgId < nImages; imgId++)
	{
		cv::Mat imLeftRect = vImgLeft[imgId];
		cv::Mat imRightRect = vImgRight[imgId];


		// Extract features
		std::vector<cv::KeyPoint> keypointsLeft, keypointsRight;
		cv::Mat descriptorsLeft, descriptorsRight;



		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		std::thread threadLeft(ExtractORB,0,imLeftRect, std::ref(keypointsLeft), std::ref(descriptorsLeft));
		std::thread threadRight(ExtractORB,1,imRightRect, std::ref(keypointsRight), std::ref(descriptorsRight));
		threadLeft.join();
		threadRight.join();


		std::vector<uchar> bitstream;
		encoder.encodeImageStereo(keypointsLeft, descriptorsLeft, keypointsRight, descriptorsRight, bitstream);


		double tframe = vTimestamps[imgId];
		compression::msg_features msg;
		msg.header.stamp = ros::Time::now();
		msg.tframe = tframe;
		msg.nrobotid = nRobotId;
		msg.data.assign(bitstream.begin(),bitstream.end());
		bitstream_pub.publish(msg);

		ros::spinOnce();


		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimestamps[imgId]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(imgId<nImages-1)
            T = vTimestamps[imgId+1]-tframe;
        else if(imgId>0)
            T = tframe-vTimestamps[imgId-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

	}
}
