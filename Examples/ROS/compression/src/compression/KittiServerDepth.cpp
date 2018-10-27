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


#include <signal.h>

#include "boost/program_options.hpp"


#include "ros/ros.h"
#include "compression/msg_features.h"

#include "System.h"
#include "feature_coder.h"





namespace po = boost::program_options;

// Setup decoder
ORBVocabulary voc;
LBFC2::CodingStats codingModel;
CORB_SLAM2::System *SLAM;
ros::AsyncSpinner *spinner;

// Adapt to agent
int bufferSize = 1;
bool inter = true;
bool stereo = false;
bool depth = true;

int nlevels = 8;
int imgWidth = 1241;
int imgHeight = 376;

std::map<int, std::mutex> mutexPool;
std::map<int, std::thread> mThreadMap;



void signal_handler(int signal)
{
	//Save trajectory / stats etc.
	std::cout << "Shutting down" << std::endl;
	SLAM->Shutdown();


	std::cout << "Exit." << std::endl;
	exit(signal);
}



void trackDepth(CORB_SLAM2::System *SLAM, const CORB_SLAM2::FrameInfo &info, const std::vector<cv::KeyPoint> &keypoints,
		const cv::Mat &descriptors, const std::vector<unsigned int> &visualWords,
		const std::vector<float> &vfDepthValues, const double &timestamp, int nAgentId)
{
	SLAM->TrackRGBDCompressed(info, keypoints, descriptors, visualWords, vfDepthValues, timestamp, nAgentId);
}



void callback(const compression::msg_features::ConstPtr msg, CORB_SLAM2::System *SLAM, std::map<int, LBFC2::FeatureCoder *> *coderMap)
{
	// Convert bitstream
	std::vector<uchar> img_bitstream(msg->data.begin(), msg->data.end());

	// Setup visual variables
	std::vector<unsigned int> vDecVisualWords;
	std::vector<cv::KeyPoint> vDecKeypointsLeft;
	cv::Mat decDescriptorsLeft;
	std::vector<float> vfDepthValues;


	// Get frame info
	CORB_SLAM2::FrameInfo info;
	info.mnHeight = imgHeight;
	info.mnWidth = imgWidth;

	// Lock decoder for the corresponding agent id
	int nAgentId = msg->nrobotid;
	std::unique_lock<std::mutex> lock(mutexPool[nAgentId]);

	// Check if decoding instance is available
	if( coderMap->find(nAgentId) == coderMap->end() )
		(*coderMap)[nAgentId] = new LBFC2::FeatureCoder(voc, codingModel, imgWidth, imgHeight, nlevels, 32, bufferSize, inter, stereo, depth);


	// Decode the features
	(*coderMap)[nAgentId]->decodeImageDepth(img_bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords, vfDepthValues);


	// Wait previous tracking to finish
	if( mThreadMap[nAgentId].joinable() )
		mThreadMap[nAgentId].join();


	// Pass the images to the SLAM system in parallel
	const double tframe = msg->tframe;



	mThreadMap[nAgentId] = std::thread(&trackDepth, SLAM, info, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords, vfDepthValues, tframe, nAgentId);
}



int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("voc,v", po::value<std::string>(), "Vocabulary path")
		("coding,c", po::value<std::string>(), "coding model")
		("settings,s", po::value<std::string>(), "settings base path");


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);


	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	std::cout << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);



	// Load coding model
	std::string stats_path = vm["coding"].as<std::string>();
	std::cout << "Loading statistics from " << stats_path << std::endl;
	codingModel.load(stats_path);



	// Setup ORB SLAM - make sure to use the correct settings file
	std::string settings_path = vm["settings"].as<std::string>();
	const string &strSettingsFile1 = settings_path + "/KITTI00-02.yaml";
	const string &strSettingsFile2 = settings_path + "/KITTI04-12.yaml";
	SLAM = new CORB_SLAM2::System(voc_path);


	bool bUseViewer = true;

	// Call init robot prior to tracking.
	SLAM->InitAgent(0, strSettingsFile1, CORB_SLAM2::Sensor::RGBD, bUseViewer);
	SLAM->InitAgent(1, strSettingsFile2, CORB_SLAM2::Sensor::RGBD, bUseViewer);


	// Setup node
	std::string name = "server";
	ros::init(argc, argv, name.c_str());
	ros::NodeHandle nh;


	// Setup signal handler to store trajectory
	signal(SIGINT, signal_handler);




	std::map<int, LBFC2::FeatureCoder *> coderMap;
	std::map<int, ros::Subscriber> subscriberMap;
	std::map<int, std::thread> threadPool;


	// Setup ros subscriber
	std::vector<int> vnRobots = {0, 1};
	for( size_t n = 0; n < vnRobots.size(); n++ )
	{
		int nAgentId = vnRobots[n];

		std::map<int, ros::Publisher> mFeedbackPub;
		std::map<int, ros::Subscriber> mBitstreamSub;

		std::string sRobotId = std::to_string(nAgentId);
		std::string bitstreamRobot = "/featComp/bitstream" + sRobotId;
		subscriberMap[nAgentId] = nh.subscribe<compression::msg_features>(bitstreamRobot, 10000, boost::bind(callback, _1, SLAM, &coderMap));
	}


	// Lets spin
	ros::MultiThreadedSpinner spinner(vnRobots.size());
	spinner.spin();

	ros::waitForShutdown();



	std::cout << "Finished" << std::endl;

	signal_handler(0);

	return 0;
}
