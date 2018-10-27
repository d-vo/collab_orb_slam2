# Collaborative ORB-SLAM2 Framework
# 1. About

**Summary:**  
This repository contains a public version of a centralized visual SLAM framework based on ORB-SLAM2. It is capable of running several visual SLAM instances for different agents in parallel. When overlap between maps is detected, the maps are merged into a common representation. The framework is combined with an exhaustive feature coding framework, where a compressed representation of the local binary ORB features is transmitted to the centralized SLAM instance. 

**Authors:**  
Dominik Van Opdenbosch (dominik dot van-opdenbosch at tum dot de) and Eckehard Steinach   
Chair of Media Technology, Technical University of Munich, 2018


# 2. Related Publications


[1] **A Joint Compression Scheme for Local Binary Feature Descriptors and their Corresponding Bag-of-Words Representation**  
D. Van Opdenbosch, M. Oelsch, A. Garcea, and E. Steinbach  
*IEEE Visual Communications and Image Processing (VCIP),* 2017. 

[2] **Efficient Map Compression for Collaborative Visual SLAM**  
D. Van Opdenbosch, T. Aykut, M. Oelsch, N. Alt, and E. Steinbach  
*IEEE Winter Conference on Applications of Computer Vision (WACV),* 2018. 

[3] **Selection and Compression of Local Binary Features for Remote Visual SLAM**  
D. Van Opdenbosch, and E. Steinbach  
*IEEE International Conference on Image Processing (ICIP),* 2018. 

[4] **Collaborative Visual SLAM using Compressed Feature Exchange**  
D. Van Opdenbosch, and E. Steinbach  
*IEEE Robotics and Automation Letters,* 2018. 

[5] **Flexible Rate Allocation for Binary Feature Compression**  
D. Van Opdenbosch, M. Oelsch, A. Garcea, and E. Steinbach  
*IEEE Visual Communications and Image Processing (VCIP),* 2018. 



# 3. License
The ORB-SLAM2 collaborative extension is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2/) and is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl.html). A list of known code dependencies with their respective licenses is noted in the Dependencies.md file. It uses the feature compression library which has further dependencies.

If you use our approach in an academic work, please cite:

	@article{VanOpdenbosch2018,
		author = {{Van Opdenbosch}, Dominik and Steinbach, Eckehard},
		journal = {IEEE Robotics and Automation Letters (RAL)},
		title = {{Collaborative Visual SLAM using Compressed Feature Exchange}},
		year = {2018}
	}



# 4. Prerequisites

**This is research code.** We think that the code might be useful for other research groups, but we want to emphasize that the code comes *without any warranty* and *no guarantee* to work in your setup. In consequence we can not promise *any support*. However, we would be glad to hear from you if you use this code, fix bugs or improve the implementation.


**You need a very powerful machine.** The processing capabilities have a significant influence on the mapping results. Either pre-record the bitstream from the agents using `rosbag record` or run the agents and the server on different computers. 

**Provide many cores.** It is recommended that the server has around *n* x (4 + 4) + 1 threads available for *n* agents. The feature decoding uses 4 threads by default (configurable) and ORB-SLAM2 runs the tracking, the mapping, the loop closing and the viewer in separate threads. Additionally, the map merging runs in a separate thread. If maps are merged then the number of threads reduces subsequently. 

**Turn on optimizations.** Make sure that everything is compiled in `release` mode with all optimizations `-O3` and `-march=native` activated. Turn off the viewer if not needed as this requires additional resources. 


We have tested the library using Ubuntu **14.04, 16.04**, but it should be easy to compile on other platforms. This version of ORB-SLAM2 has been modified to work with the feature compression. We have not tested the standalone monocular, stereo and depth visual SLAM examples included in ORB-SLAM2. 

# 5. Installation 

Install the dependencies:  

```
sudo apt-get install libglew-dev libboost-all-dev libopencv-dev build-essential cmake cmake-gui libeigen3-dev
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*.  

```
cd collab_orb_slam2
chmod +x build.sh
./build.sh
```

If you want to use ROS to run the compression make sure to have *ROS* installed and added the *Examples/ROS* path to your *ROS_PACKAGE_PATH*: 

```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:YOUR_PATH_TO/collab_orb_slam2/Examples/ROS
```

For your convenience, you can append it to your *.bashrc* in Ubuntu to set the path permanently:

```
echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:YOUR_PATH_TO/collab_orb_slam2/Examples/ROS' >> ~/.bashrc 
source ~/.bashrc
```

We then provide an additional script `build_ros_compress.sh` to build the *ROS* component: 

```
chmod +x build_ros_compress.sh
./build_ros_compress.sh
```

# 6. Run the examples

We provide two simple examples of how to run the feature compression and collaborative mapping using 1) depth coding and 2) stereo coding. Every agent has a unique identifier. The bitstream will be published as `/featComp/bitstreamN`, where *N* is the agent id. Please note that the configuration for the server is fixed in the source code for agent 0 running KITTI 00 and agent 1 running KITTI 07. 


#### Depth coding

**Agent:**

```
./Examples/ROS/compression/KittiAgentDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER -r AGEND_ID -s  Examples/ROS/compression/KITTIX.yaml
```

**Server:**

```
./Examples/ROS/compression/KittiServerDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```

#### Stereo coding

**Agent:**

```
./Examples/ROS/compression/KittiAgentStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER -r AGEND_ID -s  Examples/ROS/compression/KITTIX.yaml
```

**Server:**

```
./Examples/ROS/compression/KittiServerStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```


# 7. FAQ: 

**1) Check the coding parameters:**  

Currently, the encoder configuration such as the coding modes, number of reference frames etc. is **not** signaled in the bitstream. This means, the decoder has to be manually configured to the same number of reference frames, otherwise it might crash. 

**2) Start the server first:**  

Make sure that the server is up and running before starting the agents. Especially if you are using inter frame coding, the server is not able to decode the features if it misses the first messages. 

**3) Initialize the agents at the server:**

Make sure that for every agent id you call the `SLAM->InitAgent()` at the server side before receiving the first information.

**4) Is the depth uncertainity from the quantization scheme included in the optimization:**

Not yet. This is an open topic for future research. 