//
// Created by maxjin on 2022/4/25.
//

#ifndef MONO_ORB_SLAM_SYSTEM_H
#define MONO_ORB_SLAM_SYSTEM_H

#define SYSTEM_EXPORT __declspec(dllexport)
//#if defined(_WIN32)
//	#if defined( BUILD_DLL)
//		#define SYSTEM_EXPORT __declspec(dllexport)
//	#elif defined( USE_DLL)
//		#define SYSTEM_EXPORT __declspec(dllimport)
//	#else
//		#define SYSTEM_EXPORT
//	#endif
//#else
//	#define SYSTEM_EXPORT
//#endif

#include <iostream>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Tracking.h>

namespace ORB_SLAM2
{
	class Tracking;

	using namespace std;
	class SYSTEM_EXPORT System {
	public:
		enum eSensor {
			MONOCULAR = 0,
			STEREO = 1,
			RGBD=2
		};

	public:
		typedef std::shared_ptr<System> Ptr;
		System(const string vstrVocFile, const string strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

		cv::Mat TrackMonocular(const cv::Mat& im, const double& timestamp);

		void ShutDown();

		void SaveTrajectoryTUM(const string &filename);

		void StartViewer();



	private:
		eSensor mSensor;
		mutex mMutexMode;
		
		Tracking* mpTracker;




	};


}




#endif //MONO_ORB_SLAM_SYSTEM_H
