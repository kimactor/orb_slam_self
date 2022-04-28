//
// Created by maxjin on 2022/4/25.
//

#include "../include/System.h"

namespace ORB_SLAM2
{
	System::System(const string strVocFile, const string strSettingsFile, const eSensor sensor, 
		const bool bUseViewer):mSensor(sensor)
	{
		cout << "Input sensor was set to: ";
		if (mSensor == MONOCULAR)
			cout << "Monocular" << endl;
		else if (mSensor == STEREO)
			cout << "Stereo" << endl;
		else if (mSensor == RGBD)
			cout << "RGB-D" << endl;

		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << " Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}

		cout << endl << " Load ORB vocabulary, This could take a while ..." << endl;
		//todo load vocabulary

		mpTracker = new Tracking(this, strSettingsFile, (int)mSensor);

		
	}


	
	cv::Mat System::TrackMonocular(const cv::Mat& im, const double& timestamp)
	{
		if (mSensor != MONOCULAR)
		{
			cerr << " ERROR: you called TrackMonocular but input sensor was not set to Monocular " << endl;
			exit(-1);
		}

		//{
		//	unique_lock<mutex> lock(mMutexMode);
		//	
		//	
		//}
		return mpTracker->GrabImageMonocular(im, timestamp);


	}

	void System::ShutDown()
	{

	}

	void System::StartViewer()
	{

	}
}

