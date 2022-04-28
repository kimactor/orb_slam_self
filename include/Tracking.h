#pragma once
#include <iostream>
#include "Frame.h"
#include "ORBextractor.h"
#include "ORBVocabulary.h"
#include <string>
#include "System.h"

namespace ORB_SLAM2 {
	class System;

	class Tracking {

	public:
		Tracking(System* mpSys, const std::string &strSettingPath, const int sensor);

		cv::Mat GrabImageMonocular(const cv::Mat &im, double timestamp);

	public:
		enum eTrackingState {
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			OK = 2,
			LOST = 3
		};

		eTrackingState mState;
		eTrackingState mLastProcessedState;

		Frame mCurrentFrame;


	private:
		void Track();


	protected:
		ORBextractor* mpORBextractorLeft;
		ORBextractor* mpIniORBextractor;

		ORBVocabulary* mpORBVocabulary;

		System* mpSystem;


	private:

		int mSensor;

		cv::Mat mImGray;


		cv::Mat mK;
		cv::Mat mDistCoef;
		float mbf;

		int mMinFrames;
		int mMaxFrames;

		float mThDepth;


		bool mbRGB;
		

	
	
	};

}