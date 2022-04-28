#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include "ORBextractor.h"
#include "ORBVocabulary.h"


namespace ORB_SLAM2
{
	class Frame {
	
	public:
		typedef std::shared_ptr<Frame> Ptr;

		Frame();

		Frame(const Frame& frame);

		Frame(const cv::Mat& imLeft, const double& timeStamp, ORBextractor* extractor, cv::Mat K, cv::Mat DistCoef, const float& bf, const float& thDepth);
	
		void ExtractORB(int flag, const cv::Mat& im);

	public:
		cv::Mat mTcw;

		ORBextractor* mpORBextractorLeft;
		double mTimestamp;

		cv::Mat mK;
		static float fx;
		static float fy;
		static float cx;
		static float cy;
		static float invfx;
		static float invfy;

		cv::Mat mDistCoef;
		float mbf;
		float mb;
		float mThDepth;

		int N;

		std::vector<cv::KeyPoint> mvKeys;
		std::vector<cv::KeyPoint> mvKeysUn;

		cv::Mat mDescriptors;


		static long unsigned int nNextId;
		long unsigned int mnId;
	
	};


}