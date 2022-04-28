#include "Frame.h"


namespace ORB_SLAM2 {

	long unsigned int Frame::nNextId = 0;
	float Frame::fx, Frame::fy, Frame::cx, Frame::cy, Frame::invfx, Frame::invfy;

	Frame::Frame()
	{}

	Frame::Frame(const Frame& frame)
	{

	}

	Frame::Frame(const cv::Mat&imGray, const double& timeStamp, ORBextractor* extractor, cv::Mat K, cv::Mat DistCoef, const float& bf, const float& thDepth)
		:mpORBextractorLeft(extractor), mTimestamp(timeStamp), mK(K.clone()), mDistCoef(DistCoef.clone()), mbf(bf), mThDepth(thDepth)
	{
		mnId = nNextId++;

		ExtractORB(0, imGray);


		
	}

	void Frame::ExtractORB(int flag, const cv::Mat& imGray)
	{
		if (flag == 0)
			(*mpORBextractorLeft)(imGray, cv::Mat(), mvKeys, mDescriptors);
	}

}
