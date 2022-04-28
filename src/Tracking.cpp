#include "../include/Tracking.h"


namespace ORB_SLAM2 {

	Tracking::Tracking(System* mpSys, const std::string& strSettingPath, const int sensor) :
		mState(NO_IMAGES_YET), mSensor(sensor), mpSystem(mpSys)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];

		cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = fx;
		K.at<float>(1, 1) = fy;
		K.at<float>(0, 2) = cx;
		K.at<float>(1, 2) = cy;
		K.copyTo(mK);

		cv::Mat DistCoef(4, 1, CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if (k3 != 0)
		{
			DistCoef.resize(5);
			DistCoef.at<float>(4) = k3;
		}
		DistCoef.copyTo(mDistCoef);

		mbf = fSettings["Camera.bf"];
		float fps = fSettings["Camera.fps"];

		if (fps == 0)
			fps = 30;

		mMinFrames = 0;
		mMaxFrames = fps;

		cout << endl << "Camera Param: " << endl;
		cout << " - fx : " << fx << endl;
		cout << " - fy : " << fy << endl;
		cout << " - cx : " << cx << endl;
		cout << " - cy : " << cy << endl;
		cout << " - k1 : " << DistCoef.at<float>(0) << endl;
		cout << " - k2 : " << DistCoef.at<float>(1) << endl;
		cout << " - p1 : " << DistCoef.at<float>(2) << endl;
		cout << " - p2 : " << DistCoef.at<float>(3) << endl;
		if (k3 != 0)
			cout << " - k3 : " << DistCoef.at<float>(4) << endl;
		cout << " - fps : " << fps << endl;


		int nRGB = fSettings["Camera.RGB"];
		mbRGB = nRGB;

		if (mbRGB)
			cout << " - color order: RGB " << endl;
		else
			cout << " - color order: BGR " << endl;

		int nFeatures = fSettings["ORBextractor.nFeatures"];
		double fScaleFactor = fSettings["ORBextractor.scaleFactor"];
		int nLevels = fSettings["ORBextractor.nLevels"];
		int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
		int fMinThFAST = fSettings["ORBextractor.minThFAST"];



		//mpORBextractorLeft = new ORBextractor();
		mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		if (sensor == System::MONOCULAR)
			mpIniORBextractor = new ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		cout << endl << " ORB Extractor Parameters; " << endl;
		cout << " - Number of Features: " << nFeatures << endl;
		cout << " - Scale Factor: " << fScaleFactor << endl;
		cout << " - Scale Levels: " << nLevels << endl;
		cout << " - Initial Fast Threshold: " << fIniThFAST << endl;
		cout << " - Minimun Fast threshold: " << fMinThFAST << endl;
	}

	cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, double timestamp)
	{
		mImGray = im;

		if (mImGray.channels() == 3)
		{
			if (mbRGB)
				cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
			else
				cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);			
		}
		else if (mImGray.channels() == 4)
		{
			if (mbRGB)
				cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
			else
				cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
		}

		if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
			mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mK, mDistCoef, mbf, mThDepth);
		else
			mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);

		Track();

		return mCurrentFrame.mTcw.clone();
	}

	void Tracking::Track()
	{

	}


}
