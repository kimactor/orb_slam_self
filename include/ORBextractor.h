#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM2 
{

	class ExtractorNode
	{
	public:
		ExtractorNode() :bNoMore(false) {}

		void DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);

		std::vector<cv::KeyPoint> vKeys;
		cv::Point2i UL, UR, BL, BR;
		std::list<ExtractorNode>::iterator lit;
		bool bNoMore;
	};

	class ORBextractor {

	public:
		enum {HARRIS_SCORE=0,
		FAST_SCORE=1};

		typedef std::shared_ptr<ORBextractor> Ptr;
		ORBextractor(int nFeatures, double fScaleFactor, int nLevels, int fIniThFAST, int fMinThFAST);
		~ORBextractor(){}

		void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keyPoints,
			cv::OutputArray descriptors);

	private:
		void ComputePyramid(cv::Mat image);
		void ComputeKeyPointsOctree(std::vector<std::vector<cv::KeyPoint>> & allKeypoints);
		std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int& minX,
			const int& maxX, const int& minY, const int& maxY, const int& N, const int& level);
	private:
		std::vector<cv::Point> pattern;
		std::vector<cv::Mat> mvImagePyramid;

		int nfeatures;
		double scaleFactor;
		int nlevels;
		int iniThFAST;
		int minThFAST;

		std::vector<int> mnFeaturesPerLevel;
		std::vector<int> umax;
	
		std::vector<float> mvScaleFactor;
		std::vector<float> mvInvScaleFactor;
		std::vector<float> mvLevelSigma2;
		std::vector<float> mvInvLevelSigma2;

	};



}