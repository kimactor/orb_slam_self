#pragma once
#include <iostream>

namespace ORB_SLAM2
{
	class ORBVocabulary
	{
	public:
		typedef std::shared_ptr<ORBVocabulary> Ptr;
		ORBVocabulary();
	};

}