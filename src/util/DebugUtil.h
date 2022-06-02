#ifndef BULLETGAME_SRC_UTIL_DEBUGUTIL_H_
#define BULLETGAME_SRC_UTIL_DEBUGUTIL_H_

#include <iostream>

 class DebugUtil
{
 public:
	static void printVector3(const btVector3 vector, std::string prefix = "")
	{
		std::cout << prefix << "\tx:" << vector.x() << "\ty:" << vector.y() << "\tz:" << vector.z() << std::endl;
	}
};

#endif //BULLETGAME_SRC_UTIL_DEBUGUTIL_H_
