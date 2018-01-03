#ifndef UTIL_LIB_ASSERT_H_
#define UTIL_LIB_ASSERT_H_

#include <iostream>

#define ASSERT(condition, message)											\
	do { 																	\
		if (!(condition)) 													\
		{ 																	\
			std::cerr << "Assertion `" #condition "` failed in " << __FILE__\
					  << ": " << __LINE__ << ": " << message << std::endl; 	\
			std::terminate(); 												\
		} 																	\
	} while (false);

#endif