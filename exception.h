#ifndef EXCPETION_H
#define EXCPETION_H

class Exception : public std::exception
{
public:
	Exception(const std::string& str) : std::exception(str.c_str()) { }
};

#endif
