/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <ros/package.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <human_aware_navigation/TaskFinished.h>

#include <utilities/printer.hpp>

using namespace ros;

class KeyboardManager
{
public:

	KeyboardManager ();
	virtual ~KeyboardManager (){};

	void run();

private:

	NodeHandle                             m_nd;
	Publisher							   m_pub_TaskFinished;

	human_aware_navigation::TaskFinished   tf;

	bool kbhit();
	void enable_raw_mode();
	void disable_raw_mode();
};