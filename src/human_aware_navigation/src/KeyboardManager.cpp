/***************************** Made by Duarte Gonçalves *********************************/

#include "human_aware_navigation/KeyboardManager.hpp"


KeyboardManager::KeyboardManager(){
	m_pub_TaskFinished = m_nd.advertise<human_aware_navigation::TaskFinished>("/task_finished", 1, true);
}

void KeyboardManager::run(){

	ros::Rate loop_rate(1);
	
	while(ros::ok()){

		enable_raw_mode();

		if (kbhit()){

			char key = getchar();

			switch(key){
				case '1':
				//printer::printGreen("Key 1 pressed");
				tf.task_id = 1;
				break;

				case '2':
				//printer::printGreen("Key 2 pressed");
				tf.task_id = 2;
				break;

				case '3':
				//printer::printGreen("Key 3 pressed");
				tf.task_id = 3;
				break;
			}

			m_pub_TaskFinished.publish(tf);
		}

		disable_raw_mode();

		tcflush(0, TCIFLUSH); // Clear stdin to prevent characters appearing on prompt


		ros::spinOnce();
		loop_rate.sleep();
	}
}


void KeyboardManager::enable_raw_mode(){
	termios term;
	tcgetattr(0, &term);
    term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
    tcsetattr(0, TCSANOW, &term);
}

void KeyboardManager::disable_raw_mode(){
	termios term;
	tcgetattr(0, &term);
	term.c_lflag |= ICANON | ECHO;
	tcsetattr(0, TCSANOW, &term);
}

bool KeyboardManager::kbhit(){
	int byteswaiting;
	ioctl(0, FIONREAD, &byteswaiting);
	return byteswaiting > 0;
}
