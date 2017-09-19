/***************************** Made by Duarte Gonçalves *********************************/

#include <ros/ros.h>
#include <sstream>
#include <string>     // std::string, std::to_string

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

#define CLEAR "\033[2J"  // clear screen escape code 

namespace printer {
	void printBlack(std::string text);
	void printRed(std::string text);
	void printGreen(std::string text);
	void printYellow(std::string text);
	void printBlue(std::string text);
	void printMagenta(std::string text);
	void printCyan(std::string text);
	void printWhite(std::string text);
	void clear();
}