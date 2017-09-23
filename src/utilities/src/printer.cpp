/***************************** Made by Duarte Gonçalves *********************************/

#include "utilities/printer.hpp"

namespace printer
{
	void printBlack(std::string text){
		std::cout << BLACK << text << RESET << std::endl;
	}

	void printRed(std::string text){
		std::cout << RED << text << RESET << std::endl;
	}

	void printGreen(std::string text){
		std::cout << GREEN << text << RESET << std::endl;
	}

	void printYellow(std::string text){
		std::cout << YELLOW << text << RESET << std::endl;
	}

	void printBlue(std::string text){
		std::cout << BLUE << text << RESET << std::endl;
	}

	void printMagenta(std::string text){
		std::cout << MAGENTA << text << RESET << std::endl;
	}

	void printCyan(std::string text){
		std::cout << CYAN << text << RESET << std::endl;
	}

	void printWhite(std::string text){
		std::cout << WHITE << text << RESET << std::endl;
	}

	void clear(){
		std::cout << CLEAR << std::endl;
	}

}