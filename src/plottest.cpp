#include <iostream>
#include <string>
#include "Python.h"
int main() {
	Py_Initialize();
	int x[5] = { 0, 1, 2, 3, 4 };
	int y[5] = { 5, 1, 7, 5, 1 };
	int z[5] = { 9, 8, 7, 6, 5 };

	std::string command = "pylab.plot([";
	for (int i = 0; i < 4; i++) {
		command += std::to_string(x[i]);
		command += ", ";
	}
	command += std::to_string(x[4]);
	command += "], [";
	for (int i = 0; i < 4; i++) {
		command += std::to_string(y[i]);
		command += ", ";
	}
	command += std::to_string(y[4]);
	command += "])";
	std::cout << command;
	PyRun_SimpleString("import pylab");
	PyRun_SimpleString(command.c_str());
	PyRun_SimpleString("pylab.show()");
	Py_Exit(0);
	getchar();
}