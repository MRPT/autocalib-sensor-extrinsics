/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <pcl/console/parse.h>
#include <X11/Xlib.h>

using namespace std;
//using namespace rv::obs;
//using namespace rv::sensor;

void print_help(char ** argv)
{
	cout << "\nThis program loads a dataset and ....";
	cout << "  usage: " <<  argv[0] << " <file_name> \n";
	cout << "            <file_name> path to the dataset file" << endl;
	cout << argv[0] << " -h | --help : shows this help" << endl;
}


/*! This program loads a dataset and .... */
int main (int argc, char ** argv)
{
	try
	{
		if(argc < 2 || pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
		{
			print_help(argv);
			return 0;
		}
		if(argc > 3)
			cout << "\t WARNING: This program accepts only 1 argument (i.e. dataset filenames), following arguments in the command line are ignored! \n";

		string file_name = static_cast<string>(argv[1]);
		cout << "file_name : " << file_name << endl;

		Display* d = XOpenDisplay(NULL);
		Screen*  s = DefaultScreenOfDisplay(d);
		cout << "display window size" << s->width/2 << " " <<  s->height/2 << "\n\n";
		cout << "EXIT\n\n";
		return 1;

	} 
    catch (std::exception &e)
	{
		std::cout << " exception caught: " << e.what() << std::endl;
		return -1;
	}
    catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
