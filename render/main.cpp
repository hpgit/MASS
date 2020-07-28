#include "Window.h"
#include "Environment.h"
#include "DARTHelper.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
#include <FL/Fl.H>

namespace p = boost::python;
namespace np = boost::python::numpy;
int main(int argc,char** argv)
{
	MASS::Environment* env = nullptr;

	if(argc==1)
	{
		std::cout << "Provide Metadata.txt" << std::endl;
		return 0;
	}

	env = new MASS::Environment();
	env->Initialize(std::string(argv[1]),false);

	Py_Initialize();
	np::initialize();

	MASS::Window* window;
	if(argc == 2)
	{
		window = new MASS::Window(env);
	}
	else
	{
		if(env->GetUseMuscle())
		{
			if(argc < 4 || argc > 5){
				std::cout << "Please provide muscle networks" << std::endl;
				return 0;
			}
			else if(argc == 4) {
				window = new MASS::Window(env, argv[2], argv[3]);
			}
		}
		else
		{
			if(argc < 3 || argc > 4)
			{
				std::cout << "Please provide the network" << std::endl;
				return 0;
			}
			else if(argc == 3) {
				window = new MASS::Window(env, argv[2]);
			}
		}
	}
	// if(argc==1)
	// 	window = new MASS::Window(env);
	// else if (argc==2)
	// 	window = new MASS::Window(env,argv[1]);
	// else if (argc==3)
	// 	window = new MASS::Window(env,argv[1],argv[2]);
	
	// window->initWindow(1920,1080,"gui");
	window->show();
	
	// glutMainLoop();
	return Fl::run();
}
