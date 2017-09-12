#include <iostream>
#include <chrono>
#include <thread>

#include <voxel_visibility_checker.h>

using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char * argv[])
{
    cout << "pluing testing app" << endl;

	int		gridSize		= 32;
	float	offset[]		= { 0, 0, 0 };
	float	scale[]			= { 1, 1, 1 };
	int		windowSize[]	= { 800, 600 };

	// send a quad (2 triangles, 4 vertices)
	float vertices[12] =
	{
		-10, -2, -10,
		-10, -2, +10,
		+10, -2, -10,
		+10, -2, +10,
	};
	float indices[6] =
	{
		0, 2, 1,
		1, 2, 3,
	};

	// initial data

    initPlugin (gridSize, offset, scale, windowSize[0], windowSize[1], vertices, 4, indices, 6);

	float origin[] = { 0, 0, 0 };
	float direction[] = { 0, -1, 0 };

	bool *result = new bool[5];
	int *result3 = new int[5];

	int nrTimes = 5;
	for (int i = 0; i < nrTimes; i++) {
		test(origin, direction, result, result3);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	delete[] result;
	delete[] result3;


    finishPlugin ();

    cout << "done" << endl;

	return 0;
}
