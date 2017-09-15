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
		-5,	-2,	+0.2,
		-5,	+2,	+0.2,
		+5,	-2,	+0.2,
		+5,	+2,	+0.2,
	};
	int indices[6] =
	{
		0, 1, 2,
		2, 1, 3,
	};

	// initial data

    initPlugin (gridSize, offset, scale, windowSize[0], windowSize[1], vertices, 4, indices, 6);

	float origin[] = { 0, 0, 0 };
	//float direction[] = { 0, -1, 0 };

	float camParameters[] =
	{
		1.0f, 0.0f, 0.0f,	// right
		0.0f, 1.0f, 0.0f,	// up
		0.0f, 0.0f, 1.0f,	// forward
		45.5f,				// fovy
		4.0f/3.0f,			// camera aspect
	};

	float *distances = new float[gridSize * gridSize * gridSize];

	// test clock
	float mousePos[] = { 1.0f, 2.0f };
	float * mousePickupResult = new float[4];
	mousePickup(origin, camParameters, mousePos, mousePickupResult);
	cout << "mouse pick up " << mousePickupResult[0] << " " << mousePickupResult[1] << " " << mousePickupResult[2] << " distance " << mousePickupResult[3] << endl;

	int nrTimes = 5;
	for (int i = 0; i < nrTimes; i++) {
		test(origin, camParameters, distances);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	delete[] distances;
	delete[] mousePickupResult;


    finishPlugin ();

    cout << "done" << endl;

	return 0;
}
