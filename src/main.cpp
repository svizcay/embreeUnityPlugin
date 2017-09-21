#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <limits>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

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
		-1.5f,	-0.5f,	+0.0,
		-0.5f,	+0.5f,	+0.0,
		-0.5f,	-0.5f,	+0.0,
		-1.5f,	+0.5f,	+0.0,
	};
	int indices[6] =
	{
		// clockwise
		0, 1, 2,
		1, 0, 3,
		// counter clockwise
		//0, 1, 3,
		//0, 2, 1,
	};

	// initial data

    initPlugin (gridSize, offset, scale, windowSize[0], windowSize[1], vertices, 4, indices, 6);

	float origin[] = { 0, 0, -2 };
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
	float mousePos[] = { 34.0f, 302.0f };
	float * mousePickupResult = new float[4];
	mousePickup(origin, camParameters, mousePos, mousePickupResult);
	cout << "mouse pick up " << mousePickupResult[0] << " " << mousePickupResult[1] << " " << mousePickupResult[2] << " distance " << mousePickupResult[3] << endl;

	int nrTimes = 5;
	for (int i = 0; i < nrTimes; i++) {
		test(origin, camParameters, distances);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	float * depthmapResult = new float[windowSize[0] * windowSize[1]];

	depthMap(origin, camParameters, depthmapResult);
	depthMap(origin, camParameters, depthmapResult);


	float minValue = std::numeric_limits<float>::infinity();
	float maxValue = minValue * -1;

	for (int i = 0; i < windowSize[0] * windowSize[1]; i++) {
		if (depthmapResult[i] < minValue) minValue = depthmapResult[i];
		if (depthmapResult[i] > maxValue) maxValue = depthmapResult[i];
	}

	cout << "min and max depths: " << minValue << " " << maxValue << endl;


	delete[] distances;
	delete[] mousePickupResult;
	delete[] depthmapResult;


    finishPlugin ();


	///*
	//* working example of soil image image library
	//*/
	//const int comp = 3;	// RGB
	//int quality = 80;
	//const int imgWidht = 200;
	//const int imgHeight = 100;
	//// origin top left
	////float imgdata[] =
	////{
	////	1.0f, 0.0f, 0.0f,	// pixel 1
	////	0.7f, 0.0f, 0.0f,
	////	0.3f, 0.0f, 0.0f,

	////	0.0f, 1.0f, 0.0f,
	////	0.0f, 0.7f, 0.0f,
	////	0.0f, 0.3f, 0.0f,
	////};
	//unsigned char imgdata[imgWidht * imgHeight * comp];
	//unsigned char color = 255;
	//cout << "color: " << (unsigned) color << endl;
	//for (int y = 0; y < imgHeight; y++) {
	//	for (int x = 0; x < imgWidht; x++) {
	//		//imgdata[(y * imgWidht + x) * comp + 0] = 1.0f;
	//		//imgdata[(y * imgWidht + x) * comp + 1] = 1.0f;
	//		//imgdata[(y * imgWidht + x) * comp + 2] = 1.0f;
	//		imgdata[(y * imgWidht + x)*comp + 0] = (unsigned char) ((imgWidht - x) / (float) imgWidht * 255);		// red channel
	//		imgdata[(y * imgWidht + x)*comp + 1] = (unsigned char) ((imgHeight - y) / (float) imgHeight * 255);		// green channel
	//		imgdata[(y * imgWidht + x)*comp + 2] = 0;		// blue channel
	//	}
	//}
	//int status = stbi_write_jpg("test.jpg", imgWidht, imgHeight, comp, imgdata, quality);
	//if (status == 0) {
	//	cerr << "something went wrong when saving img test.jpg" << endl;
	//}

    cout << "done" << endl;

	return 0;
}
