#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>	// to pass parameters as reference to working threads
#include <future>
#include <vector>

#include <xmmintrin.h>	// functions for setting the control register
#include <pmmintrin.h>	// functions for setting the control register

#include "voxel_visibility_checker.h"

using std::cout;
using std::cerr;
using std::endl;

extern "C"
{

	struct Task
	{
		unsigned id;

		Task()
		{
			cout << "running Task default constructor" << endl;
		}

		Task(unsigned id_) : id(id_)
		{
			cout << "running Task parametrized constructor" << endl;
		}

	};

	struct ResultPkg
	{
		std::vector<float>	values;
		int					nrItems;

		ResultPkg()
		{
			cout << "ResultPkg default constructor" << endl;
			//nrItems = 0;
		}

		ResultPkg(unsigned _nrItems) : nrItems(_nrItems)
		{
			cout << "ResultPkg parametrized constructor" << endl;
			values.resize(_nrItems);
			//values = new float[_nrItems];

		}

		~ResultPkg()
		{
			//delete[] values;
		}

		void Reset(unsigned newSize)
		{
			//delete[] values;
			//values = new float[newSize];
			nrItems = newSize;
		}
	};

    struct Vertex { float x, y, z, a; };
    struct Triangle { int v0, v1, v2; };

    RTCDevice   rtcDevice;
    RTCScene    rtcScene;
    unsigned    proxyMeshGeomId;

    void embreeErrorCB(RTCError code, const char* msg);

	// parameters
	int		gridSize;
	float	offset[]		= { 0, 0, 0 };
	float	scale[]			= { 1, 1, 1 };
	int		windowSize[]	= { 800, 600 };

	Task currentTask;

	Task *taskPtr;

	unsigned currentTaskId;

	bool initialized = false;

	std::future<ResultPkg> * resultPtr;

	float futureGetResultElapsedTime = 0;
	float restOfCodeElapsedTime = 0;

	void initPlugin(int _gridSize, float _offset[], float _scale[], int _textureWidth, int _textureHeight, float _vertices[], int _nrVertices, float _indices[], int _nrIndices)
    {
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);			// Enable 'Flush Zero' bit
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);	// Enable 'Denormals Zero'bit

		// set static paramters
		gridSize = _gridSize;
		offset[0] = _offset[0];
		offset[1] = _offset[1];
		offset[2] = _offset[2];
		scale[0] = _scale[0];
		scale[1] = _scale[1];
		scale[2] = _scale[2];
		windowSize[0] = _textureWidth;
		windowSize[1] = _textureHeight;

        rtcDevice = rtcNewDevice(nullptr);

        // DEPRECATED. use rtcDeviceSetErrorFunction2
        rtcDeviceSetErrorFunction(rtcDevice, embreeErrorCB);

        rtcScene = rtcDeviceNewScene(rtcDevice, RTC_SCENE_STATIC | RTC_SCENE_COHERENT, RTC_INTERSECT1 /*| RTC_INTERPOLATE*/);

		//unsigned numTriangles = 2;
		//unsigned numVertices = 4;

        // load proxy (nrTriangles = nrIndices / 3
        //proxyMeshGeomId = rtcNewTriangleMesh(rtcScene, RTC_GEOMETRY_STATIC, numTriangles, numVertices);
		proxyMeshGeomId = rtcNewTriangleMesh(rtcScene, RTC_GEOMETRY_STATIC, _nrIndices / 3, _nrVertices);

		Vertex* vertices = (Vertex*)rtcMapBuffer(rtcScene, proxyMeshGeomId, RTC_VERTEX_BUFFER);
		// remember that unity uses clockwise triangle order
		// vertices data pack as following [x1,y1,z1][x2,y2,z2] ..... [xn,yn,zn] and n nrVertices
		for (int i = 0; i < _nrVertices; i++) {
			vertices[i].x = _vertices[i*3+0];
			vertices[i].y = _vertices[i*3+1];
			vertices[i].z = _vertices[i*3+2];
			//vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
			//vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
			//vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
		}
		rtcUnmapBuffer(rtcScene, proxyMeshGeomId, RTC_VERTEX_BUFFER);


		Triangle* triangles = (Triangle*)rtcMapBuffer(rtcScene, proxyMeshGeomId, RTC_INDEX_BUFFER);
		// indices data pack as following
		for (int i = 0; i < _nrIndices / 3; i++) {
			triangles[i].v0 = _indices[i*3+0];
			triangles[i].v1 = _indices[i*3+1];
			triangles[i].v2 = _indices[i*3+2];
			//triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
			//triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
		}
		rtcUnmapBuffer(rtcScene, proxyMeshGeomId, RTC_INDEX_BUFFER);





        // commit changes to the scene (all buffers have to get unmapped before commiting)
        rtcCommit(rtcScene);

		currentTaskId = 0;

		taskPtr = new Task(5);

		initialized = true;

		futureGetResultElapsedTime = 0;
		restOfCodeElapsedTime = 0;
    }

	ResultPkg calculateValue(float origin[], float viewMatrix[])
	{
		cout << "inside calculateValue with currentTaskId=" << currentTaskId << endl;

		// prepare ray
		RTCRay ray;
		ray.org[0] = origin[0];
		ray.org[1] = origin[1];
		ray.org[2] = origin[2];
		// calculate direction by using viewMatrix (right, forward, up)
		ray.dir[0] = viewMatrix[0];
		ray.dir[1] = viewMatrix[1];
		ray.dir[2] = viewMatrix[2];

		ray.tnear = 0;
		ray.tfar = std::numeric_limits<float>::infinity();
		ray.geomID = RTC_INVALID_GEOMETRY_ID;

		ResultPkg result (windowSize[0] * windowSize[1]);

		for (int i = 0; i < windowSize[0]; i++) {
			for (int j = 0; j < windowSize[1]; j++) {
				rtcIntersect(rtcScene, ray);
				result.values[i*windowSize[1] + j] = ray.tfar;
			}
		}

		cout << "distance : " << ray.tfar << endl;

		return result;
	}

    void test (float origin[], float viewMatrix[], int * result)
    {

		if (initialized) {
			auto start = std::chrono::steady_clock::now();

			ResultPkg calculatedResult (windowSize[0] * windowSize[1]);

			if (currentTaskId == 0) {
				// return dummy value
				calculatedResult.nrItems = -1;
			}
			else {
				// return value calculated at the previous call
				auto getResultStart = std::chrono::steady_clock::now();
				calculatedResult = resultPtr->get();
				auto getResultEnd = std::chrono::steady_clock::now();

				float getResultDeltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(getResultEnd - getResultStart).count();	// in ms
				futureGetResultElapsedTime += getResultDeltaTime;

				delete resultPtr;
			}

			if (calculatedResult.values.size() > 0) {
				cout << "returning previous value: " << calculatedResult.nrItems << " " << calculatedResult.values[0] << endl;
			}
			else {
				cout << "returning previous value: " << calculatedResult.nrItems << " " << calculatedResult.values.size() << endl;
			}

			resultPtr = new std::future<ResultPkg>(std::async(calculateValue, origin, viewMatrix));



			// throw ray using a different thread
			//std::thread workingThread(throwRay, std::ref(ray));
			//workingThread.join();

			for (int i = 0; i < 5; i++) {
				*(result + i) = i;
			}

			auto end = std::chrono::steady_clock::now();

			float restOfCodeDeltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();	// in ms
			restOfCodeElapsedTime += restOfCodeDeltaTime;

			//cout << "elapsed time: " << restOfCodeDeltaTime << "ms." << endl;

			currentTaskId++;
		}
    }

	void throwRay(RTCRay & ray)
	{
		for (int i = 0; i < windowSize[0]; i++) {
			for (int j = 0; j < windowSize[1]; j++) {
				rtcIntersect(rtcScene, ray);
			}
		}
	}

    void finishPlugin ()
    {
		// ask for last result (otherwise it will crash)
		ResultPkg calculatedResult(windowSize[0] * windowSize[1]);
		calculatedResult  = resultPtr->get();

		cout << "elapsed time: " << restOfCodeElapsedTime << endl;
		cout << "elapsed time waiting for the result: " << futureGetResultElapsedTime << endl;

        rtcDeleteScene(rtcScene);

        rtcDeleteDevice(rtcDevice);

		delete taskPtr;
		delete resultPtr;

		initialized = false;
    }

    void embreeErrorCB(RTCError code, const char* msg)
    {
        std::string err;

        switch (code)
        {
        case RTC_UNKNOWN_ERROR: err = std::string("RTC_UNKNOWN_ERROR"); break;
        case RTC_INVALID_ARGUMENT: err = std::string("RTC_INVALID_ARGUMENT"); break;
        case RTC_INVALID_OPERATION: err = std::string("RTC_INVALID_OPERATION"); break;
        case RTC_OUT_OF_MEMORY: err = std::string("RTC_OUT_OF_MEMORY"); break;
        case RTC_UNSUPPORTED_CPU: err = std::string("RTC_UNSUPPORTED_CPU"); break;
        case RTC_CANCELLED: err = std::string("RTC_CANCELLED"); break;
        default: err = std::string("invalid error code"); break;
        }

        cerr << "embree reported the following issue - "
            << "[" << err << "]'" << msg << "'" << std::endl;
    }
}
