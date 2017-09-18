#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <functional>	// to pass parameters as reference to working threads
#include <future>
#include <vector>

#include <xmmintrin.h>	// functions for setting the control register
#include <pmmintrin.h>	// functions for setting the control register

#include "voxel_visibility_checker.h"

#define DEG2RAD 0.0174532924f

using std::cout;
using std::cerr;
using std::endl;

extern "C"
{

	struct ResultPkg
	{
		std::vector<float>	values;
		int					nrItems;

		ResultPkg()
		{
			//cout << "ResultPkg default constructor" << endl;
			//nrItems = 0;
		}

		ResultPkg(unsigned _nrItems) : nrItems(_nrItems)
		{
			//cout << "ResultPkg parametrized constructor" << endl;
			values.resize(_nrItems, -1.0f);
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

	bool initialized = false;

	std::future<ResultPkg> * resultPtr;

	float futureGetResultElapsedTime = 0;
	float restOfCodeElapsedTime = 0;

	bool firstExecution = true;

	void initPlugin(int _gridSize, float _offset[], float _scale[], int _textureWidth, int _textureHeight, float _vertices[], int _nrVertices, int _indices[], int _nrIndices)
    {

		//cerr << "[begin initPlugin]" << endl;
		debugInUnity("[begin initPlugin]");

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

		//cerr << "[begin initPlugin] rtcDevice value before assignment" << rtcDevice << endl;
		//debugInUnity("[begin initPlugin] before rtcNewDevice");
        rtcDevice = rtcNewDevice(nullptr);
		//cerr << "[begin initPlugin] rtcDevice value after assignment" << rtcDevice << endl;
		//debugInUnity("[begin initPlugin] after rtcNewDevice");

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

		initialized = true;

		futureGetResultElapsedTime = 0;
		restOfCodeElapsedTime = 0;

		firstExecution = true;
		//cerr << "[end initPlugin]" << endl;
		debugInUnity("[end initPlugin]");


    }

	ResultPkg calculateValue(float origin[], float camParameters[])
	{
		// compute pixel derivatives
		float right[] = 
		{
			camParameters[0], camParameters[1], camParameters[2]
		};

		float up[] =
		{
			camParameters[3], camParameters[4], camParameters[5]
		};

		float forward[] =
		{
			camParameters[6], camParameters[7], camParameters[8]
		};

		float fovy = camParameters[9];				// in degrees
		float cameraAspect = camParameters[10];

		float heightInWS = 2.0f * tanf(fovy * DEG2RAD / 2.0f);
		float widthInWS = heightInWS * cameraAspect;

		float rowSize[] =
		{
			right[0] * widthInWS,
			right[1] * widthInWS,
			right[2] * widthInWS,
		};

		float colSize[] =
		{
			up[0] * heightInWS,
			up[1] * heightInWS,
			up[2] * heightInWS,
		};

		float dx[] =
		{
			rowSize[0] / (float) (windowSize[0]),
			rowSize[1] / (float) (windowSize[0]),
			rowSize[2] / (float) (windowSize[0]),
		};

		float dy[] =
		{
			colSize[0] / (float)(windowSize[1]),
			colSize[1] / (float)(windowSize[1]),
			colSize[2] / (float)(windowSize[1]),
		};

		float topLeftOffset[] =
		{
			forward[0] - rowSize[0] / 2.0f - colSize[0] / 2.0f,
			forward[1] - rowSize[1] / 2.0f - colSize[1] / 2.0f,
			forward[2] - rowSize[2] / 2.0f - colSize[2] / 2.0f,
		};

		topLeftOffset[0] = topLeftOffset[0] + origin[0];
		topLeftOffset[1] = topLeftOffset[1] + origin[1];
		topLeftOffset[2] = topLeftOffset[2] + origin[2];

		ResultPkg result(gridSize * gridSize * gridSize);

		for (int i = 0; i < windowSize[0]; i++) {
			for (int j = 0; j < windowSize[1]; j++) {
				//  TODO: check if i can reuse the same RTCRay datastructure

				// calculate target world pos
				float target[] =
				{
					dx[0] * i + dy[0] * j + topLeftOffset[0],
					dx[1] * i + dy[1] * j + topLeftOffset[1],
					dx[2] * i + dy[2] * j + topLeftOffset[2],
				};

				// prepare ray
				RTCRay ray;
				ray.org[0] = origin[0];
				ray.org[1] = origin[1];
				ray.org[2] = origin[2];
				// calculate direction by using viewMatrix (right, forward, up)
				ray.dir[0] = target[0] - origin[0];
				ray.dir[1] = target[1] - origin[1];
				ray.dir[2] = target[2] - origin[2];

				ray.tnear = 0;
				ray.tfar = std::numeric_limits<float>::infinity();
				ray.geomID = RTC_INVALID_GEOMETRY_ID;

				// throw ray
				rtcIntersect(rtcScene, ray);

				// check if ray hit something
				if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
					// normalize direction vector and get hit point
					float magnitude = sqrtf(ray.dir[0] * ray.dir[0] + ray.dir[1] * ray.dir[1] + ray.dir[2] * ray.dir[2]);
					float hitPoint[] = 
					{
						origin[0] + ray.dir[0] * ray.tfar / magnitude,
						origin[1] + ray.dir[1] * ray.tfar / magnitude,
						origin[2] + ray.dir[2] * ray.tfar / magnitude,
					};

					// transform from world space to voxel space
					float voxel[] = 
					{
						(hitPoint[0] - offset[0]) * scale[0],
						(hitPoint[1] - offset[1]) * scale[1],
						(hitPoint[2] - offset[2]) * scale[2],
					};

					unsigned voxelId = (int)( (int)voxel[0] * gridSize * gridSize + (int)voxel[1] * gridSize + (int)voxel[2]);

					if (voxelId >= 0 && voxelId < (gridSize * gridSize * gridSize)) {
						result.values[voxelId] = ray.tfar;
					}

				}
			}
		}

		return result;
	}


	/*
	* params:
	vec3 origin
	camParameters:
		vec3 cameraRight
		vec3 cameraUp
		vec3 cameraForward
		float fovy
		float cameraAspect
	vec2 screenPos
	len(results) = 4
	results[0:2] = ray direction
	reuslts[3] = hit distance (-1 if none)
	*/
	void mousePickup(float origin[], float camParameters[], float screenPos[], float * results)
	{
		if (initialized) {
			// unpack camera parameters
			float right[] =
			{
				camParameters[0], camParameters[1], camParameters[2]
			};

			float up[] =
			{
				camParameters[3], camParameters[4], camParameters[5]
			};

			float forward[] =
			{
				camParameters[6], camParameters[7], camParameters[8]
			};

			//printVec3(right, "right");
			//printVec3(up, "up");
			//printVec3(forward, "forward");

			//printVec3InUnity(right, "right");
			//printVec3InUnity(up, "up");
			//printVec3InUnity(forward, "forward");


			float fovy = camParameters[9];				// in degrees

			//cout << "field of view: " << fovy << endl;
			//debugInUnity("field of view: " + std::to_string(fovy));

			float cameraAspect = camParameters[10];

			//cout << "camera aspect: " << cameraAspect << endl;
			//debugInUnity("camera aspect: " + std::to_string(cameraAspect));

			// end unpacking camera parameters

			// calculate ray parameters
			float heightInWS = 2.0f * tanf(fovy * DEG2RAD / 2.0f);
			float widthInWS = heightInWS * cameraAspect;

			//cout << "width and height in word space of the camera plane (1 unit away of the camera pos): " << widthInWS << " " << heightInWS << endl;

			float rowSize[] =
			{
				right[0] * widthInWS,
				right[1] * widthInWS,
				right[2] * widthInWS,
			};

			float colSize[] =
			{
				up[0] * heightInWS,
				up[1] * heightInWS,
				up[2] * heightInWS,
			};

			float dx[] =
			{
				rowSize[0] / (float)(windowSize[0]),
				rowSize[1] / (float)(windowSize[0]),
				rowSize[2] / (float)(windowSize[0]),
			};

			float dy[] =
			{
				colSize[0] / (float)(windowSize[1]),
				colSize[1] / (float)(windowSize[1]),
				colSize[2] / (float)(windowSize[1]),
			};

			float topLeftOffset[] =
			{
				forward[0] - rowSize[0] / 2.0f - colSize[0] / 2.0f,
				forward[1] - rowSize[1] / 2.0f - colSize[1] / 2.0f,
				forward[2] - rowSize[2] / 2.0f - colSize[2] / 2.0f,
			};

			//cout << "top left offset before adding origin: " << topLeftOffset[0] << " " << topLeftOffset[1] << " " << topLeftOffset[2] << endl;
			//printVec3InUnity(topLeftOffset, "top left offset before adding origin");


			topLeftOffset[0] = topLeftOffset[0] + origin[0];
			topLeftOffset[1] = topLeftOffset[1] + origin[1];
			topLeftOffset[2] = topLeftOffset[2] + origin[2];

			//cout << "top left offset after adding origin: " << topLeftOffset[0] << " " << topLeftOffset[1] << " " << topLeftOffset[2] << endl;
			//printVec3InUnity(topLeftOffset, "top left offset after adding origin");

			// end calculate ray parameters

			// calculate ray direction
			float target[] =
			{
				dx[0] * screenPos[0] + dy[0] * screenPos[1] + topLeftOffset[0],
				dx[1] * screenPos[0] + dy[1] * screenPos[1] + topLeftOffset[1],
				dx[2] * screenPos[0] + dy[2] * screenPos[1] + topLeftOffset[2],
			};

			//cout << "target: " << target[0] << " " << target[1] << " " << target[2] << " (world coordinates)" << endl;
			printVec3InUnity(target, "target");

			// prepare ray
			RTCRay ray;
			ray.org[0] = origin[0];
			ray.org[1] = origin[1];
			ray.org[2] = origin[2];
			// calculate direction by using viewMatrix (right, forward, up)
			ray.dir[0] = target[0] - origin[0];
			ray.dir[1] = target[1] - origin[1];
			ray.dir[2] = target[2] - origin[2];

			ray.tnear = 0;
			ray.tfar = std::numeric_limits<float>::infinity();
			ray.geomID = RTC_INVALID_GEOMETRY_ID;

			// throw ray
			rtcIntersect(rtcScene, ray);

			// check if ray hit something
			if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
				// normalize direction vector and get hit point
				float magnitude = sqrtf(ray.dir[0] * ray.dir[0] + ray.dir[1] * ray.dir[1] + ray.dir[2] * ray.dir[2]);


				results[3] = ray.tfar;
				//float hitPoint[] =
				//{
				//	origin[0] + ray.dir[0] * ray.tfar / magnitude,
				//	origin[1] + ray.dir[1] * ray.tfar / magnitude,
				//	origin[2] + ray.dir[2] * ray.tfar / magnitude,
				//};

				//// transform from world space to voxel space
				//float voxel[] =
				//{
				//	(hitPoint[0] - offset[0]) * scale[0],
				//	(hitPoint[1] - offset[1]) * scale[1],
				//	(hitPoint[2] - offset[2]) * scale[2],
				//};

				//unsigned voxelId = (int)((int)voxel[0] * gridSize * gridSize + (int)voxel[1] * gridSize + (int)voxel[2]);

				//if (voxelId >= 0 && voxelId < (gridSize * gridSize * gridSize)) {
				//	result.values[voxelId] = ray.tfar;
				//}

			}
			else {
				results[3] = -1;
			}

			results[0] = ray.dir[0];
			results[1] = ray.dir[1];
			results[2] = ray.dir[2];
		}
	}

    void test (float origin[], float camParameters[], float * result)
    {

		if (initialized) {
			auto start = std::chrono::steady_clock::now();

			ResultPkg calculatedResult (gridSize * gridSize * gridSize);

			if (firstExecution) {
				// dont expect any result. there is no task submited yet
				firstExecution = false;
			}
			else {
				// return value calculated by previous submited task
				auto getResultStart = std::chrono::steady_clock::now();
				calculatedResult = resultPtr->get();
				auto getResultEnd = std::chrono::steady_clock::now();

				float getResultDeltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(getResultEnd - getResultStart).count();	// in ms
				futureGetResultElapsedTime += getResultDeltaTime;

				delete resultPtr;
			}

			// submit new task
			resultPtr = new std::future<ResultPkg>(std::async(calculateValue, origin, camParameters));


			// set values to return
			memcpy(result, &(calculatedResult.values[0]), sizeof(float) * gridSize * gridSize * gridSize);

			auto end = std::chrono::steady_clock::now();

			float restOfCodeDeltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();	// in ms

			restOfCodeElapsedTime += restOfCodeDeltaTime;
		}
    }

    void finishPlugin ()
    {
		//cerr << "[begin finishPlugin]" << endl;
		//debugInUnity("[begin finishPlugin]");
		if (initialized) {
			// ask for last result (otherwise it will crash)
			ResultPkg calculatedResult(windowSize[0] * windowSize[1]);

			if (resultPtr != nullptr) {
				calculatedResult = resultPtr->get();
				delete resultPtr;
			}

			//cout << "elapsed time: " << restOfCodeElapsedTime << endl;
			//cout << "elapsed time waiting for the result: " << futureGetResultElapsedTime << endl;

			rtcDeleteScene(rtcScene);

			rtcDeleteDevice(rtcDevice);

			initialized = false;
		}
		//cerr << "[end finishPlugin]" << endl;
		debugInUnity("[end finishPlugin]");
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

		debugInUnity(msg);
    }

	void printVec3(float vector[], std::string prefix)
	{
		cout << prefix << ": " << vector[0] << " " << vector[1] << " " << vector[2] << endl;
	}

	void printVec3InUnity(float vector[], std::string prefix)
	{
		debugInUnity(prefix + ": " + std::to_string(vector[0]) + " " + std::to_string(vector[1]) + " " + std::to_string(vector[2]));
	}
}
