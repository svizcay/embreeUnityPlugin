#include <iostream>
#include <string>
#include <chrono>

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>

#include <xmmintrin.h>	// functions for setting the control register
#include <pmmintrin.h>	// functions for setting the control register

#include "voxel_visibility_checker.h"

using std::cout;
using std::cerr;
using std::endl;

extern "C"
{

    struct Vertex { float x, y, z, a; };
    struct Triangle { int v0, v1, v2; };

    RTCDevice   rtcDevice;
    RTCScene    rtcScene;
    unsigned    proxyMeshGeomId;

    void embreeErrorCB(RTCError code, const char* msg);

	// parameters
	int gridSize;
	float offset[] = { 0, 0, 0 };
	float scale[]{ 1, 1, 1 };
	int windowSize[] = { 800, 600 };


    void initPlugin ()
    {
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);			// Enable 'Flush Zero' bit
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);	// Enable 'Denormals Zero'bit

        rtcDevice = rtcNewDevice(nullptr);

        // DEPRECATED. use rtcDeviceSetErrorFunction2
        rtcDeviceSetErrorFunction(rtcDevice, embreeErrorCB);

        rtcScene = rtcDeviceNewScene(rtcDevice, RTC_SCENE_STATIC | RTC_SCENE_COHERENT, RTC_INTERSECT1 /*| RTC_INTERPOLATE*/);

		unsigned numTriangles = 2;
		unsigned numVertices = 8;

        // load proxy
        proxyMeshGeomId = rtcNewTriangleMesh(rtcScene, RTC_GEOMETRY_STATIC, numTriangles, numVertices);

        Vertex* vertices = (Vertex*)rtcMapBuffer(rtcScene, proxyMeshGeomId, RTC_VERTEX_BUFFER);
        // remember that unity uses clockwise triangle order
        vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
        vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
        vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
        vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
        rtcUnmapBuffer(rtcScene, proxyMeshGeomId, RTC_VERTEX_BUFFER);

        Triangle* triangles = (Triangle*)rtcMapBuffer(rtcScene, proxyMeshGeomId, RTC_INDEX_BUFFER);
        triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
        triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
        rtcUnmapBuffer(rtcScene, proxyMeshGeomId, RTC_INDEX_BUFFER);

        // commit changes to the scene (all buffers have to get unmapped before commiting)
        rtcCommit(rtcScene);
    }

    void test (float origin[], float direction[], bool * result, int * result2)
    {
		auto start = std::chrono::steady_clock::now();

        // throw ray
        RTCRay ray;
        ray.org[0] = origin[0];
        ray.org[1] = origin[1];
        ray.org[2] = origin[2];
        ray.dir[0] = direction[0];
        ray.dir[1] = direction[1];
        ray.dir[2] = direction[2];

        ray.tnear = 0;
        ray.tfar = std::numeric_limits<float>::infinity();
        ray.geomID = RTC_INVALID_GEOMETRY_ID;

		for (int i = 0; i < windowSize[0]; i++) {
			for (int j = 0; j < windowSize[1]; j++) {
				rtcIntersect(rtcScene, ray);
			}
		}

        cout << "distance : " << ray.tfar << endl;

		for (int i = 0; i < 5; i++) {
			//result[i] = true;
			*(result + i) = true;
		}

		for (int i = 0; i < 5; i++) {
			*(result2 + i) = i;
		}

		auto end = std::chrono::steady_clock::now();

		cout << "elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms." << endl;


    }

    void finishPlugin ()
    {
        rtcDeleteScene(rtcScene);

        rtcDeleteDevice(rtcDevice);
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
