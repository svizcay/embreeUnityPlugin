#ifndef VOXEL_VISIBILITY_CHECKER_H
#define VOXEL_VISIBILITY_CHECKER_H


#define EXPORT_API __declspec(dllexport) 

#include <string>

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>

typedef void(__stdcall * DebugCallback) (const char * str);
DebugCallback gDebugCallback;

extern "C"
{
    EXPORT_API void initPlugin (int _gridSize, float _offset[], float _scale[], int _textureWidth, int _textureHeight, float _vertices[], int _nrVertices, int _indices[], int _nrIndices);
    EXPORT_API void test (float origin[], float camParameters[], float * result);
	EXPORT_API void mousePickup(float origin[], float camParameters[], float screenPos[], float * results);
    EXPORT_API void finishPlugin ();

	EXPORT_API void registerDebugCallback(DebugCallback callback)
	{
		if (callback)
		{
			gDebugCallback = callback;
		}
	}

	void printVec3(float vector[], std::string prefix = "values");
	void printVec3InUnity (float vector[], std::string prefix = "values");
}

void debugInUnity(std::string message)
{
	if (gDebugCallback)
	{
		gDebugCallback(message.c_str());
	}
}



#endif /* VOXEL_VISIBILITY_CHECKER_H */

