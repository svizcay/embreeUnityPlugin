#ifndef VOXEL_VISIBILITY_CHECKER_H
#define VOXEL_VISIBILITY_CHECKER_H


#define EXPORT_API __declspec(dllexport) 

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>

extern "C"
{
    EXPORT_API void initPlugin (int _gridSize, float _offset[], float _scale[], int _textureWidth, int _textureHeight, float _vertices[], int _nrVertices, float _indices[], int _nrIndices);
	EXPORT_API void throwRay(RTCRay & ray);
    EXPORT_API void test (float origin[], float camParameters[], float * result);
    EXPORT_API void finishPlugin ();
}


#endif /* VOXEL_VISIBILITY_CHECKER_H */

