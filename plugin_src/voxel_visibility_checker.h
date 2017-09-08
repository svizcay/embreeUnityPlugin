#ifndef VOXEL_VISIBILITY_CHECKER_H
#define VOXEL_VISIBILITY_CHECKER_H


#define EXPORT_API __declspec(dllexport) 

extern "C"
{
    EXPORT_API void initPlugin ();
    EXPORT_API void test (float origin[], float direction[], bool * result, int * result2);
    EXPORT_API void finishPlugin ();
}


#endif /* VOXEL_VISIBILITY_CHECKER_H */

