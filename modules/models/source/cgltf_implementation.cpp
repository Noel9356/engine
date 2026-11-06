// Single translation unit that compiles the cgltf parser implementation
// (CGLTF_IMPLEMENTATION). Every other file includes cgltf.h for declarations
// only, so the third-party code is instantiated exactly once in the module.

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
