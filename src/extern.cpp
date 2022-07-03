#include "../recastnavigation/Detour/Include/DetourStatus.h"
#include "../recastnavigation/Detour/Include/DetourNavMesh.h"
#include "../recastnavigation/Detour/Include/DetourNavMeshQuery.h"

extern "C"
{

  dtNavMesh *dtNavMesh_alloc()
  {
    return dtAllocNavMesh();
  }

  dtStatus dtNavMesh_init(dtNavMesh *mesh, const dtNavMeshParams *params)
  {
    return mesh->init(params);
  }

  dtStatus dtNavMesh_initSingle(dtNavMesh *mesh, unsigned char *data, int dataSize, int flags)
  {
    return mesh->init(data, dataSize, flags);
  }

  dtStatus dtNavMesh_addTile(dtNavMesh *mesh, unsigned char *data, int dataSize,
                             int flags, dtTileRef lastRef, dtTileRef *result)
  {
    return mesh->addTile(data, dataSize, flags, lastRef, result);
  }

  bool dtNavMesh_isValidPolyRef(dtNavMesh *mesh, dtPolyRef polyRef)
  {
    return mesh->isValidPolyRef(polyRef);
  }

  const dtMeshTile *dtNavMesh_getTileAt(dtNavMesh *mesh, const int x, const int y, const int layer)
  {
    return mesh->getTileAt(x, y, layer);
  }

  dtPolyRef dtNavMesh_getPolyRefBase(dtNavMesh *mesh, const dtMeshTile *tile)
  {
    return mesh->getPolyRefBase(tile);
  }

  void dtNavMesh_calcTileLoc(dtNavMesh *mesh, const float *pos, int *tx, int *ty)
  {
    return mesh->calcTileLoc(pos, tx, ty);
  }

  dtStatus dtNavMesh_getTileAndPolyByRef(dtNavMesh *mesh, const dtPolyRef ref, const dtMeshTile **tile, const dtPoly **poly)
  {
    return mesh->getTileAndPolyByRef(ref, tile, poly);
  }

  void dtNavMesh_free(dtNavMesh *mesh)
  {
    return dtFreeNavMesh(mesh);
  }

  dtQueryFilter *dtQueryFilter_alloc()
  {
    return new dtQueryFilter();
  }

  void dtQueryFilter_free(dtQueryFilter *filter)
  {
    delete filter;
  }

  void dtQueryFilter_setIncludeFlags(dtQueryFilter *filter, unsigned short flags)
  {
    filter->setIncludeFlags(flags);
  }

  unsigned short dtQueryFilter_getIncludeFlags(dtQueryFilter *filter)
  {
    return filter->getIncludeFlags();
  }

  void dtQueryFilter_setExcludeFlags(dtQueryFilter *filter, unsigned short flags)
  {
    filter->setExcludeFlags(flags);
  }

  unsigned short dtQueryFilter_getExcludeFlags(dtQueryFilter *filter)
  {
    return filter->getExcludeFlags();
  }

  dtNavMeshQuery *dtNavMeshQuery_alloc()
  {
    return dtAllocNavMeshQuery();
  }

  dtStatus dtNavMeshQuery_init(dtNavMeshQuery *query, dtNavMesh *mesh, int maxNodes)
  {
    return query->init(mesh, maxNodes);
  }

  dtStatus dtNavMeshQuery_queryPolygons(dtNavMeshQuery *query, const float *center, const float *halfExtents,
                                        const dtQueryFilter *filter,
                                        dtPolyRef *polys, int *polyCount, const int maxPolys)
  {
    return query->queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys);
  }

  dtStatus dtNavMeshQuery_findPolysAroundCircle(dtNavMeshQuery *query, dtPolyRef startRef, const float *centerPos, const float radius,
                                                const dtQueryFilter *filter,
                                                dtPolyRef *resultRef, dtPolyRef *resultParent, float *resultCost,
                                                int *resultCount, const int maxResult)
  {
    return query->findPolysAroundCircle(startRef, centerPos, radius, filter, resultRef, resultParent, resultCost, resultCount, maxResult);
  }

  dtStatus dtNavMeshQuery_getPolyHeight(dtNavMeshQuery *query, dtPolyRef polyRef, const float *pos, float *height)
  {
    return query->getPolyHeight(polyRef, pos, height);
  }

  dtStatus dtNavMeshQuery_findNearestPoly(dtNavMeshQuery *query, const float *center, const float *extents,
                                          const dtQueryFilter *filter,
                                          dtPolyRef *nearestRef, float *nearestPt)
  {
    return query->findNearestPoly(center, extents, filter, nearestRef, nearestPt);
  }

  dtStatus dtNavMeshQuery_closestPointOnPoly(dtNavMeshQuery *query, dtPolyRef ref, const float *pos, float *closest, bool *posOverPoly)
  {
    return query->closestPointOnPoly(ref, pos, closest, posOverPoly);
  }

  dtStatus dtNavMeshQuery_closestPointOnPolyBoundary(dtNavMeshQuery *query, dtPolyRef ref, const float *pos, float *closest)
  {
    return query->closestPointOnPolyBoundary(ref, pos, closest);
  }

  dtStatus dtNavMeshQuery_findPath(dtNavMeshQuery *query, dtPolyRef startRef, dtPolyRef endRef,
                                   const float *startPos, const float *endPos,
                                   const dtQueryFilter *filter,
                                   dtPolyRef *path, int *pathCount, const int maxPath)
  {
    return query->findPath(startRef, endRef, startPos, endPos, filter, path, pathCount, maxPath);
  }

  dtStatus dtNavMeshQuery_moveAlongSurface(dtNavMeshQuery *query, dtPolyRef startRef,
                                           const float *startPos, const float *endPos,
                                           const dtQueryFilter *filter,
                                           float *resultPos, dtPolyRef *visited, int *visitedCount, const int maxVisitedSize)
  {
    return query->moveAlongSurface(startRef, startPos, endPos, filter, resultPos, visited, visitedCount, maxVisitedSize);
  }

  dtStatus dtNavMeshQuery_findStraightPath(dtNavMeshQuery *query, const float *startPos, const float *endPos,
                                           const dtPolyRef *path, const int pathSize,
                                           float *straightPath, unsigned char *straightPathFlags, dtPolyRef *straightPathRefs,
                                           int *straightPathCount, const int maxStraightPath, const int options)
  {
    return query->findStraightPath(startPos, endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, straightPathCount, maxStraightPath, options);
  }

  dtStatus dtNavMeshQuery_findRandomPoint(dtNavMeshQuery *query, const dtQueryFilter *filter, float (*frand)(), dtPolyRef *randomRef, float *randomPt)
  {
    return query->findRandomPoint(filter, frand, randomRef, randomPt);
  }

  void dtNavMeshQuery_free(dtNavMeshQuery *query)
  {
    return dtFreeNavMeshQuery(query);
  }
};