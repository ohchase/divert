mod binding;

use std::{
    marker,
    ops::{Add, Mul, Sub},
};

use binding::*;
use bitflags::bitflags;
use thiserror::Error;

use recastnavigation_sys::*;

bitflags! {
    #[repr(C)]
    pub struct DtStatus: dtStatus {
        // High level status.
        const SUCCESS = DT_SUCCESS; // Operation failed.
        const IN_PROGRESS = DT_IN_PROGRESS; // Operation succeed.
        const FAILURE = DT_FAILURE; // Operation still in progress.

        // Detail information for status.
        const WRONG_MAGIC = DT_WRONG_MAGIC; // Input data is not recognized.
        const WRONG_VERSION = DT_WRONG_VERSION; // Input data is in wrong version.
        const OUT_OF_MEMORY = DT_OUT_OF_MEMORY; // Operation ran out of memory.
        const INVALID_PARAM = DT_INVALID_PARAM; // An input parameter was invalid.
        const BUFFER_TOO_SMALL = DT_BUFFER_TOO_SMALL; // Result buffer for the query was too small to store all results.
        const OUT_OF_NODES = DT_OUT_OF_NODES; // Query ran out of nodes during search.
        const PARTIAL_RESULT = DT_PARTIAL_RESULT; // Query did not reach the end location, returning best guess.
        const ALREADY_OCCUPIED = DT_ALREADY_OCCUPIED; // A tile has already been assigned to the given x,y coordinate
    }
}

impl DtStatus {
    pub fn is_success(&self) -> bool {
        self.contains(DtStatus::SUCCESS)
    }

    pub fn is_in_progress(&self) -> bool {
        self.contains(DtStatus::IN_PROGRESS)
    }

    pub fn is_failed(&self) -> bool {
        self.contains(DtStatus::FAILURE)
    }
}

// The flags for points in a "straight path".
// Note: recastnavigation-sys generates either i32 or u32 for enums, but dtStraightPathFlags are
// generally taken as u8.
bitflags! {
    #[repr(transparent)]
    pub struct DtStraightPathFlags: u8 {
        const START = dtStraightPathFlags_DT_STRAIGHTPATH_START as _;
        const END = dtStraightPathFlags_DT_STRAIGHTPATH_END as _;
        const OFFMESH_CONNECTION = dtStraightPathFlags_DT_STRAIGHTPATH_OFFMESH_CONNECTION as _;
    }
}

#[derive(Error, Debug)]
pub enum DivertError {
    #[error("detour internal status failure `{0:?}")]
    Failure(DtStatus),
    #[error("detour unexpected null ptr failure")]
    NullPtr(),
    #[error("detour nav mesh unexpected dtNavMeshQuery::getPolyHeight failure `{0:?}`")]
    GetPolyHeightFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findNearestPoly failure `{0:?}`")]
    FindNearestPolyFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::closestPointOnPoly failure `{0:?}`")]
    ClosestPointOnPolyFailure(DtStatus),
    #[error(
        "detour nav mesh unexpected dtNavMeshQuery::closestPointOnPolyBoundary failure `{0:?}`"
    )]
    ClosestPointOnPolyBoundaryFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findPath failure `{0:?}`")]
    FindPathFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findStraightPath failure `{0:?}`")]
    FindStraightPathFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::moveAlongSurface failure `{0:?}`")]
    MoveAlongSurfaceFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findRandomPoint failure `{0:?}`")]
    FindRandomPoint(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::getTileAndPolyByRef failure `{0:?}`")]
    GetTileAndPolyByRef(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findPolysAroundCircle failure `{0:?}`")]
    FindPolysAroundCircle(DtStatus),
}

pub type DivertResult<T> = std::result::Result<T, DivertError>;

/// 3D Vector used in Recast Navigation, correspond to a [f32; 3]
/// This abstraction is provided to combat misunderstanding of point ordering
/// Recast expects y, z, x ordering while many applications use x, y, z ordering
pub type Vector = DtVector;

/// Provides functionality to initialize Vectors compatible with Recast
/// Additionally provides basic math functions used with 3D Vectors
impl Vector {
    /// Simple function to help with initializing while maintaining expected ordering
    pub fn from_xyz(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Simple function to help with initializing while maintaining expected ordering
    pub fn from_yzx(y: f32, z: f32, x: f32) -> Self {
        Self { x, y, z }
    }

    /// Dot product of the vector and other
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

impl Add for Vector {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vector {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for Vector {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

/// Typedef to DtNavMeshParams
/// Affords the ability in future to add custom functionality
pub type NavMeshParams = DtNavMeshParams;

/// New Type to DdPolyRef
#[derive(Debug, Default, PartialEq, Eq, Copy, Clone)]
pub struct PolyRef(DtPolyRef);

impl std::ops::Deref for PolyRef {
    type Target = DtPolyRef;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for PolyRef {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// New Type to dtTileRef
#[derive(Debug, Default, PartialEq, Eq, Copy, Clone)]
pub struct TileRef(DtTileRef);

impl std::ops::Deref for TileRef {
    type Target = DtTileRef;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for TileRef {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// New Type to dtMeshTile
pub struct MeshTile {
    handle: *const DtMeshTile,
}

impl MeshTile {
    pub fn bv_node_count(&self) -> i32 {
        unsafe { (*(*self.handle).header).bv_node_count }
    }

    pub fn vertices(&self) -> &[DtVector] {
        unsafe {
            std::slice::from_raw_parts(
                (*self.handle).verts,
                (*(*self.handle).header).vert_count.try_into().unwrap(),
            )
        }
    }

    pub fn detail_meshes(&self) -> &[DtPolyDetail] {
        unsafe {
            std::slice::from_raw_parts(
                (*self.handle).detail_meshes,
                (*(*self.handle).header)
                    .detail_mesh_count
                    .try_into()
                    .unwrap(),
            )
        }
    }

    pub fn detail_vertices(&self) -> &[DtVector] {
        unsafe {
            std::slice::from_raw_parts(
                (*self.handle).detail_verts,
                (*(*self.handle).header)
                    .detail_vert_count
                    .try_into()
                    .unwrap(),
            )
        }
    }

    pub fn detail_tris(&self) -> &[DtDetailTri] {
        unsafe {
            std::slice::from_raw_parts(
                (*self.handle).detail_tris,
                (*(*self.handle).header)
                    .detail_tri_count
                    .try_into()
                    .unwrap(),
            )
        }
    }
}

#[derive(Debug)]
pub struct Triangle<'a> {
    pub a: &'a DtVector,
    pub b: &'a DtVector,
    pub c: &'a DtVector,
    pub flag: u8,
}

/// New Type to dtPolyDetail
pub struct PolygonDetail {
    handle: *const DtPolyDetail,
}

impl PolygonDetail {
    fn vert_base(&self) -> usize {
        unsafe { (*self.handle).vert_base as usize }
    }

    fn tri_base(&self) -> usize {
        unsafe { (*self.handle).tri_base as usize }
    }

    fn vert_count(&self) -> usize {
        unsafe { (*self.handle).vert_base as usize }
    }

    fn tri_count(&self) -> usize {
        unsafe { (*self.handle).tri_count as usize }
    }

    pub fn tris<'a>(&self, detail_tris: &'a [DtDetailTri]) -> &'a [DtDetailTri] {
        let tri_base = self.tri_base();
        let tri_count = self.tri_count();
        &detail_tris[tri_base..tri_base + tri_count]
    }

    pub fn vertices<'a>(&self, detail_vertices: &'a [DtVector]) -> &'a [DtVector] {
        let vert_base = self.vert_base();
        let vert_count = self.vert_count();
        &detail_vertices[vert_base..vert_base + vert_count]
    }
}

/// New Type to dtPolygon
pub struct Polygon {
    handle: *const DtPoly,
}

struct PolygonTriangleIterator<'a> {
    tri_flags: &'a [DtDetailTri],
    vertice_indexes: &'a [u16],
    vertices: &'a [DtVector],
    details_vert_base: usize,
    detail_vertices: &'a [DtVector],
    index: usize,
}

impl<'a> PolygonTriangleIterator<'a> {
    fn map_tri(
        vertice_index: usize,
        vertice_indexes: &'a [u16],
        vertices: &'a [DtVector],
        details_vert_base: usize,
        detail_vertices: &'a [DtVector],
    ) -> &'a DtVector {
        if vertice_index < vertice_indexes.len() {
            let vertice_index = vertice_indexes[vertice_index] as usize;
            &vertices[vertice_index]
        } else {
            let vertice_index = vertice_index - vertice_indexes.len();
            let vertice_index = details_vert_base + vertice_index;
            &detail_vertices[vertice_index]
        }
    }
}

impl<'a> Iterator for PolygonTriangleIterator<'a> {
    type Item = Triangle<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index == self.tri_flags.len() {
            return None;
        }

        let tri_flags = self.tri_flags[self.index].0;
        let triangle = Triangle {
            a: Self::map_tri(
                tri_flags[0] as usize,
                self.vertice_indexes,
                self.vertices,
                self.details_vert_base,
                self.detail_vertices,
            ),
            b: Self::map_tri(
                tri_flags[1] as usize,
                self.vertice_indexes,
                self.vertices,
                self.details_vert_base,
                self.detail_vertices,
            ),
            c: Self::map_tri(
                tri_flags[2] as usize,
                self.vertice_indexes,
                self.vertices,
                self.details_vert_base,
                self.detail_vertices,
            ),
            flag: tri_flags[3],
        };

        log::info!("TriFlags: {:?}", tri_flags);

        self.index += 1;
        Some(triangle)
    }
}

struct PolygonVerticeIterator<'a> {
    vertices: &'a [DtVector],
    vertice_indexes: &'a [u16],
    index: usize,
}

impl<'a> Iterator for PolygonVerticeIterator<'a> {
    type Item = &'a DtVector;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index == self.vertice_indexes.len() {
            return None;
        }

        let vert_index = self.vertice_indexes[self.index] as usize;
        self.index += 1;

        Some(&self.vertices[vert_index])
    }
}

impl Polygon {
    fn verts(&self) -> &[u16; 6] {
        unsafe { &(*self.handle).verts }
    }

    fn vert_count(&self) -> usize {
        unsafe { (*self.handle).vert_count as usize }
    }

    fn vertice_indexes(&self) -> &[u16] {
        &self.verts()[0..self.vert_count()]
    }

    pub fn flags(&self) -> u16 {
        unsafe { (*self.handle).flags }
    }

    pub fn triangle_iterator<'b>(
        &'b self,
        tile: &'b MeshTile,
    ) -> impl Iterator<Item = Triangle<'b>> {
        let details = self.details(tile);
        let tri_flags = details.tris(tile.detail_tris());
        let details_vert_base = details.vert_base();

        PolygonTriangleIterator {
            tri_flags,
            vertice_indexes: self.vertice_indexes(),
            vertices: tile.vertices(),
            details_vert_base,
            detail_vertices: tile.detail_vertices(),
            index: 0,
        }
    }

    pub fn vertices_iter<'b>(&'b self, tile: &'b MeshTile) -> impl Iterator<Item = &'b DtVector> {
        PolygonVerticeIterator {
            vertices: tile.vertices(),
            vertice_indexes: self.vertice_indexes(),
            index: 0,
        }
    }

    pub fn details(&self, tile: &MeshTile) -> PolygonDetail {
        let index = unsafe { self.handle.offset_from((*tile.handle).polys) };
        PolygonDetail {
            handle: &tile.detail_meshes()[index as usize],
        }
    }
}

/// Safe bindings to dtNavMesh
/// Handles life time of the dtNavMesh and will release resources when dropped
pub struct NavMesh<'a> {
    handle: *mut DtNavMesh,
    _phantom: marker::PhantomData<&'a DtNavMesh>,
}

unsafe impl Send for NavMesh<'_> {}

/// Provides functionality to interact with NavMesh and its underlying dtNavMesh
impl<'a> NavMesh<'a> {
    pub fn get_tile_and_poly_by_ref(&self, poly_ref: PolyRef) -> DivertResult<(MeshTile, Polygon)> {
        let mut output_tile = std::ptr::null();
        let mut output_poly = std::ptr::null();

        let status = unsafe {
            dtNavMesh_getTileAndPolyByRef(
                self.handle,
                *poly_ref,
                &mut output_tile,
                &mut output_poly,
            )
        };

        if status.is_failed() {
            return Err(DivertError::GetTileAndPolyByRef(status));
        }

        Ok((
            MeshTile {
                handle: output_tile,
            },
            Polygon {
                handle: output_poly,
            },
        ))
    }

    pub fn calc_tile_location(&self, position: &DtVector) -> (i32, i32) {
        unsafe {
            let mut tile_x: i32 = -1;
            let mut tile_y: i32 = -1;
            dtNavMesh_calcTileLoc(self.handle, position, &mut tile_x, &mut tile_y);
            (tile_x, tile_y)
        }
    }

    pub fn get_poly_ref_base(&self, tile: &DtMeshTile) -> Option<DtPolyRef> {
        unsafe {
            let poly_ref = dtNavMesh_getPolyRefBase(self.handle, tile);
            match dtNavMesh_isValidPolyRef(self.handle, poly_ref) {
                true => Some(poly_ref),
                false => None,
            }
        }
    }

    pub fn get_tile_at(&self, tile_x: i32, tile_y: i32, layer: i32) -> Option<&DtMeshTile> {
        unsafe { dtNavMesh_getTileAt(self.handle, tile_x, tile_y, layer).as_ref() }
    }

    /// Allocates and initializes a dtNavMesh for NavMesh to handle
    /// Errors if allocation returns a null pointer, or the dtNavMesh->init function returns a failed status
    pub fn new(nav_mesh_params: &NavMeshParams) -> DivertResult<Self> {
        let dt_nav_mesh = unsafe { dtNavMesh_alloc() };

        if dt_nav_mesh.is_null() {
            return Err(DivertError::NullPtr());
        }

        let init_status = unsafe { dtNavMesh_init(dt_nav_mesh, nav_mesh_params) };
        if init_status.is_failed() {
            return Err(DivertError::Failure(init_status));
        }

        Ok(Self {
            handle: dt_nav_mesh,
            _phantom: marker::PhantomData,
        })
    }

    /// Accepts a byte vector representing a dtTile, adding it to the inner dtNavMesh
    /// The byte vector is forgotten after being added to the dtNavMesh
    /// Forgetting the memory is critical, because the memory is now owned by the dtNavMesh
    pub fn add_tile(&mut self, input_data: Vec<u8>) -> DivertResult<TileRef> {
        let mut boxed_slice = input_data.into_boxed_slice();
        let data = boxed_slice.as_mut_ptr();
        let data_size = boxed_slice.len();

        let mut tile_ref = TileRef::default();
        let add_tile_status = unsafe {
            dtNavMesh_addTile(
                self.handle,
                data,
                data_size as i32,
                1,
                *TileRef::default(),
                &mut *tile_ref,
            )
        };

        if add_tile_status.is_failed() {
            return Err(DivertError::Failure(add_tile_status));
        }

        std::mem::forget(boxed_slice);
        Ok(tile_ref)
    }

    /// Allocates and initializes a dtNavMeshQuery for NavMeshQuery to handle
    /// Errors if allocation returns a null pointer, or the dtNavMeshQuery->init function returns a failed status
    pub fn create_query<'b>(&self, max_nodes: i32) -> DivertResult<NavMeshQuery<'b>> {
        let dt_nav_mesh_query = unsafe { dtNavMeshQuery_alloc() };

        if dt_nav_mesh_query.is_null() {
            return Err(DivertError::NullPtr());
        }

        let init_status = unsafe { dtNavMeshQuery_init(dt_nav_mesh_query, self.handle, max_nodes) };
        if init_status.is_failed() {
            return Err(DivertError::Failure(init_status));
        }

        Ok(NavMeshQuery {
            handle: dt_nav_mesh_query,
            _phantom: marker::PhantomData,
        })
    }
}

/// Handles freeing the inner dtNavMesh
/// subsequently handles freeing the tile data added to this NavMesh
impl<'a> Drop for NavMesh<'a> {
    /// Frees dtNavMesh resources with dtFreeNavMesh
    fn drop(&mut self) {
        unsafe { dtNavMesh_free(self.handle) }
    }
}

/// Safe bindings to dtQueryFilter
/// Handles life time of the dtQueryFilter and will release resources when dropped
pub struct QueryFilter<'a> {
    handle: *mut DtQueryFilter,
    _phantom: marker::PhantomData<&'a DtQueryFilter>,
}

unsafe impl Send for QueryFilter<'_> {}

/// Provides functionality to interact with QueryFilter and its underlying dtQueryFilter
impl<'a> QueryFilter<'a> {
    /// Allocates a dtQueryFilter
    /// Errors if the allocation returns a null pointer
    pub fn new() -> DivertResult<Self> {
        let dt_query_filter = unsafe { dtQueryFilter_alloc() };
        if dt_query_filter.is_null() {
            Err(DivertError::NullPtr())
        } else {
            Ok(Self {
                handle: dt_query_filter,
                _phantom: marker::PhantomData,
            })
        }
    }

    /// Sets the filter's include flags
    pub fn set_include_flags(&mut self, include_flags: u16) {
        unsafe {
            dtQueryFilter_setIncludeFlags(self.handle, include_flags);
        }
    }

    /// Retrieves the filter's include flags
    pub fn get_include_flags(&self) -> u16 {
        unsafe { dtQueryFilter_getIncludeFlags(self.handle) }
    }

    /// Sets the filter's exclude flags
    pub fn set_exclude_flags(&mut self, exclude_flags: u16) {
        unsafe {
            dtQueryFilter_setExcludeFlags(self.handle, exclude_flags);
        }
    }

    /// Retrieves the filter's exclude flags
    pub fn get_exclude_flags(&self) -> u16 {
        unsafe { dtQueryFilter_getExcludeFlags(self.handle) }
    }
}

/// Handles freeing the inner dtQueryFilter
impl<'a> Drop for QueryFilter<'a> {
    fn drop(&mut self) {
        unsafe { dtQueryFilter_free(self.handle) }
    }
}

/// Safe bindings to dtNavMeshQuery
/// Handles life time of the dtNavMeshQuery and will release resources when dropped
pub struct NavMeshQuery<'a> {
    handle: *mut DtNavMeshQuery,
    _phantom: marker::PhantomData<&'a DtNavMeshQuery>,
}

unsafe impl Send for NavMeshQuery<'_> {}

/// Provides functionality to interact with NavMeshQuery and its underlying dtNavMeshQuery
impl<'a> NavMeshQuery<'a> {
    pub fn find_polys_around_circle(
        &self,
        start_ref: PolyRef,
        center: &DtVector,
        radius: f32,
        filter: &QueryFilter,
        max_result: usize,
    ) -> DivertResult<Vec<(PolyRef, PolyRef, f32)>> {
        // Vectors
        let mut result_refs: Vec<PolyRef> = Vec::with_capacity(max_result);
        let mut result_parents: Vec<PolyRef> = Vec::with_capacity(max_result);
        let mut result_costs = Vec::with_capacity(max_result);
        let mut result_count = 0;

        let status = unsafe {
            dtNavMeshQuery_findPolysAroundCircle(
                self.handle,
                *start_ref,
                center,
                radius,
                filter.handle,
                result_refs.as_mut_ptr() as *mut DtPolyRef,
                result_parents.as_mut_ptr() as *mut DtPolyRef,
                result_costs.as_mut_ptr(),
                &mut result_count,
                max_result.try_into().unwrap(),
            )
        };

        if status.is_failed() {
            return Err(DivertError::FindPolysAroundCircle(status));
        }

        let result_count = result_count as usize;
        unsafe {
            result_refs.set_len(result_count);
            result_parents.set_len(result_count);
            result_costs.set_len(result_count);
        }

        let find_result = result_refs
            .into_iter()
            .zip(result_parents.into_iter())
            .zip(result_costs.into_iter())
            .map(|((poly_ref, parent_ref), result_cost)| (poly_ref, parent_ref, result_cost))
            .collect();

        Ok(find_result)
    }

    pub fn find_random_point(
        &self,
        frand: extern "C" fn() -> f32,
        filter: &QueryFilter,
    ) -> DivertResult<(PolyRef, DtVector)> {
        unsafe {
            let mut output_poly_ref = PolyRef::default();
            let mut output_point = DtVector::default();

            let random_point_status = dtNavMeshQuery_findRandomPoint(
                self.handle,
                filter.handle,
                frand,
                &mut output_poly_ref as *mut _ as *mut DtPolyRef,
                &mut output_point,
            );

            if random_point_status.is_failed() {
                return Err(DivertError::FindRandomPoint(random_point_status));
            }

            Ok((output_poly_ref, output_point))
        }
    }

    /// Queries for polygon height given the reference polygon and position on the polygon
    /// Errors if ffi function returns a failed DtStatus
    pub fn get_poly_height(&self, poly_ref: PolyRef, position: &DtVector) -> DivertResult<f32> {
        let mut height: f32 = 0.0;

        let get_poly_height_status =
            unsafe { dtNavMeshQuery_getPolyHeight(self.handle, *poly_ref, position, &mut height) };

        if get_poly_height_status.is_failed() {
            return Err(DivertError::GetPolyHeightFailure(get_poly_height_status));
        }

        Ok(height)
    }

    /// Queries for nearest polygon given a center point, a search area (extents), and a filter
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_nearest_poly(
        &self,
        center: &Vector,
        extents: &Vector,
        filter: &QueryFilter,
    ) -> DivertResult<(PolyRef, Vector)> {
        let mut closest_point = Vector::default();
        let mut nearest_ref = PolyRef::default();

        let nearest_status = unsafe {
            dtNavMeshQuery_findNearestPoly(
                self.handle,
                center,
                extents,
                filter.handle,
                &mut *nearest_ref,
                &mut closest_point,
            )
        };

        if nearest_status.is_failed() {
            return Err(DivertError::FindNearestPolyFailure(nearest_status));
        }

        Ok((nearest_ref, closest_point))
    }

    /// Queries for closest point on poly to a given position
    /// Errors if ffi function returns a failed DtStatus
    pub fn closest_point_on_poly(
        &self,
        poly_ref: PolyRef,
        position: &Vector,
    ) -> DivertResult<(Vector, bool)> {
        let mut closest_point = Vector::default();
        let mut position_over_poly = false;

        let nearest_status = unsafe {
            dtNavMeshQuery_closestPointOnPoly(
                self.handle,
                *poly_ref,
                position,
                &mut closest_point,
                &mut position_over_poly,
            )
        };

        if nearest_status.is_failed() {
            return Err(DivertError::ClosestPointOnPolyFailure(nearest_status));
        }

        Ok((closest_point, position_over_poly))
    }

    /// Queries for closest point on poly boundary to a given position
    /// Errors if ffi function returns a failed DtStatus
    pub fn closest_point_on_poly_boundary(
        &self,
        poly_ref: PolyRef,
        position: &Vector,
    ) -> DivertResult<Vector> {
        let mut closest_point = Vector::default();

        let dt_result = unsafe {
            dtNavMeshQuery_closestPointOnPolyBoundary(
                self.handle,
                *poly_ref,
                position,
                &mut closest_point,
            )
        };

        if dt_result.is_failed() {
            return Err(DivertError::ClosestPointOnPolyBoundaryFailure(dt_result));
        }

        Ok(closest_point)
    }

    /// Generates a polygon path from one (poly, position) to another (poly, position)
    /// Uses a user provided PolyRef Vector
    /// Max Path length is derived from the user provided PolyRef Vec's capacity
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_path_inplace(
        &self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        path: &mut Vec<PolyRef>,
    ) -> DivertResult<DtStatus> {
        let mut path_count = 0;

        let find_path_status = unsafe {
            dtNavMeshQuery_findPath(
                self.handle,
                *start_ref,
                *end_ref,
                start_pos,
                end_pos,
                filter.handle,
                path.as_mut_ptr() as *mut DtPolyRef,
                &mut path_count,
                path.capacity().try_into().unwrap(),
            )
        };

        unsafe {
            path.set_len(path_count as usize);
        }

        if find_path_status.is_failed() {
            return Err(DivertError::FindPathFailure(find_path_status));
        }

        Ok(find_path_status)
    }

    /// Generates a polygon path from one (poly, position) to another (poly, position)
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_path(
        &self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        max_path: i32,
    ) -> DivertResult<Vec<PolyRef>> {
        let mut path_count = 0;
        let mut path: Vec<PolyRef> = Vec::with_capacity(max_path.try_into().unwrap());

        let find_path_status = unsafe {
            dtNavMeshQuery_findPath(
                self.handle,
                *start_ref,
                *end_ref,
                start_pos,
                end_pos,
                filter.handle,
                path.as_mut_ptr() as *mut DtPolyRef,
                &mut path_count,
                max_path,
            )
        };

        unsafe {
            path.set_len(path_count as usize);
        }

        if find_path_status.is_failed() {
            return Err(DivertError::FindPathFailure(find_path_status));
        }

        Ok(path)
    }

    #[allow(clippy::too_many_arguments)]
    /// Generates a (poly, position) path from on (poly, position) to another (poly, position)
    /// Uses a user provided DtVector Vec, DtStraightPathFlags Vec, and PolyRef Vec
    /// Max Path length is derived from the user provided DtVector Vec's capacity
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_straight_path_inplace(
        &self,
        start_pos: &Vector,
        end_pos: &Vector,
        poly_path: &[PolyRef],
        straight_path_points: &mut Vec<DtVector>,
        straight_path_flags: &mut Vec<DtStraightPathFlags>,
        straight_path_polys: &mut Vec<PolyRef>,
        options: i32,
    ) -> DivertResult<DtStatus> {
        let mut straight_path_count = 0;

        let find_path_status = unsafe {
            dtNavMeshQuery_findStraightPath(
                self.handle,
                start_pos,
                end_pos,
                poly_path.as_ptr() as *const DtPolyRef,
                poly_path.len().try_into().unwrap(),
                straight_path_points.as_mut_ptr(),
                straight_path_flags.as_mut_ptr(),
                straight_path_polys.as_mut_ptr() as *mut DtPolyRef,
                &mut straight_path_count,
                straight_path_points.capacity().try_into().unwrap(),
                options,
            )
        };

        if find_path_status.is_failed() {
            return Err(DivertError::FindStraightPathFailure(find_path_status));
        }

        let path_count = straight_path_count as usize;
        unsafe {
            straight_path_points.set_len(path_count);
            straight_path_flags.set_len(path_count);
            straight_path_polys.set_len(path_count);
        }

        Ok(find_path_status)
    }

    /// Generates a (poly, position) path from on (poly, position) to another (poly, position)
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_straight_path(
        &self,
        start_pos: &Vector,
        end_pos: &Vector,
        poly_path: &[PolyRef],
        max_path: i32,
        options: i32,
    ) -> DivertResult<Vec<(Vector, DtStraightPathFlags, PolyRef)>> {
        let mut straight_path_count = 0;
        let mut straight_path_points: Vec<DtVector> =
            Vec::with_capacity(max_path.try_into().unwrap());
        let mut straight_path_flags: Vec<DtStraightPathFlags> =
            Vec::with_capacity(max_path.try_into().unwrap());
        let mut straight_path_polys: Vec<PolyRef> =
            Vec::with_capacity(max_path.try_into().unwrap());

        let straight_path_status = unsafe {
            dtNavMeshQuery_findStraightPath(
                self.handle,
                start_pos,
                end_pos,
                poly_path.as_ptr() as *const DtPolyRef,
                poly_path.len().try_into().unwrap(),
                straight_path_points.as_mut_ptr(),
                straight_path_flags.as_mut_ptr(),
                straight_path_polys.as_mut_ptr() as *mut DtPolyRef,
                &mut straight_path_count,
                max_path,
                options,
            )
        };

        if straight_path_status.is_failed() {
            return Err(DivertError::FindStraightPathFailure(straight_path_status));
        }

        let path_count = straight_path_count as usize;

        unsafe {
            straight_path_points.set_len(path_count);
            straight_path_flags.set_len(path_count);
            straight_path_polys.set_len(path_count);
        }

        if straight_path_status.is_failed() {
            return Err(DivertError::FindStraightPathFailure(straight_path_status));
        }

        let path_result = straight_path_points
            .into_iter()
            .zip(straight_path_flags.into_iter())
            .zip(straight_path_polys.into_iter())
            .map(|((pos, flags), poly_ref)| (pos, flags, poly_ref))
            .collect();

        Ok(path_result)
    }

    /// Generates a poly path while moving from (poly, position) to a (poly)
    /// Uses a user provided PolyRef Vec
    /// Max Path length is derived from the user provided PolyRef Vec's capacity
    /// Errors if ffi function returns a failed DtStatus
    pub fn move_along_surface_inplace(
        &self,
        start_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        result_pos: &mut Vector,
        visited: &mut Vec<PolyRef>,
    ) -> DivertResult<DtStatus> {
        let mut visited_count = 0;

        let move_along_surface_result = unsafe {
            dtNavMeshQuery_moveAlongSurface(
                self.handle,
                *start_ref,
                start_pos,
                end_pos,
                filter.handle,
                result_pos,
                visited.as_mut_ptr() as *mut DtPolyRef,
                &mut visited_count,
                visited.capacity().try_into().unwrap(),
            )
        };

        unsafe {
            visited.set_len(visited_count as usize);
        }

        if move_along_surface_result.is_failed() {
            return Err(DivertError::MoveAlongSurfaceFailure(
                move_along_surface_result,
            ));
        }

        Ok(move_along_surface_result)
    }

    /// Generates a poly path while moving from (poly, position) to a (poly)
    /// Errors if ffi function returns a failed DtStatus
    pub fn move_along_surface(
        &self,
        start_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        max_visit: i32,
    ) -> DivertResult<(Vector, Vec<PolyRef>)> {
        let mut visited_count = 0;
        let mut visited: Vec<PolyRef> = Vec::with_capacity(max_visit.try_into().unwrap());
        let mut result_pos = Vector::default();

        let move_along_surface_result = unsafe {
            dtNavMeshQuery_moveAlongSurface(
                self.handle,
                *start_ref,
                start_pos,
                end_pos,
                filter.handle,
                &mut result_pos,
                visited.as_mut_ptr() as *mut DtPolyRef,
                &mut visited_count,
                max_visit,
            )
        };

        unsafe {
            visited.set_len(visited_count as usize);
        }

        if move_along_surface_result.is_failed() {
            return Err(DivertError::MoveAlongSurfaceFailure(
                move_along_surface_result,
            ));
        }

        Ok((result_pos, visited))
    }
}

/// Handles freeing the inner dtNavMeshQuery
impl<'a> Drop for NavMeshQuery<'a> {
    /// Frees dtNavMeshQuery resources with dtFreeNavMeshQuery
    fn drop(&mut self) {
        unsafe { dtNavMeshQuery_free(self.handle) }
    }
}

#[cfg(test)]
mod tests {

    use crate::{NavMesh, NavMeshParams, QueryFilter};

    #[test]
    fn test_nav_mesh() {
        let nav_mesh_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_polys: 1000,
            max_tiles: 1,
        };

        let nav_mesh = NavMesh::new(&nav_mesh_params);
        assert!(nav_mesh.is_ok());

        let _nav_mesh = nav_mesh.unwrap();
    }

    #[test]
    fn test_nav_mesh_query() {
        let nav_mesh_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_polys: 1000,
            max_tiles: 1,
        };

        let nav_mesh = NavMesh::new(&nav_mesh_params).unwrap();
        let nav_mesh_query = nav_mesh.create_query(256);
        assert!(nav_mesh_query.is_ok());
    }

    #[test]
    fn test_query_filter() {
        let filter = QueryFilter::new();
        assert!(filter.is_ok());

        let mut filter = filter.unwrap();

        filter.set_include_flags(1);
        assert_eq!(filter.get_include_flags(), 1);

        filter.set_exclude_flags(1);
        assert_eq!(filter.get_exclude_flags(), 1);
    }
}
