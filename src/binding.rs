pub type DtTileRef = u64;

pub type DtPolyRef = u64;

#[repr(C)]
#[derive(Debug)]
pub struct DtPoly {
    pub first_link: u32,
    pub verts: [u16; 6],
    pub neighbors: [u16; 6],
    pub flags: u16,
    pub vert_count: u8,
    pub area_and_type: u8,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtLink {
    pub poly_ref: DtPolyRef,
    pub next: u32,
    pub edge: u8,
    pub side: u8,
    pub b_min: u8,
    pub b_max: u8,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtPolyDetail {
    pub vert_base: u32,
    pub tri_base: u32,
    pub vert_count: u8,
    pub tri_count: u8,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtDetailTri(pub [u8; 4]);

#[repr(C)]
#[derive(Debug)]
pub struct DtBVNode {
    pub b_min: [f32; 3],
    pub b_max: [f32; 3],
    pub i: i32,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtOffMeshConnection {
    pub pos: [f32; 6],
    pub radius: f32,
    pub poly: u16,
    pub flags: u8,
    pub side: u8,
    pub user_id: u32,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtMeshTile {
    pub salt: u32,
    pub links_free_list: u32,
    pub header: *const DtMeshHeader,
    pub polys: *const DtPoly,
    pub verts: *const DtVector,
    pub links: *const DtLink,
    pub detail_meshes: *const DtPolyDetail,
    pub detail_verts: *const DtVector,
    pub detail_tris: *const DtDetailTri,
    pub bv_tree: *const DtBVNode,
    pub off_mesh_connections: *const DtOffMeshConnection,
    pub data: *const u8,
    pub data_size: i32,
    pub flags: i32,
    pub next: *const DtMeshTile,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtNavMeshParams {
    pub origin: [f32; 3],
    pub tile_width: f32,
    pub tile_height: f32,
    pub max_tiles: i32,
    pub max_polys: i32,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtMeshHeader {
    pub magic: i32,
    pub version: i32,
    pub x: i32,
    pub y: i32,
    pub layer: i32,
    pub user_id: u32,
    pub poly_count: i32,
    pub vert_count: i32,
    pub max_link_count: i32,
    pub detail_mesh_count: i32,
    pub detail_vert_count: i32,
    pub detail_tri_count: i32,
    pub bv_node_count: i32,
    pub off_mesh_con_count: i32,
    pub off_mesh_base: i32,
    pub walkable_height: f32,
    pub walkable_climb: f32,
    pub b_min: [f32; 3],
    pub b_max: [f32; 3],
    pub bv_quant_factor: f32,
}

pub enum DtNavMesh {}

pub enum DtNavMeshQuery {}

pub enum DtQueryFilter {}

#[link(name = "detour", kind = "static")]
extern "C" {
    pub fn dtNavMesh_alloc() -> *mut DtNavMesh;
    pub fn dtNavMesh_init(_self: *mut DtNavMesh, params: *const DtNavMeshParams) -> DtStatus;
    pub fn dtNavMesh_addTile(
        _self: *mut DtNavMesh,
        data: *mut u8,
        data_size: i32,
        flags: i32,
        last_ref: DtTileRef,
        result: *mut DtTileRef,
    ) -> DtStatus;
    pub fn dtNavMesh_getTileAt(
        _self: *const DtNavMesh,
        tile_x: i32,
        tile_y: i32,
        layer: i32,
    ) -> *const DtMeshTile;
    pub fn dtNavMesh_getPolyRefBase(_self: *const DtNavMesh, tile: *const DtMeshTile) -> DtPolyRef;
    pub fn dtNavMesh_isValidPolyRef(_self: *const DtNavMesh, poly_ref: DtPolyRef) -> bool;
    pub fn dtNavMesh_calcTileLoc(
        _self: *const DtNavMesh,
        pos: *const DtVector,
        tile_x: *mut i32,
        tile_y: *mut i32,
    );
    pub fn dtNavMesh_getTileAndPolyByRef(
        _self: *const DtNavMesh,
        poly_ref: DtPolyRef,
        output_tile: *mut *const DtMeshTile,
        output_polygon: *mut *const DtPoly,
    ) -> DtStatus;
    pub fn dtNavMesh_free(_self: *mut DtNavMesh);

    pub fn dtNavMeshQuery_alloc() -> *mut DtNavMeshQuery;
    pub fn dtNavMeshQuery_init(
        _self: *mut DtNavMeshQuery,
        dt_nav_mesh: *const DtNavMesh,
        max_nodes: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_getPolyHeight(
        _self: *const DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        height: *mut f32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findNearestPoly(
        _self: *const DtNavMeshQuery,
        center: *const DtVector,
        extents: *const DtVector,
        filter: *const DtQueryFilter,
        nearest_ref: *mut DtPolyRef,
        nearest_point: *mut DtVector,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_closestPointOnPoly(
        _self: *const DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        closest: *mut DtVector,
        position_over_poly: *mut bool,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_closestPointOnPolyBoundary(
        _self: *const DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        closest: *mut DtVector,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findPath(
        _self: *const DtNavMeshQuery,
        start_ref: DtPolyRef,
        end_ref: DtPolyRef,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        filter: *const DtQueryFilter,
        path: *mut DtPolyRef,
        path_count: *mut i32,
        max_path: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findStraightPath(
        _self: *const DtNavMeshQuery,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        path: *const DtPolyRef,
        path_size: i32,
        straight_path_points: *mut DtVector,
        straight_path_flags: *mut DtStraightPathFlags,
        straight_path_polys: *mut DtPolyRef,
        straight_path_count: *mut i32,
        max_straight_path: i32,
        options: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findRandomPoint(
        _self: *const DtNavMeshQuery,
        filter: *const DtQueryFilter,
        frand: extern "C" fn() -> f32,
        outPolyRef: *mut DtPolyRef,
        outVector: *mut DtVector,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_moveAlongSurface(
        _self: *const DtNavMeshQuery,
        start_ref: DtPolyRef,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        filter: *const DtQueryFilter,
        result_pos: *mut DtVector,
        visited: *mut DtPolyRef,
        visited_count: *mut i32,
        max_visited_size: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findPolysAroundCircle(
        _self: *const DtNavMeshQuery,
        start_ref: DtPolyRef,
        center: *const DtVector,
        radius: f32,
        filter: *const DtQueryFilter,
        result_ref: *mut DtPolyRef,
        result_parent: *mut DtPolyRef,
        result_cost: *mut f32,
        result_count: *mut i32,
        max_result: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_free(_self: *mut DtNavMeshQuery);

}
