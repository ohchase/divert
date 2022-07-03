use byteorder::{LittleEndian, ReadBytesExt};
use log::{info, LevelFilter};

use divert::{
    DivertError, DivertResult, DtStraightPathFlags, NavMesh, NavMeshParams, NavMeshQuery, PolyRef,
    QueryFilter, TileRef, Vector,
};
use rand::Rng;
use thiserror::Error;

use std::{
    cmp,
    collections::HashSet,
    error::Error,
    fs::File,
    io::{self, Read, Seek, SeekFrom},
    sync::Mutex,
};

trait DataProvider: Send + Sync {
    fn read_map_params(&self) -> io::Result<NavMeshParams>;

    fn read_tile_data(&self, tile_x: &i32, tile_y: &i32) -> io::Result<Vec<u8>>;
}

struct TrinityDataProvider {
    map_id: i32,
}

impl DataProvider for TrinityDataProvider {
    fn read_map_params(&self) -> io::Result<NavMeshParams> {
        let mut map_params_file =
            File::open(format!("resources/geometry/{:03}.mmap", self.map_id))?;

        let origin_x = map_params_file.read_f32::<LittleEndian>()?;
        let origin_y = map_params_file.read_f32::<LittleEndian>()?;
        let origin_z = map_params_file.read_f32::<LittleEndian>()?;

        let tile_width = map_params_file.read_f32::<LittleEndian>()?;
        let tile_height = map_params_file.read_f32::<LittleEndian>()?;

        let max_tiles = map_params_file.read_i32::<LittleEndian>()?;
        let max_polys = map_params_file.read_i32::<LittleEndian>()?;

        Ok(NavMeshParams {
            origin: [origin_x, origin_y, origin_z],
            tile_width,
            tile_height,
            max_tiles,
            max_polys,
        })
    }

    fn read_tile_data(&self, tile_x: &i32, tile_y: &i32) -> io::Result<Vec<u8>> {
        let mut tile_file = File::open(format!(
            "resources/geometry/{:03}{:02}{:02}.mmtile",
            self.map_id, tile_x, tile_y
        ))?;

        tile_file.seek(SeekFrom::Current(20))?;
        let mut tile_data = Vec::new();
        tile_file.read_to_end(&mut tile_data)?;
        Ok(tile_data)
    }
}

struct PathFindingSettings {
    pub max_path: i32,
    pub max_smooth_path: i32,
    pub max_steer_points: i32,
    pub steer_target_radius: f32,
    pub steer_target_height: f32,
    pub max_move_visits: i32,
    pub smooth_step_size: f32,
}

impl Default for PathFindingSettings {
    fn default() -> Self {
        Self {
            max_path: 64,
            max_smooth_path: 256,
            max_steer_points: 3,
            steer_target_radius: 0.3,
            steer_target_height: 1000.0,
            max_move_visits: 16,
            smooth_step_size: 2.0,
        }
    }
}

struct TrinityNavEngine<'a> {
    nav_mesh: NavMesh<'a>,
    data_provider: Box<dyn DataProvider>,
    loaded_tiles: HashSet<u32>,
}

impl<'a> TrinityNavEngine<'a> {
    pub fn new(data_provider: Box<dyn DataProvider>) -> Result<Self, Box<dyn Error>> {
        let map_params = data_provider.read_map_params()?;
        let nav_mesh = NavMesh::new(&map_params)?;

        Ok(Self {
            nav_mesh,
            data_provider,
            loaded_tiles: HashSet::new(),
        })
    }

    fn packed_tile_id(tile_x: &i32, tile_y: &i32) -> u32 {
        (tile_x << 16 | tile_y) as u32
    }

    fn has_tile(&self, tile_x: &i32, tile_y: &i32) -> bool {
        self.loaded_tiles
            .contains(&Self::packed_tile_id(tile_x, tile_y))
    }

    pub fn create_query<'b>(&self, max_nodes: i32) -> DivertResult<NavMeshQuery<'b>> {
        self.nav_mesh.create_query(max_nodes)
    }

    pub fn add_tile(&mut self, tile_x: &i32, tile_y: &i32) -> DivertResult<TileRef> {
        if self.has_tile(tile_x, tile_y) {
            info!(
                "[NavEngine] Add tile called for already loaded tile ({}, {})",
                tile_x, tile_y
            );

            // Todo: Expose nav mesh get tile ref by tile_x, tile_y?
            return Ok(TileRef::default());
        }

        info!("[NavEngine] Adding tile ({}, {})", tile_x, tile_y);
        let tile_ref = self
            .nav_mesh
            .add_tile(self.data_provider.read_tile_data(tile_x, tile_y).unwrap())?;
        self.loaded_tiles
            .insert(Self::packed_tile_id(tile_x, tile_y));
        Ok(tile_ref)
    }
}

lazy_static::lazy_static! {
    static ref NAV_ENGINE: Mutex<TrinityNavEngine<'static>> = Mutex::new(
        TrinityNavEngine::new(Box::new(TrinityDataProvider { map_id: 530 }))
        .expect("Unable to initialize global navigation engine, example resources not available."));
}

#[derive(Error, Debug)]
enum NavigationError {
    #[error("Divert related error")]
    Divert(#[from] DivertError),
    #[error("usize to i32 convert error")]
    UsizeTruncation,
    #[error("Poly path did not have a minimum of one polygon.")]
    EmptyPolygonPath,
    #[error("Unable to fix up corridor. Could not find common.")]
    FixUpCorridor,
}

type NavigationResult<T> = std::result::Result<T, NavigationError>;

struct TrinityNavigator<'a> {
    query: NavMeshQuery<'a>,
    query_filter: QueryFilter<'a>,
    settings: PathFindingSettings,
}

impl<'a> TrinityNavigator<'a> {
    fn new(query_filter: QueryFilter<'a>) -> DivertResult<Self> {
        let nav_engine = NAV_ENGINE
            .lock()
            .expect("Global Nav Engine Mutex has been poisoned");
        let query = nav_engine.create_query(2048)?;

        Ok(Self {
            query,
            query_filter,
            settings: PathFindingSettings::default(),
        })
    }

    fn in_range(source: &Vector, destination: &Vector, radius: f32, height: f32) -> bool {
        let dx = destination.y - source.y;
        let dy = destination.z - source.z;
        let dz = destination.x - source.x;
        (dx * dx + dz * dz) < radius * radius && dy.abs() < height
    }

    fn find_common(this: &[PolyRef], other: &[PolyRef]) -> Option<(usize, usize)> {
        for i in (0..this.len()).rev() {
            for j in (0..other.len()).rev() {
                if this[i] == other[j] {
                    return Some((i, j));
                }
            }
        }
        None
    }

    fn world_to_trinity(world_x: f32, world_y: f32) -> (i32, i32) {
        (
            (32.0 - (world_x / 533.3333)) as i32,
            (32.0 - (world_y / 533.3333)) as i32,
        )
    }

    fn fix_up_corridor(path: &mut Vec<PolyRef>, visited: &[PolyRef]) -> NavigationResult<()> {
        let (furthest_path, furthest_visited) =
            Self::find_common(path, visited).ok_or(NavigationError::FixUpCorridor)?;

        let required = visited.len() - furthest_visited;
        let orig = cmp::min(furthest_path + 1, path.len());
        let mut size = cmp::max(0, path.len() - orig);

        if required + size > path.capacity() {
            size = path.capacity() - required;
        }

        unsafe {
            path.set_len(required + size);
        };

        if size > 0 {
            unsafe {
                std::ptr::copy(
                    path.as_mut_ptr().offset(
                        orig.try_into()
                            .map_err(|_| NavigationError::UsizeTruncation)?,
                    ),
                    path.as_mut_ptr().offset(
                        required
                            .try_into()
                            .map_err(|_| NavigationError::UsizeTruncation)?,
                    ),
                    size,
                );
            }
        }

        for i in 0..required {
            path[i] = visited[(visited.len() - 1) - i];
        }

        Ok(())
    }

    fn find_nearest_poly(&self, position: &Vector) -> NavigationResult<(PolyRef, Vector)> {
        let extents = Vector::from_yzx(3.0f32, 5.0f32, 3.0f32);
        self.query
            .find_nearest_poly(position, &extents, &self.query_filter)
            .map_err(NavigationError::Divert)
    }

    fn find_steer_target(
        &self,
        start_pos: &Vector,
        end_pos: &Vector,
        poly_path: &[PolyRef],
    ) -> NavigationResult<Option<(Vector, DtStraightPathFlags, PolyRef)>> {
        let steer_points = self
            .query
            .find_straight_path(
                start_pos,
                end_pos,
                poly_path,
                self.settings.max_steer_points,
                0,
            )
            .map_err(NavigationError::Divert)?;

        for (mut steer_point, steer_flag, steer_poly) in steer_points {
            if steer_flag.contains(DtStraightPathFlags::OFFMESH_CONNECTION)
                || !Self::in_range(
                    &steer_point,
                    start_pos,
                    self.settings.steer_target_radius,
                    self.settings.steer_target_height,
                )
            {
                steer_point.z = start_pos.z;
                return Ok(Some((steer_point, steer_flag, steer_poly)));
            }
        }

        Ok(None)
    }

    fn find_smooth_path(
        &self,
        start_pos: &Vector,
        end_pos: &Vector,
        mut polygons: Vec<PolyRef>,
    ) -> NavigationResult<Vec<Vector>> {
        let capacity = self
            .settings
            .max_smooth_path
            .try_into()
            .map_err(|_| NavigationError::UsizeTruncation)?;
        let mut smooth_path = Vec::with_capacity(capacity);

        let mut iter_pos = self
            .query
            .closest_point_on_poly_boundary(
                *(polygons.first().ok_or(NavigationError::EmptyPolygonPath)?),
                start_pos,
            )
            .map_err(NavigationError::Divert)?;

        let target_pos = self
            .query
            .closest_point_on_poly_boundary(
                *(polygons.last().ok_or(NavigationError::EmptyPolygonPath)?),
                end_pos,
            )
            .map_err(NavigationError::Divert)?;

        smooth_path.push(iter_pos);
        while !polygons.is_empty() && (smooth_path.len() < smooth_path.capacity()) {
            if let Some((steer_pos, steer_flags, _)) =
                self.find_steer_target(&iter_pos, &target_pos, &polygons)?
            {
                let delta = steer_pos - iter_pos;
                let mut len = delta.dot(&delta).sqrt();

                if steer_flags
                    .contains(DtStraightPathFlags::END | DtStraightPathFlags::OFFMESH_CONNECTION)
                    && len < self.settings.smooth_step_size
                {
                    len = 1.0;
                } else {
                    len = self.settings.smooth_step_size / len;
                }

                let move_target = iter_pos + (delta * len);
                let (move_result, visited) = self.query.move_along_surface(
                    polygons[0],
                    &iter_pos,
                    &move_target,
                    &self.query_filter,
                    self.settings.max_move_visits,
                )?;

                Self::fix_up_corridor(&mut polygons, &visited)?;
                let height = self
                    .query
                    .get_poly_height(polygons[0], &move_result)
                    .unwrap_or(0.0);

                iter_pos = Vector::from_yzx(move_result.y, height + 0.5, move_result.x);
                smooth_path.push(iter_pos);
            } else {
                break;
            }
        }

        Ok(smooth_path)
    }

    pub fn find_path(
        &self,
        input_start: &Vector,
        input_end: &Vector,
    ) -> NavigationResult<Vec<Vector>> {
        log::info!(
            "[TrinityNavigator] Generating path from {:?} to {:?}",
            input_start,
            input_end
        );

        {
            let nav_engine = &mut NAV_ENGINE
                .lock()
                .expect("Global Nav Engine Mutex has been poisoned");
            let start_tile = Self::world_to_trinity(input_start.x, input_start.y);
            let end_tile = Self::world_to_trinity(input_end.x, input_end.y);
            nav_engine.add_tile(&start_tile.0, &start_tile.1)?;
            nav_engine.add_tile(&end_tile.0, &end_tile.1)?;
        }

        let (start_poly, start_pos) = self.find_nearest_poly(input_start)?;
        let (end_poly, end_pos) = self.find_nearest_poly(input_end)?;

        let poly_path = self.query.find_path(
            start_poly,
            end_poly,
            &start_pos,
            &end_pos,
            &self.query_filter,
            self.settings.max_path,
        )?;

        let smooth_path = self.find_smooth_path(&start_pos, &end_pos, poly_path)?;
        log::info!(
            "[TrinityNavigator] Successfully generated path of len: {}",
            smooth_path.len()
        );

        Ok(smooth_path)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(LevelFilter::Info)
        .init();

    let mut query_filter = QueryFilter::new()?;
    query_filter.set_include_flags(1 | 8 | 4 | 2);
    query_filter.set_exclude_flags(0);

    let navigator = TrinityNavigator::new(query_filter)?;

    // Simple path
    // Shat Bridge (35,22) -> (35, 22)
    // let start_position = Vector::from_xyz(-1910.12, 5289.2, 1.424);
    // let end_position = Vector::from_xyz(-1931.90, 5099.05, 8.05);
    //navigator.find_path(&start_position, &end_position)?;
    // // Shat Bridge

    // Cross tile
    // Terrokar (35,22) -> (35, 23)
    // let start_position = Vector::from_xyz(-1916.64, 4893.65, 2.26);
    // let end_position = Vector::from_xyz(-1947.43, 4687.55, -2.09);
    //navigator.find_path(&start_position, &end_position)?;
    // Terrokar

    // Long path
    // Terrokar (35,22) -> (35, 23)
    let start_position = Vector::from_xyz(-2051.9, 4350.97, 2.25);
    let end_position = Vector::from_xyz(-1916.12, 4894.67, 2.21);
    navigator.find_path(&start_position, &end_position)?;
    // path.iter().for_each(|position| info!("{:?}", position));
    // Terrokar

    extern "C" fn frand() -> f32 {
        let mut rng = rand::thread_rng();
        rng.gen::<f32>()
    }

    match navigator
        .query
        .find_random_point(frand, &navigator.query_filter)
    {
        Ok((poly_ref, pos)) => {
            log::info!("Random Position: {:?} {:?}", poly_ref, pos);

            let points = navigator.query.find_polys_around_circle(
                poly_ref,
                &pos,
                25.0,
                &navigator.query_filter,
                256,
            )?;

            for (poly_ref, parent_ref, cost) in points.iter() {
                log::info!("Poly {:?} {:?} {}", poly_ref, parent_ref, cost);
            }
        }
        Err(err) => log::warn!("Random Position Err: {:#?}", err),
    };

    // Cross zones
    // ...

    Ok(())
}
