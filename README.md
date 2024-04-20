[![Actions Status](https://github.com/ohchase/divert/workflows/Continuous%20integration/badge.svg)](https://github.com/ohchase/divert/actions)
[![Crate](https://img.shields.io/crates/v/divert.svg)](https://crates.io/crates/divert)

# Divert
Rust bindings for [Recast Navigation](https://github.com/recastnavigation/recastnavigation).

## Purpose
Provide safe bindings to [Recast Navigation](https://github.com/recastnavigation/recastnavigation) to allow for 3d navigation in a rust application.

## How to Build
```
git clone --recurse-submodules https://github.com/ohchase/divert.git
cargo build
```

## Use Case
Refer to `examples/pathfinding.rs` for a demonstration of loading geometry generated with [Trinity Core](https://github.com/TrinityCore/TrinityCore). In the below, Proof of Concept, section the paths generated are projected to in-game space. In this repository the resources for generating paths is provided, but drawing/projecting points in the game is not in scope of this project. No questions or issues should be opened requesting help or information about video game specific applications.

```
cargo run --example path_finding
   Compiling divert v0.1.0 (...\divert)
    Finished dev [unoptimized + debuginfo] target(s) in 0.99s
     Running `target\debug\examples\path_finding.exe`
 INFO  path_finding > [TrinityNavigator] Generating path from DtVector { y: 4350.97, z: 2.25, x: -2051.9 } to DtVector { y: 4894.67, z: 2.21, x: -1916.12 }
 INFO  path_finding > [NavEngine] Adding tile (35, 23)
 INFO  path_finding > [NavEngine] Adding tile (35, 22)
 INFO  path_finding > [TrinityNavigator] Successfully generated path of len: 256
 INFO  path_finding > DtVector { y: 4350.97, z: 2.6134748, x: -2051.9 }
 INFO  path_finding > DtVector { y: 4352.9404, z: 3.0786226, x: -2051.5564 }
 INFO  path_finding > DtVector { y: 4354.9106, z: 3.0870965, x: -2051.213 }
 INFO  path_finding > DtVector { y: 4356.881, z: 2.9974942, x: -2050.8694 }
```