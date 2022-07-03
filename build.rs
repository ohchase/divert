use std::path::Path;

// Example custom build script.
fn main() {
    // Tell Cargo that if the given file changes, to rerun this build script.
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/extern.cpp");

    // Use the `cc` crate to build a C file and statically link it.
    cc::Build::new()
        .cpp(true)
        .define("DT_POLYREF64", "1")
        .includes(Some(Path::new("recastnavigation/Detour/Include")))
        .file("recastnavigation/Detour/Source/DetourAlloc.cpp")
        .file("recastnavigation/Detour/Source/DetourAssert.cpp")
        .file("recastnavigation/Detour/Source/DetourCommon.cpp")
        .file("recastnavigation/Detour/Source/DetourNavMesh.cpp")
        .file("recastnavigation/Detour/Source/DetourNavMeshBuilder.cpp")
        .file("recastnavigation/Detour/Source/DetourNavMeshQuery.cpp")
        .file("recastnavigation/Detour/Source/DetourNode.cpp")
        .file("src/extern.cpp")
        .compile("detour");
}
