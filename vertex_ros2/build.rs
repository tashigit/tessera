//! Bake an rpath to `libtashi-vertex` into the `vertex_node` binary so the
//! colcon-installed executable loads without an `LD_LIBRARY_PATH` export
//! (consumer friction).
//!
//! Same problem and same fix as `vertex_core/build.rs`: `tashi-vertex`'s own
//! build script adds an rpath only to *its own* binaries, so downstream
//! binaries link `@rpath/libtashi-vertex.*` with no matching search path. We
//! locate the library the `tashi-vertex` build produced under this package's
//! cargo target directory and add an absolute rpath to it. Under colcon that
//! target directory is `<ws>/build/vertex_ros2/<profile>`, which persists for
//! the life of the workspace, so the installed binary keeps resolving.
//! Relative `$ORIGIN`-style fallbacks cover a library copied next to the
//! binary or into the sibling `lib/` directory (e.g. by a release packaging
//! step). Moving a built workspace still needs the env-var fallback in
//! `docker/entrypoint.sh`.

use std::fs;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let lib_name = match target_os.as_str() {
        "macos" => "libtashi-vertex.dylib",
        "windows" => "tashi-vertex.dll",
        _ => "libtashi-vertex.so",
    };

    // OUT_DIR = <target>/<profile>/build/vertex_ros2-XXXX/out
    // ancestors: out -> vertex_ros2-XXXX -> build -> <profile>
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let Some(profile_dir) = out_dir.ancestors().nth(3) else {
        return;
    };
    let build_dir = profile_dir.join("build");

    if let Ok(entries) = fs::read_dir(&build_dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            let is_vertex = p
                .file_name()
                .and_then(|s| s.to_str())
                .is_some_and(|n| n.starts_with("tashi-vertex-"));
            if !is_vertex {
                continue;
            }
            let lib_dir = p.join("out").join("lib");
            if lib_dir.join(lib_name).exists() {
                println!("cargo:rustc-link-search=native={}", lib_dir.display());
                if target_os == "macos" || target_os == "linux" {
                    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
                }
            }
        }
    }

    // Relative fallbacks: a library placed next to the installed binary
    // (install/<pkg>/lib/<pkg>/) or one directory up (install/<pkg>/lib/).
    if target_os == "macos" {
        println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path/../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path");
    } else if target_os == "linux" {
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    }
}
