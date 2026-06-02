//! Make downstream binaries (unit tests, integration tests, and any consumer
//! binary) able to find `libtashi-vertex.dylib`/`.so` at runtime.
//!
//! `tashi-vertex`'s own build script adds an `rpath` via `cargo:rustc-link-arg`,
//! but that only applies to *its own* test/example binaries — it does not
//! propagate to downstream crates. So our test binaries link against
//! `@rpath/libtashi-vertex.dylib` with no matching `LC_RPATH` and fail to load.
//! Here we locate the dylib the `tashi-vertex` build produced under the shared
//! target directory and add an absolute `rpath` to it.

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

    // OUT_DIR = <target>/<profile>/build/vertex_core-XXXX/out
    // ancestors: out -> vertex_core-XXXX -> build -> <profile>
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let Some(profile_dir) = out_dir.ancestors().nth(3) else {
        return;
    };
    let build_dir = profile_dir.join("build");

    let mut lib_dirs: Vec<PathBuf> = Vec::new();
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
                lib_dirs.push(lib_dir);
            }
        }
    }

    for dir in &lib_dirs {
        println!("cargo:rustc-link-search=native={}", dir.display());
        // ELF and Mach-O both take an absolute -rpath; only the relative token
        // syntax ($ORIGIN vs @loader_path) differs (handled below).
        if target_os == "macos" || target_os == "linux" {
            println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
        }
    }

    // Relative fallbacks mirroring tashi-vertex's own rpaths, in case the dylib
    // is also copied next to the binary or one directory up.
    if target_os == "macos" {
        println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path/../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path");
    } else if target_os == "linux" {
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    }
}
