//! Generate a Vertex keypair for one node, as base58 DER.
//!
//! The fixture generators (`vertex_ros2/test/gen_test_keys.sh` and the
//! per-simulation `gen_peersN.sh`) shell out to this to mint peer identities.
//! It lives here rather than in `tashi-vertex-rs` so the workspace needs
//! nothing but the pinned `tashi-vertex` crate from crates.io — no sibling
//! checkout to run the test suites or bring up a simulation.
//!
//!     cargo run --quiet --example key-generate
//!     Secret: 3d1RiRMXUV...
//!     Public: aSq9DsNNvGhY...

use tashi_vertex::KeySecret;

fn main() {
    let secret = KeySecret::generate();
    let public = secret.public();

    println!("Secret: {secret}");
    println!("Public: {public}");
}
