//! `vertex_node` — the ROS 2 entry point for the Vertex consensus integration.
//!
//! Starts a single `rclrs` node that owns the ROS-facing contract (see
//! [`node`]) and drives the tested [`vertex_core`] controller. The node begins
//! in the `Unconfigured` lifecycle state; operators drive it with the
//! `/vertex/transition` service (`configure` → `activate` → ...). See the README
//! for a full launch + bring-up walkthrough.
//!
//! Built by colcon (ament_cargo). All ROS-independent logic — and the test
//! suite — live in the `vertex_core` crate.

mod node;
mod params;

use node::VertexNode;
// These methods come from extension traits that must be in scope:
// `create_basic_executor` (on Context) and `first_error` (on the spin result).
use rclrs::{CreateBasicExecutor, RclrsErrorFilter};

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // The node registers its publishers, subscription, services, and the
    // diagnostics timer on construction, and starts in `Unconfigured`.
    let _node = VertexNode::new(&executor)?;

    // Spin until shutdown (Ctrl-C / SIGINT). The engine itself runs on its own
    // thread inside the controller; this just services ROS callbacks.
    executor.spin(rclrs::SpinOptions::default()).first_error()?;
    Ok(())
}
