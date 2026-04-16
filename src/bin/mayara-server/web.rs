// Thin compatibility shim.
//
// The full server implementation has been moved to `mayara::server` (src/lib/server/).
// All public types used by main.rs are re-exported here so the binary compiles
// without any changes to main.rs.

pub use crate::server::{Web, WebError, generate_openapi_json};
