//! The e-ink viewer as a library: plush_memory_viewer (src/main.rs) is one
//! app built on it; another AppLoad app can reuse the same event protocol,
//! handwriting and rotation handling and add its own behavior via
//! `viewer::Hooks`.

pub mod ink;
pub mod qtfb;
pub mod script;
pub mod viewer;
