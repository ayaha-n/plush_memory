//! plush_memory e-ink viewer: shows the plush toy's touch memories on the
//! reMarkable Paper Pro. The engine (event protocol, handwriting, rotation)
//! is the library's `viewer` module; plush_memory adds nothing to it.

use plush_memory_viewer::viewer;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--preview") {
        viewer::preview(&args[2..]);
        return;
    }
    viewer::run("plush_memory_viewer", &mut viewer::NoHooks);
}
