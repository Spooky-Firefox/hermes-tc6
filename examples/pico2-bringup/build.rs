fn main() {
    // embedded-test's linker script, only for test targets (it is a dev-dependency, so the
    // `cargo run` binary doesn't link it).
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");
    println!("cargo::rerun-if-changed=memory.x");
}
