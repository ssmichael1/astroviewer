fn main() {
    match std::env::var("CARGO_CFG_TARGET_OS").as_deref() {
        // On macOS the SVBony SDK links `libusb-1.0.0.dylib` dynamically with an
        // install name of `@rpath/libusb-1.0.0.dylib`. Add rpaths so the binary
        // finds the dylib whether it sits next to the executable (plain folder
        // layout) or in `Contents/Frameworks` of a `.app` bundle.
        Ok("macos") => {
            println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path");
            println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../Frameworks");
        }
        // On Linux the SDK links `libusb-1.0.so` dynamically. `$ORIGIN` makes the
        // loader search next to the executable, so the bundled libusb resolves.
        Ok("linux") => {
            println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
        }
        _ => {}
    }
}
