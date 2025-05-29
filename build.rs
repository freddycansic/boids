fn main() {
    #[cfg(target_os = "linux")]
    {
        if let Ok(version) = std::fs::read_to_string("/proc/version") {
            if version.contains("microsoft-standard-WSL2") {
                println!("cargo:rustc-env=WGPU_ALLOW_UNDERLYING_NONCOMPLIANT_ADAPTER=1");
            }
        }
    }
}
