fn main() {
    cc::Build::new()
        .cpp(true)
        .include("third_party/meshoptimizer")
        .file("third_party/meshoptimizer/simplifier.cpp")
        .file("third_party/meshoptimizer/allocator.cpp")
        .compile("meshoptimizer");
    println!("cargo:rerun-if-changed=third_party/meshoptimizer/simplifier.cpp");
    println!("cargo:rerun-if-changed=third_party/meshoptimizer/allocator.cpp");
    println!("cargo:rerun-if-changed=third_party/meshoptimizer/meshoptimizer.h");
}
