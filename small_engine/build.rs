// Compiles the vision and grSim .proto files (pure Rust, no protoc needed).

fn main() {
    protobuf_codegen::Codegen::new()
        .pure()
        .includes(["proto/"])
        .inputs([
            "proto/ssl_vision_wrapper.proto",
            "proto/ssl_vision_detection.proto",
            "proto/ssl_vision_geometry.proto",
            "proto/grSim_Packet.proto",
            "proto/grSim_Commands.proto",
            "proto/grSim_Replacement.proto",
        ])
        .cargo_out_dir("protos")
        .run_from_script();
}
