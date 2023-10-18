# Point Cloud Tool

The Swiss army knife for point cloud data. It supports the following features:

- Format conversion
- 3D Visualization
- Text dump

## Usage

### Convert a Velodyne .pcap to a directory of .pcd files

```rust
cargo run --release -- convert \
    -i input.pcap \
    -o output/ \
    -t pcd.libpcl \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest
```


### Visualize a Velodyne .pcap file

```rust
cargo run --release -- show \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest \
    input.pcap
```


### Visualize a .pcd file

```rust
cargo run --release -- show input.pcd
```


### Dump the content of a Velodyne .pcap file

```rust
cargo run --release -- dump \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest \
    input.pcap
```


### Visualize the content of a .pcd file

```rust
cargo run --release -- dump input.pcd
```

