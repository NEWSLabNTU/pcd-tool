# pcd-tool

The `pcd-tool` is a swiss knife for point cloud data processing. It
supports data summary, data dump and file conversion.

## Installation

To install from the most recent version,

```sh
cargo install -f --git https://github.com/NEWSLabNTU/pcd-tool.git
```

## Usage

### Show the schema and statistics of a .pcd file

```sh
pcd-tool info input.pcd
```


### Convert a Velodyne .pcap to a directory of .pcd files

```sh
cargo run --release -- convert \
    -i input.pcap \
    -o output/ \
    -f pcap.velodyne \
    -t pcd.libpcl \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest
```

`-f` and `-t` are input and output format options. Supported formats
include:

- pcd.libpcl
- pcd.newslab
- pcap.velodyne
- raw.bin


### Visualize a Velodyne .pcap file

```sh
cargo run --release -- show \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest \
    input.pcap
```


### Visualize a .pcd file

```sh
cargo run --release -- show input.pcd
```


### Dump the content of a Velodyne .pcap file

```sh
cargo run --release -- dump \
    --velodyne-model VLP32C \
    --velodyne-return-mode strongest \
    input.pcap
```


### Visualize the content of a .pcd file

```sh
cargo run --release -- dump input.pcd
```

## License

The software is distributed under MIT license. Please check the
[LICENSE](LICENSE) file.
