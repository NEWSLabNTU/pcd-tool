# pcd-tool

The `pcd-tool` is a swiss knife for point cloud data processing. It
supports data summary, data dump and file conversion.

## Installation

To install from the most recent version,

```sh
cargo install -f --git https://github.com/NEWSLabNTU/pcd-tool.git
```

## Usage

Show the schema and statistics.

```sh
pcd-tool info input.pcd
```


Show the data in terminal.

```sh
pcd-tool dump input.pcd
```

Convert from a Velodyne .pcap file to PCD files.

```sh
pcd-tool convert input.pcap output_dir/
```

## License

The software is distributed under MIT license. Please check the
[LICENSE](LICENSE) file.
