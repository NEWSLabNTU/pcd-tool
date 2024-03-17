use clap::ValueEnum;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, ValueEnum)]
pub enum FileFormat {
    #[value(name = "pcd.libpcl")]
    LibpclPcd,
    #[value(name = "pcd.newslab")]
    NewslabPcd,
    #[value(name = "pcap.velodyne")]
    VelodynePcap,
    #[value(name = "raw.bin")]
    RawBin,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BinPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}
