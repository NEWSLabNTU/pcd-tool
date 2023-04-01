use clap::ValueEnum;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, ValueEnum)]
pub enum FileFormat {
    #[value(name = "pcd.libpcl")]
    LibpclPcd,
    #[value(name = "pcd.newslab")]
    NewslabPcd,
    #[value(name = "pcap.velodyne")]
    VelodynePcap,
}
