use crate::types::FileFormat;
use anyhow::bail;
use clap::Parser;
use std::{path::PathBuf, str::FromStr};
use velodyne_lidar::{ProductID, ReturnMode};

/// The Swiss army knife for point cloud data.
#[derive(Debug, Clone, Parser)]
pub enum Opts {
    Info(Info),
    Dump(Dump),
    Show(Show),
    Convert(Convert),
}

/// Dump the content of the point cloud file.
#[derive(Debug, Clone, Parser)]
pub struct Dump {
    /// The input file format.
    #[clap(short, long)]
    pub format: Option<FileFormat>,

    /// The input file path.
    pub input: PathBuf,
}

/// Show the point cloud data in a graphics user interface.
#[derive(Debug, Clone, Parser)]
pub struct Show {
    /// The input file format.
    #[clap(short, long)]
    pub format: Option<FileFormat>,

    /// The input file path.
    pub input: PathBuf,

    /// The Velodyne LiDAR model name.
    #[clap(long)]
    pub velodyne_model: Option<ProductID>,

    /// The return mode configured on the Velodyne LiDAR.
    #[clap(long)]
    pub velodyne_return_mode: Option<VelodyneReturnMode>,
}

/// Show the information of a point cloud file.
#[derive(Debug, Clone, Parser)]
pub struct Info {
    /// The file to be inspected.
    pub file: PathBuf,
}

/// Convert point cloud file format.
#[derive(Debug, Clone, Parser)]
pub struct Convert {
    /// The input file format.
    #[clap(short, long)]
    pub from: Option<FileFormat>,

    /// The output file format.
    #[clap(short, long)]
    pub to: Option<FileFormat>,

    /// The input file path.
    #[clap(short, long)]
    pub input: PathBuf,

    /// The output file path.
    #[clap(short, long)]
    pub output: PathBuf,

    /// The Velodyne LiDAR model name.
    #[clap(long)]
    pub velodyne_model: Option<ProductID>,

    /// The return mode configured on the Velodyne LiDAR.
    #[clap(long)]
    pub velodyne_return_mode: Option<VelodyneReturnMode>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VelodyneReturnMode(pub ReturnMode);

impl FromStr for VelodyneReturnMode {
    type Err = anyhow::Error;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let mode = match text {
            "strongest" => ReturnMode::Strongest,
            "last" => ReturnMode::Last,
            "dual" => ReturnMode::Dual,
            _ => bail!("invalid return mode '{text}'"),
        };

        Ok(Self(mode))
    }
}
