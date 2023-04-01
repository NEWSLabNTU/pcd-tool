use anyhow::bail;
use clap::Parser;
use std::{path::PathBuf, str::FromStr};
use velodyne_lidar::{ProductID, ReturnMode};

use crate::types::FileFormat;

#[derive(Debug, Clone, Parser)]
pub enum Opts {
    Info(Info),
    Convert(Convert),
}

#[derive(Debug, Clone, Parser)]
pub struct Info {
    pub file: PathBuf,
}

/// Point cloud file conversion.
#[derive(Debug, Clone, Parser)]
pub struct Convert {
    #[clap(short, long)]
    pub from: Option<FileFormat>,

    #[clap(short, long)]
    pub to: Option<FileFormat>,

    #[clap(short, long)]
    pub input: PathBuf,

    #[clap(short, long)]
    pub output: PathBuf,

    #[clap(long)]
    pub velodyne_model: Option<ProductID>,

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
