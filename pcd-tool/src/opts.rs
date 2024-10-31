use crate::types::FileFormat;
use clap::Parser;
use eyre::bail;
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

    /// The Velodyne LiDAR model name.
    #[clap(long)]
    pub velodyne_model: Option<ProductID>,

    /// The return mode configured on the Velodyne LiDAR.
    #[clap(long)]
    pub velodyne_return_mode: Option<VelodyneReturnMode>,
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

    /// The first frame number to start to convert.
    ///
    /// If positive number is provided, it's frame number starting
    /// from 1. If negative number is provided, it's the number
    /// counted from the last frame.
    #[clap(long, default_value = "1", value_parser = parse_start_frame)]
    pub start: StartFrame,

    /// The last frame number or the number of frames to be converted.
    ///
    /// If the number is prefixed with a '+', the value is treated as
    /// the frame count. if a positive number without '+' prefix is
    /// provided, it's frame number starting from 1. If negative
    /// number is provided, it's the number counted from the last
    /// frame.
    #[clap(long, default_value = "-1", value_parser = parse_end_or_count)]
    pub end: EndFrame,

    /// Apply point transformation described in the file.
    ///
    /// The transformation file format is defined in tftk.
    /// https://github.com/NEWSLabNTU/tftk#file-format
    #[clap(long)]
    pub transform_file: Option<PathBuf>,

    /// Apply point transformation according to the text description.
    ///
    /// The transformation text format is defined in tftk.
    /// https://github.com/NEWSLabNTU/tftk#file-format
    #[clap(long)]
    pub transform: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VelodyneReturnMode(pub ReturnMode);

impl FromStr for VelodyneReturnMode {
    type Err = eyre::Error;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StartFrame {
    Forward(usize),
    Backward(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EndFrame {
    Forward(usize),
    Backward(usize),
    Count(usize),
}

fn parse_end_or_count(arg: &str) -> Result<EndFrame, String> {
    macro_rules! bail {
        () => {
            return Err(format!("invalid end-or-count '{arg}'"));
        };
    }

    let arg = if let Some(value) = arg.strip_prefix('+') {
        let Ok(count) = value.parse::<usize>() else {
            bail!();
        };

        EndFrame::Count(count)
    } else if let Some(value) = arg.strip_prefix('-') {
        let Ok(value) = value.parse::<usize>() else {
            bail!();
        };

        if value == 0 {
            return Err("The value provided to --end must be non-zero.".to_string());
        }

        EndFrame::Backward(value)
    } else {
        let Ok(value) = arg.parse::<usize>() else {
            bail!();
        };

        if value == 0 {
            return Err("The value provided to --end must be non-zero.".to_string());
        }

        EndFrame::Forward(value)
    };

    Ok(arg)
}

fn parse_start_frame(arg: &str) -> Result<StartFrame, String> {
    macro_rules! bail {
        () => {
            return Err(format!("invalid end-or-count '{arg}'"));
        };
    }

    let arg = if let Some(value) = arg.strip_prefix('-') {
        let Ok(value) = value.parse::<usize>() else {
            bail!();
        };

        if value == 0 {
            return Err("The value provided to --end must be non-zero.".to_string());
        }

        StartFrame::Backward(value)
    } else {
        let Ok(value) = arg.parse::<usize>() else {
            bail!();
        };

        if value == 0 {
            return Err("The value provided to --end must be non-zero.".to_string());
        }

        StartFrame::Forward(value)
    };

    Ok(arg)
}
