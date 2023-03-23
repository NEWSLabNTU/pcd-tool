use clap::Parser;
use std::path::PathBuf;

#[derive(Debug, Clone, Parser)]
pub enum Opts {
    Info(Info),
    Convert(Convert),
}

#[derive(Debug, Clone, Parser)]
pub struct Info {
    pub file: PathBuf,
}

#[derive(Debug, Clone, Parser)]
/// Point cloud file conversion.
pub struct Convert {
    pub input_path: PathBuf,
    pub output_path: PathBuf,
}
