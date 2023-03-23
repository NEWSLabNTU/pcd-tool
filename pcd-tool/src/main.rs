mod convert;
mod info;
mod opts;
mod types;
mod utils;

use anyhow::Result;
use clap::Parser;
use opts::{Info, Opts};

fn main() -> Result<()> {
    let opts = Opts::parse();

    match opts {
        Opts::Info(Info { file }) => {
            crate::info::info(file)?;
        }
        Opts::Convert(args) => {
            crate::convert::convert(args)?;
        } /* Opts::DeviceTime { file } => {
           *     crate::utils::time(file)?;
           * } */
    }

    Ok(())
}
