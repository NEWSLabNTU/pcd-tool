use crate::types::FileFormat;
use std::path::Path;

// use crate::types::LidarType;
// use anyhow::Result;
// use console::Term;
// use dialoguer::Select;
// use iterator_ext::IteratorExt;
// use lidar_utils::velodyne::{self, FrameConverter as _};
// use std::{iter, path::Path};

pub fn guess_file_format<P>(file: P) -> Option<FileFormat>
where
    P: AsRef<Path>,
{
    let file = file.as_ref();
    let file_name = file.file_name()?.to_str()?;

    let format = if file_name.ends_with(".newslab.pcd") {
        FileFormat::NewslabPcd
    } else if file_name.ends_with(".pcd") {
        FileFormat::LibpclPcd
    } else if file_name.ends_with(".pcap") {
        FileFormat::VelodynePcap
    } else {
        return None;
    };

    Some(format)
}

// pub fn time(file: impl AsRef<Path>) -> Result<()> {
//     let term = Term::stdout();
//     let lidar_type = {
//         let choice = Select::new()
//             .with_prompt("What kind of lidar was used to collect this pcap?")
//             .items(&["Vlp16", "Vlp32"])
//             .default(1)
//             .interact_on(&term)?;

//         match choice {
//             0 => LidarType::Vlp16,
//             1 => LidarType::Vlp32,
//             _ => unreachable!(),
//         }
//     };

//     // prepare pcap capture handlers
//     let mut cap = pcap::Capture::from_file(file)?;
//     let mut frame_converter = match lidar_type {
//         LidarType::Vlp16 => velodyne::Dynamic_FrameConverter::from_config(
//             velodyne::Config::puck_hires_dynamic_return(velodyne::ReturnMode::StrongestReturn)
//                 .into_dyn(),
//         ),
//         LidarType::Vlp32 => velodyne::Dynamic_FrameConverter::from_config(
//             velodyne::Config::vlp_32c_dynamic_return(velodyne::ReturnMode::StrongestReturn)
//                 .into_dyn(),
//         ),
//     };

//     iter::from_fn(|| {
//         let packet = cap.next().ok()?;
//         let packet = velodyne::DataPacket::from_pcap(&packet).ok();
//         Some(packet)
//     })
//     .flatten()
//     .map(Ok)
//     .try_flat_map(|packet| frame_converter.convert::<velodyne::DataPacket>(packet))
//     .enumerate()
//     .map(|(index, frame)| -> Result<_> { Ok((index, frame?)) })
//     .try_for_each(|x| -> Result<()> {
//         let (idx, frame) = x?;
//         let device_timestamp = frame
//             .into_iter()
//             .map(|point| {
//                 let nanos = point.timestamp().get::<uom::si::time::nanosecond>() as u64;
//                 std::time::Duration::from_nanos(nanos)
//             })
//             .min()
//             .unwrap();
//         println!(
//             "Frame: {}, device_time: {:>10.4} (sec)",
//             idx,
//             device_timestamp.as_secs_f64()
//         );
//         Ok(())
//     })?;
//     Ok(())
// }
