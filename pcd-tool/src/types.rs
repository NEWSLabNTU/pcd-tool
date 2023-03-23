#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FileFormat {
    LibpclPcd,
    NewslabPcd,
    Pcap,
}
