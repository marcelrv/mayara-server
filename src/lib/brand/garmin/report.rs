use anyhow::{Error, bail};
use std::collections::HashMap;
use std::io;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::time::sleep;
use tokio_graceful_shutdown::SubsystemHandle;

use super::GarminRadarType;
use super::command::Command;
use crate::Cli;
use crate::network::create_udp_multicast_listen;
use crate::radar::settings::ControlId;
use crate::radar::spoke::GenericSpoke;
use crate::radar::{
    BYTE_LOOKUP_LENGTH, CommonRadar, Legend, Power, RadarError, RadarInfo, SharedRadars,
};
use crate::util::c_string;

/// Lookup table for converting raw pixel values to blob values
/// For xHD, we divide by 2 to make room for legend values (like Raymarine)
type PixelToBlobType = [u8; BYTE_LOOKUP_LENGTH];

fn pixel_to_blob(_legend: &Legend, is_xhd: bool) -> PixelToBlobType {
    let mut lookup = [0u8; BYTE_LOOKUP_LENGTH];

    if is_xhd {
        // xHD: divide by 2 to make room for legend values
        for j in 0..BYTE_LOOKUP_LENGTH {
            lookup[j] = (j / 2) as u8;
        }
    } else {
        // HD: binary data, no transformation needed
        for j in 0..BYTE_LOOKUP_LENGTH {
            lookup[j] = j as u8;
        }
    }

    lookup
}

/// HD spoke packet header (0x2a3)
/// Size: 52 bytes
const HD_SPOKE_HEADER_SIZE: usize = 52;

/// xHD spoke packet header (0x2a3 on port 50102)
/// Size: 36 bytes
const XHD_SPOKE_HEADER_SIZE: usize = 36;

pub(crate) struct GarminReportReceiver {
    common: CommonRadar,
    radar_type: GarminRadarType,
    report_socket: Option<UdpSocket>,
    data_socket: Option<UdpSocket>,
    command_sender: Option<Command>,
    reported_unknown: HashMap<u32, bool>,

    range_meters: u32,
    pixel_to_blob: PixelToBlobType,

    // No-transmit sector state (xHD sends mode/start/end as separate packets)
    no_tx_enabled: Option<bool>,
    no_tx_start: Option<f64>,
    no_tx_end: Option<f64>,
}

impl GarminReportReceiver {
    pub fn new(args: &Cli, info: RadarInfo, radars: SharedRadars) -> GarminReportReceiver {
        let key = info.key();

        let replay = args.replay;
        log::debug!(
            "{}: Creating GarminReportReceiver with args {:?}",
            key,
            args
        );

        // Detect radar type from spoke count
        let radar_type = if info.spokes_per_revolution > 720 {
            GarminRadarType::XHD
        } else {
            GarminRadarType::HD
        };

        let command_sender = Some(Command::new(radar_type, info.send_command_addr));

        let control_update_rx = info.control_update_subscribe();
        let blob_tx = radars.get_blob_tx();

        let pixel_to_blob = pixel_to_blob(&info.get_legend(), radar_type == GarminRadarType::XHD);

        let common = CommonRadar::new(
            args,
            key,
            info,
            radars.clone(),
            control_update_rx,
            replay,
            blob_tx,
        );

        GarminReportReceiver {
            common,
            radar_type,
            report_socket: None,
            data_socket: None,
            command_sender,
            reported_unknown: HashMap::new(),
            range_meters: 0,
            pixel_to_blob,
            no_tx_enabled: None,
            no_tx_start: None,
            no_tx_end: None,
        }
    }

    async fn start_sockets(&mut self) -> io::Result<()> {
        // Report socket (239.254.2.0:50100)
        match create_udp_multicast_listen(&self.common.info.report_addr, &self.common.info.nic_addr)
        {
            Ok(socket) => {
                self.report_socket = Some(socket);
                log::debug!(
                    "{}: {} via {}: listening for reports",
                    self.common.key,
                    &self.common.info.report_addr,
                    &self.common.info.nic_addr
                );
            }
            Err(e) => {
                log::debug!(
                    "{}: {} via {}: create multicast failed: {}",
                    self.common.key,
                    &self.common.info.report_addr,
                    &self.common.info.nic_addr,
                    e
                );
                return Err(e);
            }
        }

        // xHD uses a separate data socket
        if self.radar_type == GarminRadarType::XHD {
            match create_udp_multicast_listen(
                &self.common.info.spoke_data_addr,
                &self.common.info.nic_addr,
            ) {
                Ok(socket) => {
                    self.data_socket = Some(socket);
                    log::debug!(
                        "{}: {} via {}: listening for xHD data",
                        self.common.key,
                        &self.common.info.spoke_data_addr,
                        &self.common.info.nic_addr
                    );
                }
                Err(e) => {
                    log::debug!(
                        "{}: {} via {}: create data multicast failed: {}",
                        self.common.key,
                        &self.common.info.spoke_data_addr,
                        &self.common.info.nic_addr,
                        e
                    );
                    return Err(e);
                }
            }
        }

        Ok(())
    }

    async fn socket_loop(&mut self, subsys: &SubsystemHandle) -> Result<(), RadarError> {
        log::debug!(
            "{}: listening for reports (type={}, report_socket={}, data_socket={})",
            self.common.key,
            self.radar_type,
            self.report_socket.is_some(),
            self.data_socket.is_some()
        );
        let mut report_buf = Vec::with_capacity(10000);
        let mut data_buf = Vec::with_capacity(10000);

        loop {
            tokio::select! {
                _ = subsys.on_shutdown_requested() => {
                    log::debug!("{}: shutdown", self.common.key);
                    return Err(RadarError::Shutdown);
                },
                r = async {
                    if let Some(sock) = self.report_socket.as_ref() {
                        sock.recv_buf_from(&mut report_buf).await
                    } else {
                        std::future::pending().await
                    }
                } => {
                    match r {
                        Ok((_len, _addr)) => {
                            if let Err(e) = self.process_report(&report_buf) {
                                log::error!("{}: {}", self.common.key, e);
                            }
                            report_buf.clear();
                        }
                        Err(e) => {
                            log::error!("{}: receive error: {}", self.common.key, e);
                            return Err(RadarError::Io(e));
                        }
                    }
                },
                r = async {
                    if let Some(sock) = self.data_socket.as_ref() {
                        sock.recv_buf_from(&mut data_buf).await
                    } else {
                        std::future::pending().await
                    }
                } => {
                    match r {
                        Ok((_len, _addr)) => {
                            if let Err(e) = self.process_data(&data_buf) {
                                log::error!("{}: {}", self.common.key, e);
                            }
                            data_buf.clear();
                        }
                        Err(e) => {
                            log::error!("{}: receive error: {}", self.common.key, e);
                            return Err(RadarError::Io(e));
                        }
                    }
                },
                r = self.common.control_update_rx.recv() => {
                    match r {
                        Err(_) => {},
                        Ok(cv) => {
                            let _ = self.common.process_control_update(cv, &mut self.command_sender).await;
                        },
                    }
                }
            }
        }
    }

    pub async fn run(mut self, subsys: SubsystemHandle) -> Result<(), RadarError> {
        loop {
            if let Err(e) = self.start_sockets().await {
                log::warn!("{}: Failed to start sockets: {}", self.common.key, e);
                sleep(Duration::from_millis(1000)).await;
                continue;
            }

            match self.socket_loop(&subsys).await {
                Err(RadarError::Shutdown) => {
                    return Ok(());
                }
                _ => {
                    self.report_socket = None;
                    self.data_socket = None;
                }
            }

            sleep(Duration::from_millis(1000)).await;
        }
    }

    fn process_report(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < 8 {
            bail!("Report too short: {} bytes", data.len());
        }

        let packet_type = u32::from_le_bytes(data[0..4].try_into().unwrap());
        let len = u32::from_le_bytes(data[4..8].try_into().unwrap());

        log::trace!(
            "{}: Report packet_type={:04X} len={}",
            self.common.key,
            packet_type,
            len
        );

        match packet_type {
            // HD spoke data (on same port as reports)
            0x2a3 if self.radar_type == GarminRadarType::HD => {
                self.process_hd_spoke(data)?;
            }
            // HD status
            0x2a5 => {
                self.process_hd_status(data)?;
            }
            // HD settings
            0x2a7 => {
                log::trace!("{}: HD settings packet len={}", self.common.key, data.len());
            }
            // xHD status reports
            0x0916 => self.process_xhd_scan_speed(data)?,
            0x0919 => self.process_xhd_transmit_state(data)?,
            0x091e => self.process_xhd_range(data)?,
            0x0924 => self.process_xhd_gain_mode(data)?,
            0x0925 => self.process_xhd_gain_level(data)?,
            0x091d => self.process_xhd_gain_auto_level(data)?,
            0x0930 => self.process_xhd_bearing_alignment(data)?,
            0x0932 => self.process_xhd_crosstalk(data)?,
            0x0933 => self.process_xhd_rain_mode(data)?,
            0x0934 => self.process_xhd_rain_level(data)?,
            0x0939 => self.process_xhd_sea_mode(data)?,
            0x093a => self.process_xhd_sea_level(data)?,
            0x093b => self.process_xhd_sea_auto_level(data)?,
            0x093f => self.process_xhd_no_tx_mode(data)?,
            0x0940 => self.process_xhd_no_tx_start(data)?,
            0x0941 => self.process_xhd_no_tx_end(data)?,
            0x0942 => self.process_xhd_timed_idle_mode(data)?,
            0x0943 => self.process_xhd_timed_idle_time(data)?,
            0x0944 => self.process_xhd_timed_run_time(data)?,
            0x0992 => self.process_xhd_scanner_state(data)?,
            0x0993 => self.process_xhd_state_change(data)?,
            0x099b => self.process_xhd_message(data)?,
            _ => {
                if self.reported_unknown.get(&packet_type).is_none() {
                    log::debug!(
                        "{}: Unknown report packet_type={:04X} len={}",
                        self.common.key,
                        packet_type,
                        len
                    );
                    self.reported_unknown.insert(packet_type, true);
                }
            }
        }

        Ok(())
    }

    fn process_data(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < XHD_SPOKE_HEADER_SIZE {
            bail!("Data too short: {} bytes", data.len());
        }

        // xHD data socket receives spoke data - process all packets as spokes
        // (C++ ProcessFrame doesn't filter by packet_type on data socket)
        self.process_xhd_spoke(data)?;

        Ok(())
    }

    fn process_hd_spoke(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < HD_SPOKE_HEADER_SIZE + 4 {
            bail!("HD spoke packet too short: {} bytes", data.len());
        }

        // Parse header
        let angle = u16::from_le_bytes(data[8..10].try_into().unwrap());
        let scan_length = u16::from_le_bytes(data[10..12].try_into().unwrap()) as usize;
        let range_meters = u32::from_le_bytes(data[16..20].try_into().unwrap()) + 1;

        log::trace!(
            "{}: HD spoke: angle={} scan_length={} range={}m data_len={}",
            self.common.key,
            angle,
            scan_length,
            range_meters,
            data.len()
        );

        if self.range_meters != range_meters {
            self.range_meters = range_meters;
            self.common
                .set_value(&ControlId::Range, range_meters as f64);
        }

        // HD packs 4 spokes per packet
        let spoke_data = &data[HD_SPOKE_HEADER_SIZE..];
        let bytes_per_spoke = scan_length / 4;

        if spoke_data.len() < scan_length {
            log::warn!(
                "{}: HD spoke data too short: {} < {}",
                self.common.key,
                spoke_data.len(),
                scan_length
            );
            return Ok(());
        }

        self.common.new_spoke_message();

        for i in 0..4usize {
            let spokes = self.common.info.spokes_per_revolution;
            let spoke_angle = (angle * 2 + i as u16) % spokes;
            let start = i * bytes_per_spoke;
            let end = start + bytes_per_spoke;

            if end > spoke_data.len() {
                break;
            }

            let packed_data = &spoke_data[start..end];

            // Unpack 1-bit samples to 8-bit
            let samples = unpack_hd_spoke(packed_data, &self.pixel_to_blob);

            self.common
                .add_spoke(range_meters, spoke_angle, None, samples);
        }

        self.common.send_spoke_message();
        Ok(())
    }

    fn process_xhd_spoke(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < XHD_SPOKE_HEADER_SIZE {
            bail!("xHD spoke packet too short: {} bytes", data.len());
        }

        // Parse header (matching C++ radar_line struct)
        // Offsets: packet_type(0-3), len1(4-7), fill_1(8-9), scan_length(10-11),
        //          angle(12-13), fill_2(14-15), range_meters(16-19), display_meters(20-23),
        //          fill_3(24-25), scan_length_bytes_s(26-27), fills_4(28-29),
        //          scan_length_bytes_i(30-33), fills_5(34-35), line_data(36+)
        let angle = u16::from_le_bytes(data[12..14].try_into().unwrap());
        let range_meters = u32::from_le_bytes(data[16..20].try_into().unwrap());
        let scan_length_bytes = u16::from_le_bytes(data[26..28].try_into().unwrap()) as usize;

        // Validate packet has enough data (like C++ does)
        if data.len() < XHD_SPOKE_HEADER_SIZE + scan_length_bytes {
            log::warn!(
                "{}: xHD spoke packet incomplete: {} < {} + {}",
                self.common.key,
                data.len(),
                XHD_SPOKE_HEADER_SIZE,
                scan_length_bytes
            );
            return Ok(());
        }

        // xHD: angle is in 1/8 degree units (0-11519 for 0-1439.875 degrees)
        // Divide by 8 to get spoke index, modulo to ensure within bounds
        let spokes = self.common.info.spokes_per_revolution;
        let spoke_angle = (angle / 8) % spokes;

        log::trace!(
            "{}: xHD spoke: angle={} spoke_angle={} range={}m data_len={} scan_len={}",
            self.common.key,
            angle,
            spoke_angle,
            range_meters,
            data.len(),
            scan_length_bytes
        );

        if self.range_meters != range_meters {
            self.range_meters = range_meters;
            self.common
                .set_value(&ControlId::Range, range_meters as f64);
        }

        // Use actual received data length (like C++ does: len = received - header)
        let spoke_data = &data[XHD_SPOKE_HEADER_SIZE..];
        if spoke_data.is_empty() {
            return Ok(());
        }

        self.common.new_spoke_message();

        // xHD: 8-bit samples, apply pixel_to_blob transformation (divide by 2)
        let samples: GenericSpoke = spoke_data
            .iter()
            .map(|&v| self.pixel_to_blob[v as usize])
            .collect();

        self.common
            .add_spoke(range_meters, spoke_angle, None, samples);

        self.common.send_spoke_message();
        Ok(())
    }

    fn process_hd_status(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < 48 {
            bail!("HD status packet too short");
        }

        let scanner_state = u16::from_le_bytes(data[8..10].try_into().unwrap());
        let warmup = u16::from_le_bytes(data[10..12].try_into().unwrap());
        let range_meters = u32::from_le_bytes(data[12..16].try_into().unwrap()) + 1;
        let gain_level = data[16];
        let gain_mode = data[17];
        let sea_clutter_level = data[20];
        let sea_clutter_mode = data[21];
        let rain_clutter_level = data[24];
        let dome_offset = i16::from_le_bytes(data[28..30].try_into().unwrap());
        let crosstalk_onoff = data[31];
        let dome_speed = data[40];

        log::debug!(
            "{}: HD status: state={} warmup={} range={}m gain={}({}) sea={}({}) rain={} bearing={} ir={} speed={}",
            self.common.key,
            scanner_state,
            warmup,
            range_meters,
            gain_level,
            gain_mode,
            sea_clutter_level,
            sea_clutter_mode,
            rain_clutter_level,
            dome_offset,
            crosstalk_onoff,
            dome_speed
        );

        // Update controls
        let power = match scanner_state {
            1 => Power::Preparing, // WARMING_UP
            3 => Power::Standby,
            4 => Power::Transmit,
            5 => Power::Preparing, // SPINNING_UP
            _ => Power::Off,
        };
        self.common
            .set_value(&ControlId::Power, power as i32 as f64);

        if warmup > 0 {
            self.common.set_value(&ControlId::WarmupTime, warmup as f64);
        }

        self.common.set_value_auto(
            &ControlId::Gain,
            gain_level as f64,
            if gain_mode > 0 { 1 } else { 0 },
        );
        self.common.set_value_auto(
            &ControlId::Sea,
            sea_clutter_level as f64,
            if sea_clutter_mode == 2 { 1 } else { 0 },
        );
        self.common
            .set_value(&ControlId::Rain, rain_clutter_level as f64);
        self.common
            .set_value(&ControlId::BearingAlignment, dome_offset as f64);
        self.common
            .set_value(&ControlId::InterferenceRejection, crosstalk_onoff as f64);
        self.common
            .set_value(&ControlId::ScanSpeed, dome_speed as f64);

        Ok(())
    }

    // xHD status handlers
    fn process_xhd_scan_speed(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD scan speed: {}", self.common.key, value >> 1);
        self.common
            .set_value(&ControlId::ScanSpeed, (value >> 1) as f64);
        Ok(())
    }

    fn process_xhd_transmit_state(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD transmit state: {}", self.common.key, value);
        let power = if value == 1 {
            Power::Transmit
        } else {
            Power::Standby
        };
        self.common
            .set_value(&ControlId::Power, power as i32 as f64);
        Ok(())
    }

    fn process_xhd_range(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD range: {} m", self.common.key, value);
        self.range_meters = value;
        self.common.set_value(&ControlId::Range, value as f64);
        Ok(())
    }

    fn process_xhd_gain_mode(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD gain mode: {}", self.common.key, value);
        // 0=manual, 2=auto - just log for now, actual value set via gain_level
        Ok(())
    }

    fn process_xhd_gain_level(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD gain level: {}", self.common.key, value / 100);
        self.common
            .set_value(&ControlId::Gain, (value / 100) as f64);
        Ok(())
    }

    fn process_xhd_gain_auto_level(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD gain auto level: {}", self.common.key, value);
        // auto level 0=low, 1=high - just log for now
        Ok(())
    }

    fn process_xhd_bearing_alignment(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)? as i32;
        let degrees = value / 32;
        log::debug!(
            "{}: xHD bearing alignment: {} deg",
            self.common.key,
            degrees
        );
        self.common
            .set_value(&ControlId::BearingAlignment, degrees as f64);
        Ok(())
    }

    fn process_xhd_crosstalk(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD crosstalk: {}", self.common.key, value);
        self.common
            .set_value(&ControlId::InterferenceRejection, value as f64);
        Ok(())
    }

    fn process_xhd_rain_mode(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD rain mode: {}", self.common.key, value);
        let enabled = if value == 1 { 1u8 } else { 0u8 };
        self.common
            .set_value_enabled(&ControlId::Rain, 0.0, enabled);
        Ok(())
    }

    fn process_xhd_rain_level(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD rain level: {}", self.common.key, value / 100);
        self.common
            .set_value(&ControlId::Rain, (value / 100) as f64);
        Ok(())
    }

    fn process_xhd_sea_mode(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD sea mode: {}", self.common.key, value);
        let auto = if value == 2 { 1u8 } else { 0u8 };
        self.common.set_value_auto(&ControlId::Sea, 0.0, auto);
        Ok(())
    }

    fn process_xhd_sea_level(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD sea level: {}", self.common.key, value / 100);
        self.common.set_value(&ControlId::Sea, (value / 100) as f64);
        Ok(())
    }

    fn process_xhd_sea_auto_level(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD sea auto level: {}", self.common.key, value);
        // Just log for now
        Ok(())
    }

    fn process_xhd_no_tx_mode(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        let enabled = value == 1;
        log::debug!(
            "{}: xHD no-TX mode: {} (enabled={})",
            self.common.key,
            value,
            enabled
        );
        self.no_tx_enabled = Some(enabled);
        self.try_set_no_tx_sector();
        Ok(())
    }

    fn process_xhd_no_tx_start(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)? as i32;
        let degrees = value / 32;
        log::debug!("{}: xHD no-TX start: {} deg", self.common.key, degrees);
        self.no_tx_start = Some(degrees as f64);
        self.try_set_no_tx_sector();
        Ok(())
    }

    fn process_xhd_no_tx_end(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)? as i32;
        let degrees = value / 32;
        log::debug!("{}: xHD no-TX end: {} deg", self.common.key, degrees);
        self.no_tx_end = Some(degrees as f64);
        self.try_set_no_tx_sector();
        Ok(())
    }

    /// Try to set the no-transmit sector if all values have been received
    fn try_set_no_tx_sector(&mut self) {
        if let (Some(enabled), Some(start), Some(end)) =
            (self.no_tx_enabled, self.no_tx_start, self.no_tx_end)
        {
            log::debug!(
                "{}: Setting no-TX sector: enabled={} start={} end={}",
                self.common.key,
                enabled,
                start,
                end
            );
            self.common
                .set_sector(&ControlId::NoTransmitSector1, start, end, Some(enabled));
        }
    }

    fn process_xhd_timed_idle_mode(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD timed idle mode: {}", self.common.key, value);
        Ok(())
    }

    fn process_xhd_timed_idle_time(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!(
            "{}: xHD timed idle time: {} min",
            self.common.key,
            value / 60
        );
        Ok(())
    }

    fn process_xhd_timed_run_time(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!(
            "{}: xHD timed run time: {} min",
            self.common.key,
            value / 60
        );
        Ok(())
    }

    fn process_xhd_scanner_state(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        log::debug!("{}: xHD scanner state: {}", self.common.key, value);

        // xHD states: 2=WARMING_UP, 3=STANDBY, 4=SPINNING_UP, 5=TRANSMIT,
        // 6=STOPPING, 7=SPINNING_DOWN, 10=STARTING
        let power = match value {
            2 => Power::Preparing, // WARMING_UP
            3 => Power::Standby,
            4 | 10 => Power::Preparing, // SPINNING_UP
            5 => Power::Transmit,
            6 | 7 => Power::Preparing, // SPINNING_DOWN
            _ => Power::Off,
        };
        self.common
            .set_value(&ControlId::Power, power as i32 as f64);
        Ok(())
    }

    fn process_xhd_state_change(&mut self, data: &[u8]) -> Result<(), Error> {
        let value = self.extract_xhd_value(data)?;
        let seconds = value / 1000;
        log::debug!("{}: xHD state change in {} s", self.common.key, seconds);
        if seconds > 0 {
            self.common
                .set_value(&ControlId::WarmupTime, seconds as f64);
        }
        Ok(())
    }

    fn process_xhd_message(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.len() < 16 + 64 {
            return Ok(());
        }

        let info: [u8; 64] = data[16..16 + 64].try_into().unwrap();
        if let Some(msg) = c_string(&info) {
            log::debug!("{}: xHD message: \"{}\"", self.common.key, msg);
        }
        Ok(())
    }

    /// Extract value from xHD status packet based on length
    fn extract_xhd_value(&self, data: &[u8]) -> Result<u32, Error> {
        if data.len() < 9 {
            bail!("xHD packet too short");
        }

        let len = u32::from_le_bytes(data[4..8].try_into().unwrap());

        match len {
            1 => Ok(data[8] as u32),
            2 => Ok(u16::from_le_bytes(data[8..10].try_into().unwrap()) as u32),
            4 => Ok(u32::from_le_bytes(data[8..12].try_into().unwrap())),
            _ => Ok(0),
        }
    }
}

/// Unpack HD 1-bit packed spoke data to 8-bit values
fn unpack_hd_spoke(packed: &[u8], pixel_to_blob: &PixelToBlobType) -> GenericSpoke {
    let mut samples = Vec::with_capacity(packed.len() * 8);
    for byte in packed {
        for bit in 0..8 {
            let value = if (byte >> bit) & 1 == 1 { 255u8 } else { 0u8 };
            samples.push(pixel_to_blob[value as usize]);
        }
    }
    samples
}
