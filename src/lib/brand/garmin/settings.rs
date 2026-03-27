use std::collections::HashMap;
use std::f64::consts::PI;

use super::GarminRadarType;
use crate::{
    Cli,
    radar::settings::{
        ControlId, HAS_AUTO_NOT_ADJUSTABLE, SharedControls, new_auto, new_list, new_numeric,
        new_sector, new_string,
    },
    stream::SignalKDelta,
};

pub fn new(
    radar_id: String,
    sk_client_tx: tokio::sync::broadcast::Sender<SignalKDelta>,
    args: &Cli,
    radar_type: GarminRadarType,
) -> SharedControls {
    let mut controls = HashMap::new();

    new_string(ControlId::ModelName).build(&mut controls);
    new_string(ControlId::UserName).build(&mut controls);

    // Bearing alignment: internal radians, wire degrees
    // HD uses direct degrees, xHD uses degrees × 32
    // wire_scale_factor converts wire degrees to internal radians
    new_numeric(ControlId::BearingAlignment, -PI, PI)
        .wire_scale_factor(180. / PI, false)
        .build(&mut controls);

    // Gain: HD 0-100 or auto=344, xHD has separate mode/level
    new_auto(ControlId::Gain, 0., 100., HAS_AUTO_NOT_ADJUSTABLE).build(&mut controls);

    // Interference rejection
    new_numeric(ControlId::InterferenceRejection, 0., 100.).build(&mut controls);

    // Rain clutter
    new_numeric(ControlId::Rain, 0., 100.).build(&mut controls);

    // Sea clutter
    new_auto(ControlId::Sea, 0., 100., HAS_AUTO_NOT_ADJUSTABLE).build(&mut controls);

    // Scan speed
    new_numeric(ControlId::ScanSpeed, 0., 10.).build(&mut controls);

    // FTC mode (HD only)
    if radar_type == GarminRadarType::HD {
        new_list(ControlId::TargetExpansion, &["Off", "On"]).build(&mut controls);
    }

    // No-transmit sector (xHD only) - internal radians, wire degrees
    // wire_scale_factor converts wire degrees to internal radians
    if radar_type == GarminRadarType::XHD {
        new_sector(ControlId::NoTransmitSector1, -PI, PI)
            .wire_scale_factor(180. / PI, true)
            .build(&mut controls);
    }

    SharedControls::new(radar_id, sk_client_tx, args, controls)
}
