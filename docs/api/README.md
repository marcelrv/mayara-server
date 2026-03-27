# Mayara API

Mayara implements the [Signal K Radar API](https://github.com/SignalK/signalk-server/blob/master/docs/develop/rest-api/radar_api.md).

## Base URL

All radar API endpoints are under:

```
/signalk/v2/api/vessels/self/radars
```

## OpenAPI Specification

The full OpenAPI specification is available at runtime:

```
GET /signalk/v2/api/vessels/self/radars/resources/openapi.json
```

Or generate it with:

```bash
mayara-server --openapi
```

## Quick Reference

### REST Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/radars` | List all detected radars |
| GET | `/radars/interfaces` | List network interfaces and radar discovery status |
| GET | `/radars/{id}/capabilities` | Get radar capabilities |
| GET | `/radars/{id}/controls` | Get all control values |
| GET | `/radars/{id}/controls/{cid}` | Get specific control value |
| PUT | `/radars/{id}/controls/{cid}` | Set control value |
| POST | `/radars/{id}/targets` | Acquire target at position |
| DELETE | `/radars/{id}/targets/{tid}` | Delete tracked target |

### WebSocket Streams

| Endpoint | Description |
|----------|-------------|
| `/signalk/v1/stream` | Signal K delta stream (controls, targets) |
| `/radars/{id}/spokes` | Binary spoke data stream |

## See Also

- [Signal K Radar API Specification](https://github.com/SignalK/signalk-server/blob/master/docs/develop/rest-api/radar_api.md) - Full API specification
- [Power Control](controls/power.md) - Power/transmit control details
- [Range Control](controls/range.md) - Range control details
