[README.md](https://github.com/user-attachments/files/31539142/README.md)
# MMT Controller Network Configuration

Reads and sets the Ethernet configuration (IP address, subnet mask, gateway, TCP port) of an
MMT MMDC-series 4-axis stepper controller over RS-232, and provides a TCP client GUI for
manual motion control of a two-controller, eight-axis stage.

The RS-232 tool exists because the controller's Ethernet settings cannot be changed over
Ethernet. If the controller's IP is unknown or on a different subnet than the PC, the serial
port is the only way back in.

## Contents

| File | Purpose |
|------|---------|
| `MMT_Network_Config_GUI.py` | PyQt5 GUI. Reads and writes IP / SM / GW / PORT over RS-232, with a raw byte-level communication log. |
| `motor.py` | PyQt5 GUI. Connects to two controllers over TCP and drives eight axes (position, speed, stop, homing). |
| `Change_controller_IP.py` | Minimal script that writes a hardcoded IP and prints the reply. Superseded by the GUI; kept as a reference for the bare command sequence. |
| `Manuals/` | Vendor documentation for the MMDC controller. |

## Requirements

- Python 3.8 or newer
- `pyserial`
- `PyQt5`

```bash
pip install pyserial PyQt5
```

## Hardware setup (RS-232)

Connect the PC to the DB9 connector on the controller front panel.

| DB9 pin | Signal |
|---------|--------|
| 2 | TX |
| 3 | RX |
| 5 | GND |

Leave pins 1, 4, 6, 7, 8, and 9 unconnected. Set the controller's MODE switch to **RUN**, not
PRG. Link parameters are 115200 baud, 8 data bits, no parity, 1 stop bit.

## Usage

### Reading and changing the network configuration

```bash
python MMT_Network_Config_GUI.py
```

1. Select the COM port and press **Read from Controller**. Each field is queried
   independently, so an unsupported command does not block the others.
2. Press **Copy to New Configuration** to carry the current values across, or edit the fields
   directly.
3. Tick the checkbox next to each setting you want to write. Only ticked fields are sent.
4. Press **Apply Configuration** and confirm. Every value is validated before anything reaches
   the hardware: addresses go through `ipaddress.IPv4Address`, and the port must fall in
   1..65535.
5. Press **Reset Controller**, then **Read from Controller** again.

Step 5 is not optional. A `*ok` reply means the controller accepted the command, not that the
value survived a power cycle. The read-back after reset is the only confirmation that the
setting reached non-volatile memory.

After a successful write, any open Ethernet session to the controller drops, and the PC must
be on the new subnet to reach it again.

### Motion control

```bash
python motor.py
```

Connects to two controllers over TCP (default `192.168.0.128` and `192.168.0.129`, port 5001).
Axis map:

| Controller | Port 1 | Port 2 | Port 3 | Port 4 |
|-----------|--------|--------|--------|--------|
| 1 | X (mm) | Y (mm) | Z (mm) | CAM (mm) |
| 2 | Tx (deg) | Ty (deg) | Tz (deg) | Lens_Loader (mm) |

Linear axes use 51200 steps/mm. Tilt axes convert degrees to steps through a 50 mm rotation
radius, `step = R * steps_per_mm * sin(theta)`, referenced to the step count captured at
connect time. Tilt travel is clamped to ±5.4°.

Connecting powers on every used port (`ST0`); disconnecting powers them off (`ST1`). The
Lens_Loader homing button runs `4ST0` → `4MA1` → `4HM1` and then polls `@R` until port 4
reports ready, with a 30 s timeout.

## Protocol notes

The manual specifies a 2 ms frame-end interval and defines a reply as mask + echo + data with
no terminating character. A reply is therefore delimited by the line going idle rather than by
a byte value.

`MMT_Network_Config_GUI.py` uses a 50 ms idle gap instead of the specified 2 ms. A USB-serial
bridge batches incoming bytes on its own latency timer, commonly 16 ms, so a 2 ms gap would
split a single reply into fragments. CR or LF closes a frame early if a firmware revision
happens to send one, and its absence is not treated as an error.

Reply shapes observed:

```
*IP=192.168.0.123     ethernet getters
*PORT5001             port getter
*ok / *okAtten        setter acknowledgement
```

`*okAtten` is returned when motor power is off. It is an acceptance, not a failure.

Setter payloads use comma-separated octets: `ip192,168,0,123`.

## Firmware differences

`sm` and `gw` exist only in the newer command set. On older firmware they return no reply, the
subnet mask and gateway are fixed, and the GUI reports those two fields as NO REPLY rather than
silently substituting a value.

## Design constraints

The network configuration tool has no retry logic and no fallback parsing. A command that goes
unanswered is reported as unanswered. A reply that does not echo the command it was sent for
raises rather than being coerced into a plausible-looking value. The communication log records
every transmitted and received byte sequence verbatim, so a framing problem stays visible in
the transcript instead of being absorbed.

Factory defaults (`192.168.0.123`, `255.255.255.0`, `192.168.0.1`, port 5001) populate the
input fields on startup as a convenience. They are never written without an explicit user
action.

## Known limitations

- `Change_controller_IP.py` hardcodes `COM7` and performs no validation. Use the GUI.
- `motor.py` swallows several exception paths, so a dropped TCP connection can surface as a
  stale reading rather than an error.
- Homing is implemented for port 4 on each controller only.
- Tilt zero is captured at connect time from whatever position the stage happens to be in. It
  is not an absolute reference.

## Default values

| Setting | Factory default |
|---------|-----------------|
| IP address | 192.168.0.123 |
| Subnet mask | 255.255.255.0 |
| Gateway | 192.168.0.1 |
| TCP port | 5001 |
