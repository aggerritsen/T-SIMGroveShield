# UART Stub Transport – XIAO ESP32-S3

This repository contains a **minimal, deterministic UART transport stub** intended for bring-up, wiring validation, and protocol testing between a **XIAO ESP32-S3** and a peer device (e.g. T-SIM board).

The implementation focuses on **reliability, observability, and simplicity**. It deliberately avoids application logic (AI, Grove, sensors) and instead provides a clean UART framing, CRC validation, ACK/NACK flow, and timestamped logging.

---

## Purpose

This stub exists to answer one question unambiguously:

> *Does my UART link work correctly under load, with retries and integrity checks?*

It is designed as a **transport sanity harness** that you can trust before layering higher-level protocols or large binary payloads (images, inference data, etc.) on top.

---

## Key Features

* Uses **second hardware UART** (`HardwareSerial(1)`) on ESP32-S3
* High-speed UART: **921600 baud**
* Explicit TX/RX pin mapping
* Application-level **frame protocol**
* **CRC32 (little-endian)** payload integrity check
* **ACK / NACK** flow control
* Automatic resend on timeout or NACK
* Timestamped USB serial logging (`millis()`)
* No external library dependencies

---

## UART Configuration

| Parameter     | Value                       |
| ------------- | --------------------------- |
| UART instance | UART1 (`HardwareSerial(1)`) |
| Baud rate     | 921600                      |
| Data format   | 8N1                         |
| TX pin        | GPIO 43                     |
| RX pin        | GPIO 44                     |

> ⚠️ TX/RX **must be crossed** and **GND must be shared** with the peer device.

---

## Frame Protocol

All communication uses a simple, line-based application frame.

### Wire Format

```
FRAME <id> <len> <crc>
<payload>
END
```

### Fields

| Field     | Description                                |
| --------- | ------------------------------------------ |
| `id`      | Monotonically increasing frame ID (uint32) |
| `len`     | Exact payload length in bytes              |
| `crc`     | CRC32 (hex, little-endian) of payload only |
| `payload` | Message body (JSON in this stub)           |

The header and `END` marker are **not** included in the CRC calculation.

---

## Payload

In this development stub, the payload is a **small JSON message**:

```json
{
  "type": "sample",
  "frame": 1,
  "ts_ms": 12345,
  "msg": "Hello from XIAO UART1"
}
```

This is intentionally human-readable to simplify debugging. The protocol is payload-agnostic and can later be replaced with binary data without changing the transport logic.

---

## CRC Handling

* CRC algorithm: **CRC32 little-endian**
* Implementation: `esp_crc32_le()` from ESP32 ROM
* CRC is calculated over the **exact payload bytes**

### Validation rules

A received frame is rejected if:

* Payload length does not match header `len`
* Payload length exceeds safety limit
* Calculated CRC does not match header CRC

Rejected frames trigger a `NACK <id>`.

---

## ACK / NACK Flow

### Normal flow

1. Sender transmits a frame
2. Receiver validates length and CRC
3. Receiver replies:

```
ACK <id>
```

4. Sender clears its retry state and proceeds with next frame

### Error flow

If validation fails:

```
NACK <id>
```

The sender **resends the same cached frame** (ID does not advance).

### Timeout flow

If no ACK/NACK is received within **5 seconds**:

* Sender automatically resends the cached frame
* Retries continue until ACK is received

This guarantees idempotent, loss-tolerant delivery.

---

## Logging

All events are logged to USB Serial (`Serial`) with timestamps:

```
[      1692] TX Prepared frame id=1 len=70 crc=6f60a379
[      1692] TX Sent frame id=1 (70 bytes)
[      6693] FLOW ACK timeout for frame 1 → resend
```

This makes timing, retries, and protocol state immediately visible during development.

---

## Dependencies

**None beyond the ESP32 Arduino core.**

Used headers:

* `<Arduino.h>`
* `"esp_crc.h"`

No Grove, AI, Wire, RTOS, or external libraries are required.

---

## Intended Use

This stub is ideal for:

* UART wiring validation
* High-baud stability testing
* RX/TX pin verification
* CRC and framing validation
* Developing the peer receiver firmware
* Debugging transport issues in isolation

It is **not** intended as a final production protocol.

---

## Next Steps

Once this stub is proven stable, you can safely:

* Replace JSON payloads with binary data
* Add multi-chunk payload support
* Integrate camera or AI outputs
* Add protocol versioning
* Move from line-based to length-based RX

The transport layer can remain unchanged.

---

## Status

✅ Transport verified

This code represents a known-good UART transport baseline.
