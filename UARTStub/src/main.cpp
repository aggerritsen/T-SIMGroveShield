//main.cpp

#include <Arduino.h>
#include "esp_crc.h"

/* =========================================================
   SIMPLE UART STUB (XIAO ESP32-S3 | UART1 Broker)
   - Uses second UART (HardwareSerial(1)) on pins 43/44
   - Periodically sends a sample message frame with CRC32
   - Receives frames, validates CRC, replies ACK/NACK
   - Timestamped logging on Serial (ms since boot)
   ========================================================= */

/* ================================
   UART CONFIG (XIAO → T-SIM)
   ================================ */
static constexpr uint32_t UART_BAUD   = 921600;
static constexpr int      UART_TX_PIN = 43;
static constexpr int      UART_RX_PIN = 44;

static constexpr bool     ENABLE_UART_TRANSPORT = true;

/* ================================
   PROTOCOL / FLOW
   ================================ */
static constexpr uint32_t TX_INTERVAL_MS  = 2000;  // send a sample frame every 2s (when not awaiting ACK)
static constexpr uint32_t ACK_TIMEOUT_MS  = 5000;  // resend after 5s if no ACK
static constexpr uint32_t MAX_PAYLOAD_LEN = 2048;  // safeguard for payload size

/* ================================
   STATE
   ================================ */
HardwareSerial BrokerUART(1);

static uint32_t frame_id      = 0;
static bool     awaiting_ack  = false;
static uint32_t last_send_ms  = 0;
static uint32_t next_tx_ms    = 0;

/* Cached frame for resend */
static String   cached_payload;
static uint32_t cached_crc    = 0;

/* RX parser state */
static bool     rx_in_frame   = false;
static uint32_t rx_id         = 0;
static size_t   rx_len        = 0;
static uint32_t rx_crc        = 0;
static String   rx_payload;

/* ================================
   LOGGING (timestamps)
   ================================ */
static inline void log_ts(const char* tag, const char* fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.printf("[%10lu] %s %s\n", (unsigned long)millis(), tag, buf);
}

/* ================================
   CRC32 helper (LE, same as your file)
   ================================ */
static inline uint32_t crc32_of_string(const String& s)
{
  return esp_crc32_le(0, (const uint8_t*)s.c_str(), s.length());
}

/* ================================
   TX: prepare a sample message
   ================================ */
static void prepare_sample_frame()
{
  frame_id++;

  // Sample payload — keep it simple but non-trivial
  // Feel free to change this string; CRC will adapt.
  cached_payload = "";
  cached_payload += "{\"type\":\"sample\",\"frame\":";
  cached_payload += frame_id;
  cached_payload += ",\"ts_ms\":";
  cached_payload += (uint32_t)millis();
  cached_payload += ",\"msg\":\"Hello from XIAO UART1\"}";

  cached_crc = crc32_of_string(cached_payload);

  log_ts("TX", "Prepared frame id=%lu len=%u crc=%08lx",
         (unsigned long)frame_id,
         (unsigned)cached_payload.length(),
         (unsigned long)cached_crc);
}

/* ================================
   TX: send cached frame (and start ACK wait)
   Frame format:
     FRAME <id> <len> <crc>\n
     <payload bytes exactly len>\n
     END\n
   ================================ */
static void send_cached_frame()
{
  if (!ENABLE_UART_TRANSPORT) return;

  const size_t len = cached_payload.length();

  BrokerUART.print("FRAME ");
  BrokerUART.print(frame_id);
  BrokerUART.print(" ");
  BrokerUART.print(len);
  BrokerUART.print(" ");
  BrokerUART.printf("%08lx\n", (unsigned long)cached_crc);

  // Payload line (ASCII JSON in this stub); receiver uses <len> to validate exact length.
  // IMPORTANT: We still terminate with '\n' for readStringUntil convenience on the other side.
  BrokerUART.print(cached_payload);
  BrokerUART.print("\n");

  BrokerUART.println("END");

  last_send_ms = millis();
  awaiting_ack = true;

  log_ts("TX", "Sent frame id=%lu (%u bytes)", (unsigned long)frame_id, (unsigned)len);
}

/* ================================
   TX: send ACK/NACK
   ================================ */
static void send_ack(uint32_t id)
{
  if (!ENABLE_UART_TRANSPORT) return;
  BrokerUART.print("ACK ");
  BrokerUART.println(id);
  log_ts("TX", "ACK %lu", (unsigned long)id);
}

static void send_nack(uint32_t id)
{
  if (!ENABLE_UART_TRANSPORT) return;
  BrokerUART.print("NACK ");
  BrokerUART.println(id);
  log_ts("TX", "NACK %lu", (unsigned long)id);
}

/* ================================
   RX: attempt to parse a "FRAME ..." header
   ================================ */
static bool parse_frame_header(const String& line, uint32_t& out_id, size_t& out_len, uint32_t& out_crc)
{
  // Expect: "FRAME <id> <len> <crc_hex>"
  if (!line.startsWith("FRAME ")) return false;

  // Simple token split
  // FRAME [1]id [2]len [3]crc
  int p1 = line.indexOf(' ');
  if (p1 < 0) return false;
  int p2 = line.indexOf(' ', p1 + 1);
  if (p2 < 0) return false;
  int p3 = line.indexOf(' ', p2 + 1);
  if (p3 < 0) return false;

  String sid  = line.substring(p1 + 1, p2);
  String slen = line.substring(p2 + 1, p3);
  String scrc = line.substring(p3 + 1);

  sid.trim(); slen.trim(); scrc.trim();

  if (sid.length() == 0 || slen.length() == 0 || scrc.length() == 0) return false;

  out_id  = (uint32_t)sid.toInt();
  out_len = (size_t)slen.toInt();

  // crc hex
  char* endptr = nullptr;
  out_crc = (uint32_t)strtoul(scrc.c_str(), &endptr, 16);
  if (endptr == scrc.c_str()) return false;

  return true;
}

/* ================================
   RX: process one complete received frame
   ================================ */
static void handle_complete_rx_frame()
{
  // Basic length checks
  if (rx_len > MAX_PAYLOAD_LEN)
  {
    log_ts("RX", "Frame id=%lu rejected: len=%u exceeds MAX_PAYLOAD_LEN=%u",
           (unsigned long)rx_id, (unsigned)rx_len, (unsigned)MAX_PAYLOAD_LEN);
    send_nack(rx_id);
    return;
  }

  if (rx_payload.length() != rx_len)
  {
    log_ts("RX", "Frame id=%lu length mismatch: header len=%u actual=%u",
           (unsigned long)rx_id, (unsigned)rx_len, (unsigned)rx_payload.length());
    send_nack(rx_id);
    return;
  }

  uint32_t calc = crc32_of_string(rx_payload);
  if (calc != rx_crc)
  {
    log_ts("RX", "Frame id=%lu CRC FAIL: rx=%08lx calc=%08lx",
           (unsigned long)rx_id, (unsigned long)rx_crc, (unsigned long)calc);
    send_nack(rx_id);
    return;
  }

  log_ts("RX", "Frame id=%lu OK: len=%u crc=%08lx payload=%s",
         (unsigned long)rx_id,
         (unsigned)rx_len,
         (unsigned long)rx_crc,
         rx_payload.c_str());

  send_ack(rx_id);
}

/* ================================
   RX: line-based parser
   Handles:
     ACK <id>
     NACK <id>
     FRAME ... / payload / END
   ================================ */
static void poll_uart_rx()
{
  if (!ENABLE_UART_TRANSPORT) return;

  while (BrokerUART.available())
  {
    String line = BrokerUART.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    // ACK/NACK handling (for our outbound frames)
    if (line.startsWith("ACK "))
    {
      uint32_t ack_id = (uint32_t)line.substring(4).toInt();
      log_ts("RX", "ACK %lu", (unsigned long)ack_id);

      if (awaiting_ack && ack_id == frame_id)
      {
        awaiting_ack = false;
        log_ts("FLOW", "Outbound frame %lu acknowledged", (unsigned long)ack_id);
      }
      continue;
    }

    if (line.startsWith("NACK "))
    {
      uint32_t nack_id = (uint32_t)line.substring(5).toInt();
      log_ts("RX", "NACK %lu", (unsigned long)nack_id);

      if (awaiting_ack && nack_id == frame_id)
      {
        log_ts("FLOW", "Outbound frame %lu NACKed → resend", (unsigned long)nack_id);
        send_cached_frame();
      }
      continue;
    }

    // Frame parsing
    if (!rx_in_frame)
    {
      uint32_t id = 0;
      size_t len  = 0;
      uint32_t crc = 0;

      if (parse_frame_header(line, id, len, crc))
      {
        rx_in_frame = true;
        rx_id = id;
        rx_len = len;
        rx_crc = crc;
        rx_payload = "";

        log_ts("RX", "Header FRAME id=%lu len=%u crc=%08lx",
               (unsigned long)rx_id, (unsigned)rx_len, (unsigned long)rx_crc);
      }
      else
      {
        // Unknown line; log it (useful during bring-up)
        log_ts("RX", "Unrecognized line: %s", line.c_str());
      }
      continue;
    }
    else
    {
      // We are inside a frame: expect payload line(s) and END
      if (line == "END")
      {
        log_ts("RX", "END for frame id=%lu", (unsigned long)rx_id);
        handle_complete_rx_frame();

        // Reset RX state
        rx_in_frame = false;
        rx_id = 0;
        rx_len = 0;
        rx_crc = 0;
        rx_payload = "";
      }
      else
      {
        // In this stub the payload is a single line.
        // If you later want multi-line payloads, you can append with "\n" and count bytes carefully.
        rx_payload = line;

        log_ts("RX", "Payload captured for frame id=%lu (len=%u)",
               (unsigned long)rx_id, (unsigned)rx_payload.length());
      }
    }
  }
}

/* ================================
   SETUP
   ================================ */
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("=======================================");
  Serial.println(" XIAO ESP32-S3 | SIMPLE UART STUB (UART1)");
  Serial.println("=======================================");
  log_ts("BOOT", "Build: %s %s", __DATE__, __TIME__);
  log_ts("BOOT", "UART1 TX=%d RX=%d baud=%lu", UART_TX_PIN, UART_RX_PIN, (unsigned long)UART_BAUD);

  if (ENABLE_UART_TRANSPORT)
  {
    BrokerUART.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    log_ts("BOOT", "UART transport enabled");
  }
  else
  {
    log_ts("BOOT", "UART transport DISABLED (ENABLE_UART_TRANSPORT=false)");
  }

  next_tx_ms = millis() + 1000; // give peer a second to boot
}

/* ================================
   LOOP
   ================================ */
void loop()
{
  // 1) Poll RX continuously
  poll_uart_rx();

  // 2) Resend on ACK timeout
  if (ENABLE_UART_TRANSPORT && awaiting_ack)
  {
    if (millis() - last_send_ms > ACK_TIMEOUT_MS)
    {
      log_ts("FLOW", "ACK timeout for frame %lu → resend", (unsigned long)frame_id);
      send_cached_frame();
      return;
    }
  }

  // 3) Periodic TX when not awaiting ACK
  if (ENABLE_UART_TRANSPORT && !awaiting_ack)
  {
    const uint32_t now = millis();
    if ((int32_t)(now - next_tx_ms) >= 0)
    {
      prepare_sample_frame();
      send_cached_frame();
      next_tx_ms = now + TX_INTERVAL_MS;
    }
  }
}
