#include <Arduino.h>
#include <Wire.h>
#include "esp_crc.h"

/* =========================================================
   UART PEER STUB (T-SIM7080G-S3 | UART1)
   - Uses HardwareSerial(1)
   - Receives FRAME/LEN/CRC + payload + END
   - Validates length + CRC32 (esp_crc32_le)
   - Responds ACK <id> or NACK <id>
   - Optional periodic TX sample frames (duplex testing)
   - Timestamped USB serial logging (millis)
   ========================================================= */

/* ================================
   BROKER UART CONFIG (defaults; overridable via build_flags *_CFG)
   Use non-generic names to avoid collisions with Arduino core macros.
   ================================ */
static constexpr uint32_t BROKER_UART_BAUD_DEFAULT     = 921600;
static constexpr int      BROKER_UART_TX_GPIO_DEFAULT  = 18; // T-SIM7080G-S3 TX (P1.5) GPIO18
static constexpr int      BROKER_UART_RX_GPIO_DEFAULT  = 8;  // T-SIM7080G-S3 RX (P1.6) GPIO08

static constexpr bool     ENABLE_BROKER_UART_DEFAULT        = true;
static constexpr bool     ENABLE_BROKER_PERIODIC_TX_DEFAULT = false;

/* Resolve final config (compile-time) */
#ifdef BROKER_UART_BAUD_CFG
static constexpr uint32_t BROKER_UART_BAUD = (uint32_t)BROKER_UART_BAUD_CFG;
#else
static constexpr uint32_t BROKER_UART_BAUD = BROKER_UART_BAUD_DEFAULT;
#endif

#ifdef BROKER_UART_TX_GPIO_CFG
static constexpr int BROKER_UART_TX_GPIO = (int)BROKER_UART_TX_GPIO_CFG;
#else
static constexpr int BROKER_UART_TX_GPIO = BROKER_UART_TX_GPIO_DEFAULT;
#endif

#ifdef BROKER_UART_RX_GPIO_CFG
static constexpr int BROKER_UART_RX_GPIO = (int)BROKER_UART_RX_GPIO_CFG;
#else
static constexpr int BROKER_UART_RX_GPIO = BROKER_UART_RX_GPIO_DEFAULT;
#endif

#ifdef ENABLE_BROKER_UART_CFG
static constexpr bool ENABLE_BROKER_UART = (ENABLE_BROKER_UART_CFG != 0);
#else
static constexpr bool ENABLE_BROKER_UART = ENABLE_BROKER_UART_DEFAULT;
#endif

#ifdef ENABLE_BROKER_PERIODIC_TX_CFG
static constexpr bool ENABLE_BROKER_PERIODIC_TX = (ENABLE_BROKER_PERIODIC_TX_CFG != 0);
#else
static constexpr bool ENABLE_BROKER_PERIODIC_TX = ENABLE_BROKER_PERIODIC_TX_DEFAULT;
#endif

/* ================================
   GPIO OUTPUT TEST (D0..D7)
   - Non-blocking "walking 1" pattern: one pin HIGH at a time, sequential, perpetual
   - Purely additive: does not alter UART/protocol logic
   - Pins are set via build_flags: D0_GPIO_CFG..D7_GPIO_CFG
   ================================ */
static constexpr bool     ENABLE_GPIO_TEST_DEFAULT        = true;
static constexpr uint32_t GPIO_TEST_INTERVAL_MS_DEFAULT   = 200;

#ifdef ENABLE_GPIO_TEST_CFG
static constexpr bool ENABLE_GPIO_TEST = (ENABLE_GPIO_TEST_CFG != 0);
#else
static constexpr bool ENABLE_GPIO_TEST = ENABLE_GPIO_TEST_DEFAULT;
#endif

#ifdef GPIO_TEST_INTERVAL_MS_CFG
static constexpr uint32_t GPIO_TEST_INTERVAL_MS = (uint32_t)GPIO_TEST_INTERVAL_MS_CFG;
#else
static constexpr uint32_t GPIO_TEST_INTERVAL_MS = GPIO_TEST_INTERVAL_MS_DEFAULT;
#endif

// Defaults are placeholders; final values should come from build_flags per env.
#ifdef D0_GPIO_CFG
static constexpr int D0_GPIO = (int)D0_GPIO_CFG;
#else
static constexpr int D0_GPIO = -1;
#endif

#ifdef D1_GPIO_CFG
static constexpr int D1_GPIO = (int)D1_GPIO_CFG;
#else
static constexpr int D1_GPIO = -1;
#endif

#ifdef D2_GPIO_CFG
static constexpr int D2_GPIO = (int)D2_GPIO_CFG;
#else
static constexpr int D2_GPIO = -1;
#endif

#ifdef D3_GPIO_CFG
static constexpr int D3_GPIO = (int)D3_GPIO_CFG;
#else
static constexpr int D3_GPIO = -1;
#endif

#ifdef D4_GPIO_CFG
static constexpr int D4_GPIO = (int)D4_GPIO_CFG;
#else
static constexpr int D4_GPIO = -1;
#endif

#ifdef D5_GPIO_CFG
static constexpr int D5_GPIO = (int)D5_GPIO_CFG;
#else
static constexpr int D5_GPIO = -1;
#endif

#ifdef D6_GPIO_CFG
static constexpr int D6_GPIO = (int)D6_GPIO_CFG;
#else
static constexpr int D6_GPIO = -1;
#endif

#ifdef D7_GPIO_CFG
static constexpr int D7_GPIO = (int)D7_GPIO_CFG;
#else
static constexpr int D7_GPIO = -1;
#endif

static const int GPIO_TEST_PINS[8] = {
  D0_GPIO, D1_GPIO, D2_GPIO, D3_GPIO, D4_GPIO, D5_GPIO, D6_GPIO, D7_GPIO
};

static uint32_t gpio_next_step_ms = 0;
static uint8_t  gpio_step_index   = 0;

static inline void gpio_write_safe(int pin, uint8_t level)
{
  if (pin < 0) return;
  digitalWrite(pin, level);
}

/* ================================
   I2C-1 SCAN (AFTER EACH GPIO CYCLE) - WATCHDOG SAFE
   - Must scan bus 1 (not bus 0)
   - Uses TwoWire(1)
   - Bus initialized once
   - Scan runs incrementally: 1 address per loop tick (no blocking)
   ================================ */
static constexpr bool     ENABLE_I2C_SCAN_DEFAULT = true;

#ifdef ENABLE_I2C_SCAN_CFG
static constexpr bool ENABLE_I2C_SCAN = (ENABLE_I2C_SCAN_CFG != 0);
#else
static constexpr bool ENABLE_I2C_SCAN = ENABLE_I2C_SCAN_DEFAULT;
#endif

#ifdef I2C1_SDA_GPIO_CFG
static constexpr int I2C1_SDA_GPIO = (int)I2C1_SDA_GPIO_CFG;
#else
static constexpr int I2C1_SDA_GPIO = -1;
#endif

#ifdef I2C1_SCL_GPIO_CFG
static constexpr int I2C1_SCL_GPIO = (int)I2C1_SCL_GPIO_CFG;
#else
static constexpr int I2C1_SCL_GPIO = -1;
#endif

#ifdef I2C1_FREQ_CFG
static constexpr uint32_t I2C1_FREQ = (uint32_t)I2C1_FREQ_CFG;
#else
static constexpr uint32_t I2C1_FREQ = 400000;
#endif

TwoWire I2Cbus1 = TwoWire(1);
static bool i2c1_initialized = false;

// incremental scan state
static bool     i2c_scan_active = false;
static uint8_t  i2c_scan_addr   = 0x03;
static uint8_t  i2c_scan_found  = 0;
static uint32_t i2c_scan_start_ms = 0;

static void i2c1_init_once()
{
  if (!ENABLE_I2C_SCAN) return;
  if (i2c1_initialized) return;

  if (I2C1_SDA_GPIO < 0 || I2C1_SCL_GPIO < 0)
  {
    Serial.println("[I2C] I2C-1 pins not configured (I2C1_SDA_GPIO_CFG / I2C1_SCL_GPIO_CFG)");
    return;
  }

  Serial.println("=======================================");
  Serial.println(" I2C-1 INIT (external bus)");
  Serial.println("=======================================");
  Serial.printf(" SDA=%d SCL=%d freq=%lu Hz\n",
                I2C1_SDA_GPIO, I2C1_SCL_GPIO, (unsigned long)I2C1_FREQ);

  // Initialize I2C bus 1 with explicit pins
  I2Cbus1.begin(I2C1_SDA_GPIO, I2C1_SCL_GPIO, I2C1_FREQ);

  i2c1_initialized = true;
}

static void i2c1_scan_start()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;

  // start a new scan
  i2c_scan_active = true;
  i2c_scan_addr = 0x03;
  i2c_scan_found = 0;
  i2c_scan_start_ms = millis();

  Serial.println("=======================================");
  Serial.println(" I2C-1 SCAN START (after GPIO cycle)");
  Serial.println("=======================================");
}

static void i2c1_scan_tick()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;
  if (!i2c_scan_active) return;

  // Scan exactly ONE address per loop iteration (watchdog safe)
  const uint8_t addr = i2c_scan_addr;

  I2Cbus1.beginTransmission(addr);
  if (I2Cbus1.endTransmission() == 0)
  {
    Serial.printf("  ✓ I2C device found at 0x%02X\n", addr);
    i2c_scan_found++;
  }

  // advance
  if (i2c_scan_addr >= 0x77)
  {
    const uint32_t dur = millis() - i2c_scan_start_ms;

    if (i2c_scan_found == 0)
      Serial.println("  (no I2C devices found)");
    else
      Serial.printf("  Total devices: %u\n", i2c_scan_found);

    Serial.printf("  Scan duration: %lu ms\n", (unsigned long)dur);

    i2c_scan_active = false;
    return;
  }

  i2c_scan_addr++;

  // Give FreeRTOS/USB time; avoids WDT even on busy systems.
  delay(0);
}

/* ================================
   GPIO test init/tick
   ================================ */
static void gpio_test_init()
{
  if (!ENABLE_GPIO_TEST) return;

  // Configure pins as outputs, set LOW
  for (int i = 0; i < 8; i++)
  {
    const int pin = GPIO_TEST_PINS[i];
    if (pin >= 0)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }

  gpio_step_index = 0;
  gpio_next_step_ms = millis() + GPIO_TEST_INTERVAL_MS;
}

static void gpio_test_tick()
{
  if (!ENABLE_GPIO_TEST) return;

  const uint32_t now = millis();
  if ((int32_t)(now - gpio_next_step_ms) < 0) return;

  // Walking 1: all LOW except the active index HIGH
  for (int i = 0; i < 8; i++)
  {
    gpio_write_safe(GPIO_TEST_PINS[i], (i == gpio_step_index) ? HIGH : LOW);
  }

  // Advance and detect wrap (end of cycle)
  gpio_step_index = (uint8_t)((gpio_step_index + 1) & 0x07);

  // After each full D0..D7 cycle (wrap back to 0), start I2C-1 scan
  if (gpio_step_index == 0)
  {
    i2c1_scan_start();
  }

  gpio_next_step_ms = now + GPIO_TEST_INTERVAL_MS;
}

/* ================================
   PROTOCOL / FLOW
   ================================ */
static constexpr uint32_t ACK_TIMEOUT_MS  = 5000;
static constexpr uint32_t TX_INTERVAL_MS  = 2000;
static constexpr uint32_t MAX_PAYLOAD_LEN = 4096;

/* ================================
   UART
   ================================ */
HardwareSerial BrokerUART(1);

/* ================================
   STATE (outbound)
   ================================ */
static uint32_t tx_frame_id     = 0;
static bool     awaiting_ack    = false;
static uint32_t last_send_ms    = 0;
static uint32_t next_tx_ms      = 0;
static String   cached_payload;
static uint32_t cached_crc      = 0;

/* ================================
   STATE (inbound parser)
   ================================ */
static bool     rx_in_frame     = false;
static uint32_t rx_id           = 0;
static size_t   rx_len          = 0;
static uint32_t rx_crc          = 0;
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
   CRC32 helper (LE)
   ================================ */
static inline uint32_t crc32_of_string(const String& s)
{
  return esp_crc32_le(0, (const uint8_t*)s.c_str(), s.length());
}

/* ================================
   TX: ACK / NACK
   ================================ */
static void send_ack(uint32_t id)
{
  if (!ENABLE_BROKER_UART) return;
  BrokerUART.print("ACK ");
  BrokerUART.println(id);
  log_ts("TX", "ACK %lu", (unsigned long)id);
}

static void send_nack(uint32_t id)
{
  if (!ENABLE_BROKER_UART) return;
  BrokerUART.print("NACK ");
  BrokerUART.println(id);
  log_ts("TX", "NACK %lu", (unsigned long)id);
}

/* ================================
   TX: sample outbound frame (optional duplex)
   ================================ */
static void prepare_sample_frame()
{
  tx_frame_id++;

  cached_payload = "";
  cached_payload += "{\"type\":\"peer\",\"frame\":";
  cached_payload += tx_frame_id;
  cached_payload += ",\"ts_ms\":";
  cached_payload += (uint32_t)millis();
  cached_payload += ",\"msg\":\"Hello from T-SIM UART1\"}";

  cached_crc = crc32_of_string(cached_payload);

  log_ts("TX", "Prepared frame id=%lu len=%u crc=%08lx",
         (unsigned long)tx_frame_id,
         (unsigned)cached_payload.length(),
         (unsigned long)cached_crc);
}

static void send_cached_frame()
{
  if (!ENABLE_BROKER_UART) return;

  const size_t len = cached_payload.length();

  BrokerUART.print("FRAME ");
  BrokerUART.print(tx_frame_id);
  BrokerUART.print(" ");
  BrokerUART.print(len);
  BrokerUART.print(" ");
  BrokerUART.printf("%08lx\n", (unsigned long)cached_crc);

  BrokerUART.print(cached_payload);
  BrokerUART.print("\n");

  BrokerUART.println("END");

  last_send_ms = millis();
  awaiting_ack = true;

  log_ts("TX", "Sent frame id=%lu (%u bytes)", (unsigned long)tx_frame_id, (unsigned)len);
}

/* ================================
   RX: parse header
   ================================ */
static bool parse_frame_header(const String& line, uint32_t& out_id, size_t& out_len, uint32_t& out_crc)
{
  if (!line.startsWith("FRAME ")) return false;

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
  if (sid.isEmpty() || slen.isEmpty() || scrc.isEmpty()) return false;

  out_id  = (uint32_t)sid.toInt();
  out_len = (size_t)slen.toInt();

  char* endptr = nullptr;
  out_crc = (uint32_t)strtoul(scrc.c_str(), &endptr, 16);
  if (endptr == scrc.c_str()) return false;

  return true;
}

/* ================================
   RX: validate + respond
   ================================ */
static void handle_complete_rx_frame()
{
  if (rx_len > MAX_PAYLOAD_LEN)
  {
    log_ts("RX", "Frame id=%lu rejected: len=%u > MAX=%u",
           (unsigned long)rx_id, (unsigned)rx_len, (unsigned)MAX_PAYLOAD_LEN);
    send_nack(rx_id);
    return;
  }

  if (rx_payload.length() != rx_len)
  {
    log_ts("RX", "Frame id=%lu length mismatch: header=%u actual=%u",
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
   RX: poll UART and process lines
   ================================ */
static void poll_uart_rx()
{
  if (!ENABLE_BROKER_UART) return;

  while (BrokerUART.available())
  {
    String line = BrokerUART.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    // ACK/NACK for our outbound frames (if periodic TX enabled)
    if (line.startsWith("ACK "))
    {
      uint32_t ack_id = (uint32_t)line.substring(4).toInt();
      log_ts("RX", "ACK %lu", (unsigned long)ack_id);

      if (awaiting_ack && ack_id == tx_frame_id)
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

      if (awaiting_ack && nack_id == tx_frame_id)
      {
        log_ts("FLOW", "Outbound frame %lu NACKed → resend", (unsigned long)nack_id);
        send_cached_frame();
      }
      continue;
    }

    // Inbound frame parsing
    if (!rx_in_frame)
    {
      uint32_t id = 0;
      size_t len = 0;
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
        log_ts("RX", "Unrecognized line: %s", line.c_str());
      }
      continue;
    }
    else
    {
      if (line == "END")
      {
        log_ts("RX", "END for frame id=%lu", (unsigned long)rx_id);
        handle_complete_rx_frame();

        rx_in_frame = false;
        rx_id = 0;
        rx_len = 0;
        rx_crc = 0;
        rx_payload = "";
      }
      else
      {
        // single-line payload in this stub
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
  Serial.println(" T-SIM | UART PEER STUB (UART1) ");
  Serial.println("=======================================");
  log_ts("BOOT", "Build: %s %s", __DATE__, __TIME__);
  log_ts("BOOT", "UART1 TX=%d RX=%d baud=%lu",
         BROKER_UART_TX_GPIO, BROKER_UART_RX_GPIO, (unsigned long)BROKER_UART_BAUD);

  if (ENABLE_BROKER_UART)
  {
    BrokerUART.begin(BROKER_UART_BAUD, SERIAL_8N1, BROKER_UART_RX_GPIO, BROKER_UART_TX_GPIO);
    log_ts("BOOT", "UART transport enabled");
  }
  else
  {
    log_ts("BOOT", "UART transport DISABLED");
  }

  // GPIO test init (additive)
  if (ENABLE_GPIO_TEST)
  {
    log_ts("GPIO", "GPIO test enabled interval=%lu ms", (unsigned long)GPIO_TEST_INTERVAL_MS);
    log_ts("GPIO", "D0=%d D1=%d D2=%d D3=%d D4=%d D5=%d D6=%d D7=%d",
           D0_GPIO, D1_GPIO, D2_GPIO, D3_GPIO, D4_GPIO, D5_GPIO, D6_GPIO, D7_GPIO);
  }
  else
  {
    log_ts("GPIO", "GPIO test DISABLED");
  }
  gpio_test_init();

  // I2C-1 init once (scan runs incrementally after each GPIO cycle)
  if (ENABLE_I2C_SCAN)
  {
    log_ts("I2C", "I2C-1 scan enabled (after each GPIO cycle)");
  }
  else
  {
    log_ts("I2C", "I2C-1 scan DISABLED");
  }
  i2c1_init_once();

  next_tx_ms = millis() + 1500;
}

/* ================================
   LOOP
   ================================ */
void loop()
{
  // GPIO test tick (additive; non-blocking; triggers I2C scan start on wrap)
  gpio_test_tick();

  // I2C scan tick (non-blocking; one address per loop)
  i2c1_scan_tick();

  poll_uart_rx();

  if (!ENABLE_BROKER_UART) return;

  // Outbound timeout handling (only relevant if this side transmits)
  if (awaiting_ack)
  {
    if (millis() - last_send_ms > ACK_TIMEOUT_MS)
    {
      log_ts("FLOW", "ACK timeout for frame %lu → resend", (unsigned long)tx_frame_id);
      send_cached_frame();
      return;
    }
  }

  // Optional periodic TX to test duplex
  if (ENABLE_BROKER_PERIODIC_TX && !awaiting_ack)
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
