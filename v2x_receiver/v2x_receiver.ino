/*
 * V2X RECEIVER — ESP32 (30-pin)
 * ================================
 * Simulates a nearby vehicle receiving V2X thermal runaway alerts
 * from the BMS ESP32-S3 via ESP-NOW broadcast.
 *
 * WHAT IT DOES:
 *   - Listens for ESP-NOW broadcast packets from any BMS node
 *   - Decodes the 32-byte alert packet
 *   - Displays risk level, temperatures, gradients, time-to-critical
 *   - Built-in LED blinks on alert (WARNING=slow, HIGH=fast, DANGER=solid)
 *
 * HARDWARE:
 *   - Any ESP32 board (30-pin DevKit, NodeMCU-32S, etc.)
 *   - No extra wiring needed — just USB for power + serial monitor
 *   - Built-in LED on GPIO 2 (most 30-pin boards)
 *
 * BOARD SETTINGS (Arduino IDE):
 *   - Board: ESP32 Dev Module
 *   - Upload Speed: 921600
 *   - Flash Frequency: 80MHz
 *
 * USAGE:
 *   1. Flash this to your 30-pin ESP32
 *   2. Open serial monitor at 115200
 *   3. Power on the BMS (ESP32-S3 with V30 code)
 *   4. When BMS risk >= 30%, you'll see alerts on serial
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Must match the BMS transmitter struct exactly
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint8_t  alert_level;
    uint8_t  combined_risk;
    uint8_t  ai_risk;
    uint8_t  ai_class;
    int16_t  max_temp_x10;
    int16_t  dt_dt_x100;
    int16_t  time_to_crit_x10;
    uint16_t pack_mv;
    int16_t  intra_x100;
    int16_t  term_x100;
    int16_t  inter_x100;
    int16_t  v_imbal_x10000;
    uint8_t  thermal_gate;
    uint8_t  ai_confirmed;
    uint32_t uptime_sec;
    uint8_t  reserved[2];
} v2x_alert_t;
#pragma pack(pop)

#define V2X_MAGIC   0xB55A
#define LED_PIN     2       // built-in LED on most 30-pin ESP32 boards

// Stats
uint32_t alerts_received = 0;
uint32_t last_alert_ms = 0;
uint8_t  last_alert_level = 0;
uint32_t boot_time = 0;

const char* levelStr[] = {"CLEAR", "WARNING", "HIGH", "DANGER"};
const char* classStr[] = {"NORMAL", "WARNING", "HIGH", "DANGER"};

// ═══════════════════════════════════════════════════════════════════════════════
// ESP-NOW Receive Callback — fires every time a V2X packet arrives
// ═══════════════════════════════════════════════════════════════════════════════
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(v2x_alert_t)) {
        Serial.printf("  [V2X] Bad packet size: %d (expected %d)\n", len, sizeof(v2x_alert_t));
        return;
    }

    v2x_alert_t alert;
    memcpy(&alert, data, sizeof(alert));

    // Verify magic number
    if (alert.magic != V2X_MAGIC) {
        Serial.printf("  [V2X] Bad magic: 0x%04X\n", alert.magic);
        return;
    }

    alerts_received++;
    last_alert_ms = millis();
    last_alert_level = alert.alert_level;

    // Decode values
    float max_temp   = alert.max_temp_x10 / 10.0f;
    float dt_dt      = alert.dt_dt_x100 / 100.0f;
    float ttc        = alert.time_to_crit_x10 / 10.0f;
    float pack_v     = alert.pack_mv / 1000.0f;
    float intra      = alert.intra_x100 / 100.0f;
    float term       = alert.term_x100 / 100.0f;
    float inter      = alert.inter_x100 / 100.0f;
    float v_imbal    = alert.v_imbal_x10000 / 10000.0f;

    // Print sender MAC from info struct
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             info->src_addr[0], info->src_addr[1], info->src_addr[2],
             info->src_addr[3], info->src_addr[4], info->src_addr[5]);

    // Display alert
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════╗");

    if (alert.alert_level == 3) {
        Serial.println("║  * DANGER *  THERMAL RUNAWAY ALERT  * DANGER *  ║");
    } else if (alert.alert_level == 2) {
        Serial.println("║  !! HIGH RISK !!  THERMAL EVENT DETECTED  !! HIGH !!   ║");
    } else if (alert.alert_level == 1) {
        Serial.println("║  ~ WARNING ~  THERMAL ANOMALY DEVELOPING  ~ WARNING ~  ║");
    } else {
        Serial.println("║  [OK] CLEAR — Conditions returning to normal           ║");
    }

    Serial.println("╠══════════════════════════════════════════════════════════╣");
    Serial.printf( "║  FROM: %s  (BMS Node)              ║\n", mac_str);
    Serial.printf( "║  Alert: %-8s  Risk: %3d%%  AI: %3d%% (%s)        \n",
                   levelStr[alert.alert_level], alert.combined_risk,
                   alert.ai_risk, classStr[alert.ai_class]);
    Serial.println("╠══════════════════════════════════════════════════════════╣");
    Serial.printf( "║  Pack Voltage:  %.2f V                               \n", pack_v);
    Serial.printf( "║  V Imbalance:   %.4f V                              \n", v_imbal);
    Serial.printf( "║  Max Temp:      %.1f °C                              \n", max_temp);
    Serial.printf( "║  dT/dt:         %+.2f °C/min                        \n", dt_dt);
    Serial.println("╠──────────────────────────────────────────────────────────╣");
    Serial.println("║  THREE-TIER GRADIENT ANALYSIS:                          ║");
    Serial.printf( "║    Intra-cell:  %.2f °C  %s                       \n",
                   intra, intra > 5.0 ? "[DANGER]" : intra > 2.0 ? "[WARN]" : "[OK]");
    Serial.printf( "║    Terminal:    %.2f °C  %s                       \n",
                   term, term > 8.0 ? "[DANGER]" : term > 3.0 ? "[WARN]" : "[OK]");
    Serial.printf( "║    Inter-cell:  %.2f °C  %s                       \n",
                   inter, inter > 6.0 ? "[DANGER]" : inter > 3.0 ? "[WARN]" : "[OK]");
    Serial.println("╠──────────────────────────────────────────────────────────╣");
    Serial.printf( "║  Thermal Gate:  %s                                \n",
                   alert.thermal_gate ? "ANOMALY ACTIVE" : "FLAT (no thermal event)");
    Serial.printf( "║  AI Confirmed:  %s                                \n",
                   alert.ai_confirmed ? "YES — sustained warning" : "NO — not yet confirmed");
    Serial.println("╠══════════════════════════════════════════════════════════╣");

    if (ttc < 60.0f && ttc > 0) {
        Serial.printf( "║  >>> TIME TO CRITICAL: %.1f MINUTES <<<                \n", ttc);
        Serial.println("║  >>> RECOMMEND: INCREASE FOLLOWING DISTANCE <<<         ║");
    } else {
        Serial.printf( "║  Time to Critical: >60 min (monitoring)               \n");
    }

    Serial.printf( "║  BMS Uptime: %lu sec  |  Alerts received: %lu          \n",
                   alert.uptime_sec, alerts_received);
    Serial.println("╚══════════════════════════════════════════════════════════╝");
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════════
// LED Control — visual alert indicator
// ═══════════════════════════════════════════════════════════════════════════════
void updateLED() {
    uint32_t now = millis();

    // If no alert received in last 10 seconds, LED off
    if (now - last_alert_ms > 10000) {
        digitalWrite(LED_PIN, LOW);
        return;
    }

    switch (last_alert_level) {
        case 3:  // DANGER — LED solid ON
            digitalWrite(LED_PIN, HIGH);
            break;
        case 2:  // HIGH — fast blink (200ms)
            digitalWrite(LED_PIN, (now / 200) % 2);
            break;
        case 1:  // WARNING — slow blink (1000ms)
            digitalWrite(LED_PIN, (now / 1000) % 2);
            break;
        default: // CLEAR — LED off
            digitalWrite(LED_PIN, LOW);
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════╗");
    Serial.println("║         V2X RECEIVER — NEARBY VEHICLE SIMULATOR        ║");
    Serial.println("║         ESP-NOW Broadcast Listener                     ║");
    Serial.println("║         Waiting for BMS thermal alerts...              ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝");
    Serial.println();

    // Init WiFi — connect to SAME hotspot as BMS to match channel
    WiFi.mode(WIFI_STA);
    WiFi.begin("realme narzo 60 Pro 5G", "RROOzzAA");
    Serial.print("WiFi: Connecting to hotspot (for channel sync)... ");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi: CONNECTED! Channel=%d IP=%s\n",
                      WiFi.channel(), WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi: FAILED — V2X may not work (channel mismatch)");
    }

    // Print MAC address
    Serial.printf("Receiver MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("WiFi Channel: %d\n", WiFi.channel());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("ESP-NOW: READY — listening for V2X broadcasts...\n");
    Serial.println("(When the BMS detects risk >= 30%, alerts will appear here)\n");

    boot_time = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    updateLED();

    // Periodic status every 30 seconds
    static uint32_t last_status = 0;
    uint32_t now = millis();

    if (now - last_status >= 30000) {
        last_status = now;
        uint32_t uptime = (now - boot_time) / 1000;
        Serial.printf("[V2X RX] Uptime: %lus | Alerts: %lu | Last: %lus ago\n",
                      uptime, alerts_received,
                      alerts_received > 0 ? (now - last_alert_ms) / 1000 : 0);
    }

    delay(10);  // small delay to prevent watchdog
}
