/*
 * ESP32-S3 BMS INTELLIGENCE LAYER — V30 (EDGE-AI + V2X)
 * =======================================================
 * V29-FIX + V2X ESP-NOW broadcast alerts
 *
 * FEATURES:
 *   - CAN bus receive from STM32 safety layer
 *   - 3-tier gradient analysis (intra, terminal, inter-cell)
 *   - Edge-AI 1D CNN thermal runaway prediction
 *   - Thermal gate + persistence filter (false positive elimination)
 *   - OLED SPI display (4 pages)
 *   - WiFi + Ubidots cloud dashboard (10 variables)
 *   - V2X ESP-NOW broadcast alerts to nearby vehicles
 *
 * V2X ALERT SYSTEM:
 *   Protocol: ESP-NOW (peer-to-peer, no WiFi/internet needed)
 *   Mode: Broadcast (FF:FF:FF:FF:FF:FF) — all nearby devices
 *   Trigger: combined_risk >= 30% (WARNING+)
 *   Packet: 32 bytes — risk, AI class, temps, time-to-critical
 *   Range: ~200m line-of-sight (like real V2X range)
 *   Coexists with WiFi on same channel
 *
 * EDGE-AI MODEL:
 *   Architecture: Conv1D(32)->Conv1D(64)->Conv1D(32)->GAP->Dense(32)->Dense(4)
 *   Input: 30 timesteps x 14 features (float32)
 *   Output: 4 classes [NORMAL, WARNING, HIGH, DANGER]
 *   Size: ~29 KB (int8 quantized)
 *   Inference: <50ms on ESP32-S3
 *
 * HARDWARE: Same as V29 — no wiring changes needed!
 * LIBRARIES: mcp_can, Adafruit SSD1306+GFX, TensorFlowLite_ESP32, esp_now
 * BOARD: ESP32S3 Dev Module, USB CDC On Boot: Enabled, PSRAM: OPI
 */

#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <mcp_can.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <esp_now.h>
#include <esp_wifi.h>

// TensorFlow Lite Micro (Chirale_TensorFlowLite)
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Our trained model (place bms_cnn_model.h in same folder as this .ino)
#include "bms_cnn_model.h"

// TFLite globals
constexpr int kTensorArenaSize = 32 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
const tflite::Model* tfl_model = nullptr;
tflite::MicroInterpreter* tfl_interpreter = nullptr;
TfLiteTensor* tfl_input = nullptr;
TfLiteTensor* tfl_output = nullptr;

// ── WiFi + Ubidots ──────────────────────────────────────────────────────────
#define WIFI_SSID     "realme narzo 60 Pro 5G"
#define WIFI_PASS     "RROOzzAA"
#define UBIDOTS_TOKEN "BBUS-1boXHMw68LHu95rSM1VnnfoZ0tldN4"
#define DEVICE_LABEL  "bms-v29"
#define UBIDOTS_URL   "http://industrial.api.ubidots.com/api/v1.6/devices/" DEVICE_LABEL
#define UBIDOTS_MS    10000

// ── V2X ESP-NOW ─────────────────────────────────────────────────────────────
#define V2X_ALERT_THRESHOLD  30       // combined_risk >= this triggers V2X broadcast
#define V2X_SEND_INTERVAL_MS 3000     // broadcast every 3s when alert active
#define V2X_MAGIC            0xB55A   // packet identifier
uint8_t v2x_broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // broadcast

// V2X alert packet — 32 bytes, sent via ESP-NOW broadcast
typedef struct __attribute__((packed)) {
    uint16_t magic;             // 0xB55A — identifies this as BMS V2X alert
    uint8_t  alert_level;       // 0=CLEAR, 1=WARNING, 2=HIGH, 3=DANGER
    uint8_t  combined_risk;     // 0-100%
    uint8_t  ai_risk;           // 0-100% (corrected)
    uint8_t  ai_class;          // 0=NORMAL, 1=WARNING, 2=HIGH, 3=DANGER
    int16_t  max_temp_x10;      // max temp * 10 (e.g., 235 = 23.5°C)
    int16_t  dt_dt_x100;        // dT/dt * 100 (e.g., 50 = 0.50°C/min)
    int16_t  time_to_crit_x10;  // time to critical * 10 (e.g., 150 = 15.0 min)
    uint16_t pack_mv;           // pack voltage in mV
    int16_t  intra_x100;        // intra gradient * 100
    int16_t  term_x100;         // terminal gradient * 100
    int16_t  inter_x100;        // inter gradient * 100
    int16_t  v_imbal_x10000;    // voltage imbalance * 10000
    uint8_t  thermal_gate;      // 0=flat, 1=anomaly
    uint8_t  ai_confirmed;      // 0=no, 1=confirmed
    uint32_t uptime_sec;        // seconds since boot
    uint8_t  reserved[2];       // pad to 32 bytes
} v2x_alert_t;

uint8_t  espnow_ready = 0;
uint32_t v2x_sent = 0;
uint32_t v2x_fail = 0;
uint32_t last_v2x_ms = 0;

// ── Pins ────────────────────────────────────────────────────────────────────
#define SPI_SCK_PIN  12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define MCP_CS_PIN   10
#define MCP_INT_PIN  14
#define OLED_DC      21
#define OLED_CS      17
#define OLED_RST     18
#define OLED_WIDTH   128
#define OLED_HEIGHT  64

MCP_CAN CAN(MCP_CS_PIN);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);

// ── Thresholds ──────────────────────────────────────────────────────────────
#define DT_DT_WARN       0.5f
#define DT_DT_DANGER     1.5f
#define INTRA_WARN       2.0f
#define INTRA_DANGER     5.0f
#define TERM_WARN        3.0f
#define TERM_DANGER      8.0f
#define INTER_WARN       3.0f
#define INTER_DANGER     6.0f
#define ABS_TEMP_WARN    40.0f
#define ABS_TEMP_DANGER  50.0f
#define ABS_TEMP_CRIT    55.0f
#define DV_DT_WARN       0.02f
#define DV_DT_DANGER     0.08f
#define VIMBAL_WARN      0.02f
#define VIMBAL_DANGER    0.05f

// ── AI Post-Inference Correction Thresholds ─────────────────────────────────
#define THERMAL_GATE_DTDT     0.5f    // dT/dt must exceed this to count as thermal anomaly
#define THERMAL_GATE_ABS_TEMP 35.0f   // absolute temp must exceed this
#define THERMAL_GATE_INTER    3.0f    // inter-cell spread must exceed this
#define THERMAL_DISCOUNT      0.2f    // multiply AI risk by this when thermals are flat
#define WARNING_PERSIST_FRAMES 5      // consecutive WARNING+ frames before escalation
#define WARNING_CAP_NORMAL    25      // cap AI risk at this until persistence confirmed

#define HISTORY_SIZE      30
#define MIN_HIST          20
#define ANALYSIS_MS       3000
#define OLED_PAGE_MS      3000
#define TEMP_JUMP         5.0f
#define NUM_PAGES         4

#define RISK_NORMAL      30
#define RISK_WARNING     60
#define RISK_HIGH        80

// AI prediction results
int ai_class = 0;          // 0=NORMAL, 1=WARNING, 2=HIGH, 3=DANGER
float ai_confidence = 0.0f;
int ai_risk_score = 0;     // mapped to 0-100% (RAW from CNN)
int ai_risk_corrected = 0; // after thermal gate + persistence filter
uint32_t ai_inference_ms = 0;
uint8_t tfl_ready = 0;
uint8_t thermal_anomaly_flag = 0;   // 1 if thermal anomaly detected
int warning_persist_count = 0;       // consecutive WARNING+ frame counter
uint8_t ai_warning_confirmed = 0;   // 1 if persistence threshold met

// ── Data ────────────────────────────────────────────────────────────────────
float raw_temps[8]={0}, cellV[4]={0};
float packV=0, minV=0;
uint8_t bms_status=0, stm_loop=0;
uint8_t got_v=0, got_t1=0, got_t2=0, got_pack=0;

float filt_temps[8]={0};
uint8_t temp_seeded[8]={0};

float temp_history[8][HISTORY_SIZE];
float volt_history[4][HISTORY_SIZE];
uint32_t hist_time[HISTORY_SIZE];
int hist_idx=0, hist_count=0;

float dT_dt[8]={0}, dV_dt[4]={0}, intra_grad[3]={0};
float term_grad=0, inter_grad=0, v_imbalance=0;
float max_dT_dt=0, max_temp=0, mins_to_crit=99.0f;
int risk_score=0;
int combined_risk=0;  // max of threshold and CORRECTED AI risk

uint32_t rx_count=0, last_rx_ms=0;
uint32_t last_analysis_ms=0, last_sample_ms=0, last_oled_ms=0;
uint32_t last_ubidots_ms=0;
uint8_t oled_page=0;
uint8_t oled_ok=0;
uint8_t wifi_connected=0;
uint32_t ubidots_ok=0, ubidots_fail=0;

// ── Helpers ─────────────────────────────────────────────────────────────────
static uint16_t getU16(uint8_t*b,uint8_t o){return((uint16_t)b[o]<<8)|b[o+1];}
static int16_t getI16(uint8_t*b,uint8_t o){return(int16_t)(((uint16_t)b[o]<<8)|b[o+1]);}
static float fabsf_l(float x){return x<0?-x:x;}
const char* riskLabel(int s){if(s>=RISK_HIGH)return"DANGER";if(s>=RISK_WARNING)return"HIGH";if(s>=RISK_NORMAL)return"WARNING";return"NORMAL";}
const char* riskIcon(int s){if(s>=RISK_HIGH)return"[!!!]";if(s>=RISK_WARNING)return"[!! ]";if(s>=RISK_NORMAL)return"[!  ]";return"[ OK]";}
const char* aiClassLabel(int c){
    switch(c){case 0:return"NORMAL";case 1:return"WARNING";case 2:return"HIGH";case 3:return"DANGER";}
    return"?";
}

// ── Temp filter ─────────────────────────────────────────────────────────────
void filterTemp(uint8_t i,float raw){
    if(!temp_seeded[i]){filt_temps[i]=raw;temp_seeded[i]=1;return;}
    if(fabsf_l(raw-filt_temps[i])>TEMP_JUMP)return;
    filt_temps[i]=0.7f*filt_temps[i]+0.3f*raw;
}

// ── History ─────────────────────────────────────────────────────────────────
void recordSample(){
    for(int i=0;i<8;i++)temp_history[i][hist_idx]=filt_temps[i];
    for(int i=0;i<4;i++)volt_history[i][hist_idx]=cellV[i];
    hist_time[hist_idx]=millis();
    hist_idx=(hist_idx+1)%HISTORY_SIZE;
    if(hist_count<HISTORY_SIZE)hist_count++;
}

// ── Gradient calc ───────────────────────────────────────────────────────────
void calculateGradients(){
    if(hist_count<MIN_HIST)return;
    int na=3;if(hist_count<6)na=1;
    float ot[8]={0},nt[8]={0},ov[4]={0},nv[4]={0};
    uint32_t otm=0,ntm=0;
    for(int k=0;k<na;k++){
        int oi,ni;
        if(hist_count<HISTORY_SIZE){oi=k;ni=hist_count-1-k;}
        else{oi=(hist_idx+k)%HISTORY_SIZE;ni=(hist_idx-1-k+HISTORY_SIZE)%HISTORY_SIZE;}
        for(int i=0;i<8;i++){ot[i]+=temp_history[i][oi];nt[i]+=temp_history[i][ni];}
        for(int i=0;i<4;i++){ov[i]+=volt_history[i][oi];nv[i]+=volt_history[i][ni];}
        otm+=hist_time[oi];ntm+=hist_time[ni];
    }
    for(int i=0;i<8;i++){ot[i]/=na;nt[i]/=na;}
    for(int i=0;i<4;i++){ov[i]/=na;nv[i]/=na;}
    otm/=na;ntm/=na;
    float ds=(ntm-otm)/1000.0f;if(ds<5.0f)return;
    float dm=ds/60.0f;
    max_dT_dt=0;
    for(int i=0;i<8;i++){dT_dt[i]=(nt[i]-ot[i])/dm;if(fabsf_l(dT_dt[i])>fabsf_l(max_dT_dt))max_dT_dt=dT_dt[i];}
    for(int i=0;i<4;i++)dV_dt[i]=(nv[i]-ov[i])/dm;
    intra_grad[0]=fabsf_l(filt_temps[0]-filt_temps[1]);
    intra_grad[1]=fabsf_l(filt_temps[2]-filt_temps[3]);
    intra_grad[2]=fabsf_l(filt_temps[4]-filt_temps[5]);
    term_grad=fabsf_l(filt_temps[6]-filt_temps[7]);
    float bmin=filt_temps[0],bmax=filt_temps[0];
    for(int i=1;i<6;i++){if(filt_temps[i]<bmin)bmin=filt_temps[i];if(filt_temps[i]>bmax)bmax=filt_temps[i];}
    inter_grad=bmax-bmin;
    max_temp=filt_temps[0];
    for(int i=1;i<8;i++)if(filt_temps[i]>max_temp)max_temp=filt_temps[i];
    float vmin=cellV[0],vmax=cellV[0];
    for(int i=1;i<4;i++){if(cellV[i]<vmin)vmin=cellV[i];if(cellV[i]>vmax)vmax=cellV[i];}
    v_imbalance=vmax-vmin;
}

// ── Threshold-based Risk ────────────────────────────────────────────────────
int calculateRisk(){
    if(hist_count<MIN_HIST)return 0;
    int s=0;float ad=fabsf_l(max_dT_dt);
    if(ad>DT_DT_DANGER)s+=30;else if(ad>DT_DT_WARN)s+=15;
    float mi=intra_grad[0];if(intra_grad[1]>mi)mi=intra_grad[1];if(intra_grad[2]>mi)mi=intra_grad[2];
    if(mi>INTRA_DANGER)s+=25;else if(mi>INTRA_WARN)s+=12;
    if(term_grad>TERM_DANGER)s+=25;else if(term_grad>TERM_WARN)s+=12;
    if(inter_grad>INTER_DANGER)s+=15;else if(inter_grad>INTER_WARN)s+=8;
    if(max_temp>ABS_TEMP_DANGER)s+=30;else if(max_temp>ABS_TEMP_WARN)s+=15;
    if(v_imbalance>VIMBAL_DANGER)s+=10;else if(v_imbalance>VIMBAL_WARN)s+=5;
    return s>100?100:s;
}

float estimateTimeToCrit(){
    if(max_dT_dt>0.01f){float r=ABS_TEMP_CRIT-max_temp;return r<=0?0.0f:r/max_dT_dt;}
    return 99.0f;
}

// ═══════════════════════════════════════════════════════════════════════════════
// EDGE-AI: TFLite Inference + Post-Inference Correction
// ═══════════════════════════════════════════════════════════════════════════════

void setupTFLite() {
    Serial.print("TFLite: Loading model... ");

    tfl_model = tflite::GetModel(bms_cnn_model);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("FAIL! Version mismatch\n");
        return;
    }

    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(
        tfl_model, resolver, tensor_arena, kTensorArenaSize);
    tfl_interpreter = &static_interpreter;

    if (tfl_interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("FAIL! AllocateTensors");
        return;
    }

    tfl_input = tfl_interpreter->input(0);
    tfl_output = tfl_interpreter->output(0);

    Serial.println("OK!");
    Serial.printf("  Model: %d bytes\n", bms_cnn_model_len);
    Serial.printf("  Input:  [%d x %d]\n", tfl_input->dims->data[1], tfl_input->dims->data[2]);
    Serial.printf("  Output: [%d]\n", tfl_output->dims->data[1]);
    Serial.printf("  Arena:  %d / %d bytes\n", tfl_interpreter->arena_used_bytes(), kTensorArenaSize);

    tfl_ready = 1;
}

void runInference() {
    if (!tfl_ready || hist_count < HISTORY_SIZE) return;

    uint32_t t0 = millis();

    // Fill input tensor with normalized history data
    for (int t = 0; t < HISTORY_SIZE; t++) {
        int hi = (hist_idx + t) % HISTORY_SIZE;
        int offset = t * MODEL_NUM_FEATURES;

        // Features 0-3: cell voltages (normalized)
        for (int c = 0; c < 4; c++) {
            float raw = volt_history[c][hi];
            tfl_input->data.f[offset + c] = (raw - feat_min[c]) / feat_range[c];
        }

        // Features 4-9: cell body temps
        float body_temps[6] = {
            temp_history[1][hi], temp_history[6][hi],  // C1L, C1R
            temp_history[2][hi], temp_history[3][hi],  // C2L, C2R
            temp_history[5][hi], temp_history[0][hi]   // C3L, C3R
        };
        for (int s = 0; s < 6; s++) {
            tfl_input->data.f[offset + 4 + s] = (body_temps[s] - feat_min[4+s]) / feat_range[4+s];
        }

        // Features 10-11: terminal temps (Pos=S7, Neg=S4)
        tfl_input->data.f[offset + 10] = (temp_history[7][hi] - feat_min[10]) / feat_range[10];
        tfl_input->data.f[offset + 11] = (temp_history[4][hi] - feat_min[11]) / feat_range[11];

        // Feature 12: max dT/dt
        tfl_input->data.f[offset + 12] = (max_dT_dt - feat_min[12]) / feat_range[12];

        // Feature 13: voltage imbalance
        tfl_input->data.f[offset + 13] = (v_imbalance - feat_min[13]) / feat_range[13];
    }

    // Run inference
    if (tfl_interpreter->Invoke() != kTfLiteOk) {
        Serial.println("  AI: Inference FAILED!");
        return;
    }

    ai_inference_ms = millis() - t0;

    // Find highest probability class
    float max_prob = 0;
    ai_class = 0;
    for (int c = 0; c < MODEL_NUM_CLASSES; c++) {
        float prob = tfl_output->data.f[c];
        if (prob > max_prob) {
            max_prob = prob;
            ai_class = c;
        }
    }
    ai_confidence = max_prob;

    // Map AI class to risk score (0-100%) — RAW score from CNN
    switch (ai_class) {
        case 0: ai_risk_score = (int)(5 * (1.0f - ai_confidence));  break;
        case 1: ai_risk_score = 30 + (int)(29 * ai_confidence);     break;
        case 2: ai_risk_score = 60 + (int)(19 * ai_confidence);     break;
        case 3: ai_risk_score = 80 + (int)(20 * ai_confidence);     break;
    }
    if (ai_risk_score > 100) ai_risk_score = 100;

    // ═══════════════════════════════════════════════════════════════════
    // POST-INFERENCE CORRECTION — eliminates false positives
    // ═══════════════════════════════════════════════════════════════════
    ai_risk_corrected = ai_risk_score;  // start with raw CNN score

    // ── Step 1: Thermal Gate ────────────────────────────────────────
    // Thermal runaway on LiFePO4 requires BOTH voltage anomaly AND
    // thermal signature. If thermals are flat, discount AI risk by 80%.
    // This fixes the 58% WARNING on static voltage imbalance alone.
    thermal_anomaly_flag = 0;
    if (fabsf_l(max_dT_dt) > THERMAL_GATE_DTDT) thermal_anomaly_flag = 1;   // temp rising fast
    if (max_temp > THERMAL_GATE_ABS_TEMP)        thermal_anomaly_flag = 1;   // absolute hot
    if (inter_grad > THERMAL_GATE_INTER)          thermal_anomaly_flag = 1;   // cell spread

    if (!thermal_anomaly_flag) {
        // Thermals are flat — discount AI risk heavily
        // Example: 58% raw → 58 * 0.2 = 12% → correctly NORMAL
        ai_risk_corrected = (int)(ai_risk_corrected * THERMAL_DISCOUNT);
    }

    // ── Step 2: Persistence Filter ──────────────────────────────────
    // A single WARNING frame is meaningless. Thermal runaway builds
    // over minutes. Require N consecutive WARNING+ frames before
    // treating it as real. This kills startup transients at hist=30/30.
    if (ai_risk_corrected >= RISK_NORMAL) {  // WARNING or above (>=30%)
        warning_persist_count++;
    } else {
        warning_persist_count = 0;  // reset on any NORMAL frame
    }

    ai_warning_confirmed = (warning_persist_count >= WARNING_PERSIST_FRAMES) ? 1 : 0;

    if (!ai_warning_confirmed) {
        // Not yet sustained — cap at NORMAL range
        if (ai_risk_corrected > WARNING_CAP_NORMAL) {
            ai_risk_corrected = WARNING_CAP_NORMAL;
        }
    }

    // ── Combined risk: max of threshold-based and CORRECTED AI ──────
    // Step 3: EMA Smoothing — eliminates frame-to-frame noise
    // Alpha=0.3 means 30% new value + 70% previous — stable but responsive
    // Responds to real events within 2-3 frames (~6-9 sec)
    static float ai_risk_smooth = 0.0f;
    ai_risk_smooth = 0.3f * ai_risk_corrected + 0.7f * ai_risk_smooth;
    ai_risk_corrected = (int)(ai_risk_smooth + 0.5f);  // round to nearest int

    combined_risk = (ai_risk_corrected > risk_score) ? ai_risk_corrected : risk_score;
}

// ═══════════════════════════════════════════════════════════════════════════════
// V2X: ESP-NOW Broadcast Alerts
// ═══════════════════════════════════════════════════════════════════════════════

void onV2XSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    // Callback — runs after each ESP-NOW send
    if (status == ESP_NOW_SEND_SUCCESS) v2x_sent++;
    else v2x_fail++;
}

void setupESPNOW() {
    Serial.print("ESP-NOW V2X... ");

    if (esp_now_init() != ESP_OK) {
        Serial.println("FAIL! esp_now_init");
        return;
    }

    esp_now_register_send_cb(onV2XSent);

    // Add broadcast peer
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, v2x_broadcast_addr, 6);
    peer.channel = 0;   // use current WiFi channel
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("FAIL! add_peer");
        return;
    }

    espnow_ready = 1;
    Serial.println("OK! (broadcast mode)");
}

void sendV2XAlert() {
    if (!espnow_ready) return;
    if (combined_risk < V2X_ALERT_THRESHOLD) return;  // only broadcast on WARNING+

    // Determine alert level
    uint8_t level = 0;
    if (combined_risk >= RISK_HIGH)       level = 3;  // DANGER
    else if (combined_risk >= RISK_WARNING) level = 2;  // HIGH
    else if (combined_risk >= RISK_NORMAL)  level = 1;  // WARNING

    float intra_max = intra_grad[0];
    if (intra_grad[1] > intra_max) intra_max = intra_grad[1];
    if (intra_grad[2] > intra_max) intra_max = intra_grad[2];

    // Build alert packet
    v2x_alert_t alert;
    memset(&alert, 0, sizeof(alert));
    alert.magic            = V2X_MAGIC;
    alert.alert_level      = level;
    alert.combined_risk    = (uint8_t)combined_risk;
    alert.ai_risk          = (uint8_t)ai_risk_corrected;
    alert.ai_class         = (uint8_t)ai_class;
    alert.max_temp_x10     = (int16_t)(max_temp * 10);
    alert.dt_dt_x100       = (int16_t)(max_dT_dt * 100);
    alert.time_to_crit_x10 = (int16_t)((mins_to_crit > 60.0f ? 60.0f : mins_to_crit) * 10);
    alert.pack_mv          = (uint16_t)(packV * 1000);
    alert.intra_x100       = (int16_t)(intra_max * 100);
    alert.term_x100        = (int16_t)(term_grad * 100);
    alert.inter_x100       = (int16_t)(inter_grad * 100);
    alert.v_imbal_x10000   = (int16_t)(v_imbalance * 10000);
    alert.thermal_gate     = thermal_anomaly_flag;
    alert.ai_confirmed     = ai_warning_confirmed;
    alert.uptime_sec       = millis() / 1000;

    esp_err_t result = esp_now_send(v2x_broadcast_addr, (uint8_t*)&alert, sizeof(alert));

    const char* levelStr[] = {"CLEAR", "WARNING", "HIGH", "DANGER"};
    Serial.printf("  V2X: BROADCAST %s risk=%d%% ttc=%.1fmin [sent=%lu fail=%lu]\n",
                  levelStr[level], combined_risk,
                  (mins_to_crit > 60.0f) ? 60.0f : mins_to_crit,
                  v2x_sent, v2x_fail);
}

// ═══════════════════════════════════════════════════════════════════════════════
// WiFi + UBIDOTS (fixed: fresh connection every send to avoid HTTP -11)
// ═══════════════════════════════════════════════════════════════════════════════

void connectWiFi() {
    Serial.printf("WiFi: Connecting to '%s'... ", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    if (oled_ok) {
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println("Connecting WiFi...");
        display.setCursor(0, 30);
        display.println(WIFI_SSID);
        display.display();
    }

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = 1;
        Serial.printf("\nWiFi: CONNECTED! IP=%s RSSI=%ddBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        wifi_connected = 0;
        Serial.println("\nWiFi: FAILED - continuing without cloud");
    }
}

void sendToUbidots() {
    if (!wifi_connected) return;
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        delay(3000);
        if (WiFi.status() != WL_CONNECTED) { wifi_connected = 0; return; }
    }

    // ── 10 variables (Ubidots STEM free tier = max 10) ──────────────
    // Battery:    1. pack_voltage  2. v_imbalance
    // Thermal:    3. max_temp      4. dt_dt
    // 3-TIER GRADIENTS (PROJECT NOVELTY):
    //             5. intra_gradient   — hotspot within a cell
    //             6. term_gradient    — terminal connection fault
    //             7. inter_gradient   — cell-to-cell spread
    // AI + Prediction:
    //             8. combined_risk    — final risk decision
    //             9. ai_risk_corrected — Edge-AI after correction
    //             10. time_to_critical — early warning countdown

    float intra_max = intra_grad[0];
    if (intra_grad[1] > intra_max) intra_max = intra_grad[1];
    if (intra_grad[2] > intra_max) intra_max = intra_grad[2];

    char payload[512];
    snprintf(payload, sizeof(payload),
        "{"
        "\"pack_voltage\":%.3f,"
        "\"v_imbalance\":%.4f,"
        "\"max_temp\":%.1f,"
        "\"dt_dt\":%.3f,"
        "\"intra_gradient\":%.2f,"
        "\"term_gradient\":%.2f,"
        "\"inter_gradient\":%.2f,"
        "\"combined_risk\":%d,"
        "\"ai_risk_corrected\":%d,"
        "\"time_to_critical\":%.1f"
        "}",
        packV,
        v_imbalance,
        max_temp,
        max_dT_dt,
        intra_max,
        term_grad,
        inter_grad,
        combined_risk,
        ai_risk_corrected,
        (mins_to_crit > 60.0f) ? 60.0f : mins_to_crit
    );

    // ── HTTP -11 FIX: fresh WiFiClient + HTTPClient every send ──────
    // Ubidots free tier drops idle connections — never reuse them
    WiFiClient wifiClient;
    HTTPClient http;
    http.begin(wifiClient, UBIDOTS_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Auth-Token", UBIDOTS_TOKEN);
    http.setTimeout(10000);
    http.setReuse(false);  // force connection close after each POST

    int httpCode = http.POST(payload);
    if (httpCode == 200 || httpCode == 201) {
        ubidots_ok++;
        Serial.printf("  UBIDOTS: OK (%d) [ok=%lu fail=%lu]\n", httpCode, ubidots_ok, ubidots_fail);
    } else {
        ubidots_fail++;
        Serial.printf("  UBIDOTS: FAIL (HTTP %d) [ok=%lu fail=%lu]\n", httpCode, ubidots_ok, ubidots_fail);
    }
    http.end();
    wifiClient.stop();  // force TCP socket closed
}

// ═══════════════════════════════════════════════════════════════════════════════
// OLED DISPLAY PAGES (updated with corrected AI info)
// ═══════════════════════════════════════════════════════════════════════════════

void drawHeader(const char* title) {
    display.fillRect(0, 0, OLED_WIDTH, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(2, 1);
    display.print(title);
    display.setCursor(72, 1);
    if (hist_count < MIN_HIST)
        display.print("INIT...");
    else
        display.printf("R:%d%%AI:%d%%", risk_score, ai_risk_corrected);
    display.setTextColor(SSD1306_WHITE);
}

void drawPageVoltage() {
    drawHeader("VOLTAGE");
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.printf("C1:%.3fV C2:%.3fV", cellV[0], cellV[1]);
    display.setCursor(0, 24);
    display.printf("C3:%.3fV C4:%.3fV", cellV[2], cellV[3]);
    display.setCursor(0, 36);
    display.printf("Pack: %.2fV", packV);
    display.setCursor(0, 46);
    display.printf("Min:%.3fV Imb:%.3fV", minV, v_imbalance);
    display.setCursor(0, 56);
    if (v_imbalance > VIMBAL_DANGER) display.print("!! IMBALANCE HIGH !!");
    else if (v_imbalance > VIMBAL_WARN) display.print("~ Imbalance warning");
    else display.print("Voltages OK");
}

void drawPageTemp() {
    drawHeader("TEMPS");
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.printf("C1: %.1f / %.1f C", filt_temps[0], filt_temps[1]);
    display.setCursor(0, 24);
    display.printf("C2: %.1f / %.1f C", filt_temps[2], filt_temps[3]);
    display.setCursor(0, 34);
    display.printf("C3: %.1f / %.1f C", filt_temps[4], filt_temps[5]);
    display.setCursor(0, 44);
    display.printf("Pos:%.1f Neg:%.1f", filt_temps[6], filt_temps[7]);
    display.setCursor(0, 56);
    display.printf("MAX: %.1f C %s", max_temp,
                   max_temp > ABS_TEMP_DANGER ? "DANGER!" :
                   max_temp > ABS_TEMP_WARN   ? "WARNING" : "OK");
}

void drawPageGradient() {
    drawHeader("GRADIENTS");
    display.setTextSize(1);
    if (hist_count < MIN_HIST) {
        display.setCursor(0, 20);
        display.printf("Collecting... %d/%d", hist_count, MIN_HIST);
        return;
    }
    display.setCursor(0, 14);
    display.printf("dT/dt:%+.2f %s", max_dT_dt,
                  fabsf_l(max_dT_dt)>DT_DT_DANGER?"!!":fabsf_l(max_dT_dt)>DT_DT_WARN?"! ":"OK");
    display.setCursor(0, 24);
    display.printf("Intra:%.1f %.1f %.1f", intra_grad[0], intra_grad[1], intra_grad[2]);
    display.setCursor(0, 34);
    display.printf("Term:%.1f Inter:%.1f", term_grad, inter_grad);
    display.setCursor(0, 44);
    display.printf("VImbal:%.3fV", v_imbalance);
    // AI inference info with correction status
    display.setCursor(0, 56);
    display.printf("AI:%s%s %d%%>%d%%", aiClassLabel(ai_class),
                   thermal_anomaly_flag ? "T" : "t",    // T=thermal, t=flat
                   ai_risk_score, ai_risk_corrected);
}

void drawPageRisk() {
    drawHeader("RISK+AI");
    display.setTextSize(1);
    if (hist_count < MIN_HIST) {
        display.setCursor(10, 20);
        display.print("Initializing...");
        display.setCursor(10, 35);
        display.printf("Samples: %d/%d", hist_count, MIN_HIST);
        return;
    }
    // Show combined risk large
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.printf("%3d%%", combined_risk);
    display.setTextSize(1);
    display.setCursor(55, 14);
    display.print(riskLabel(combined_risk));

    // Visual bar
    display.drawRect(0, 30, 128, 8, SSD1306_WHITE);
    int barW = (combined_risk * 124) / 100;
    if (barW > 0) display.fillRect(2, 32, barW, 4, SSD1306_WHITE);

    // Dual scores with correction info
    display.setCursor(0, 40);
    display.printf("Thr:%d%% AI:%d%%>%d%%", risk_score, ai_risk_score, ai_risk_corrected);

    display.setCursor(0, 50);
    if (mins_to_crit < 60.0f && mins_to_crit > 0)
        display.printf("Crit:%.0fm %s%s", mins_to_crit,
                       thermal_anomaly_flag ? "THRM" : "flat",
                       ai_warning_confirmed ? " CONF" : "");
    else
        display.printf(">60m %s%s",
                       thermal_anomaly_flag ? "THRM" : "flat",
                       ai_warning_confirmed ? " CONFIRMED" : "");

    display.setCursor(0, 58);
    if (wifi_connected)
        display.printf("WiFi:%ddB UB:%lu", WiFi.RSSI(), ubidots_ok);
    else
        display.print("WiFi:OFFLINE");
}

void updateOLED() {
    if (!oled_ok) return;
    display.clearDisplay();
    switch (oled_page) {
        case 0: drawPageVoltage();  break;
        case 1: drawPageTemp();     break;
        case 2: drawPageGradient(); break;
        case 3: drawPageRisk();     break;
    }
    display.display();
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL OUTPUT (shows raw vs corrected AI risk)
// ═══════════════════════════════════════════════════════════════════════════════
void printAnalysis(){
    Serial.println("============================================================");
    Serial.printf("  BMS V30+AI+V2X  %s  THRESH:%3d%%  AI-RAW:%3d%%  AI-CORR:%3d%%  COMBINED:%3d%%\n",
                  riskIcon(combined_risk), risk_score, ai_risk_score, ai_risk_corrected, combined_risk);
    Serial.println("------------------------------------------------------------");
    Serial.printf("  VOLT: C1=%.3fV C2=%.3fV C3=%.3fV C4=%.3fV\n",
                  cellV[0],cellV[1],cellV[2],cellV[3]);
    Serial.printf("  PACK: %.3fV MIN=%.3fV Imbal=%.3fV %s\n",
                  packV,minV,v_imbalance,
                  v_imbalance>VIMBAL_DANGER?"[!]":v_imbalance>VIMBAL_WARN?"[~]":"[OK]");
    Serial.println("------------------------------------------------------------");
    Serial.printf("  TEMP: C1[%.1f/%.1f] C2[%.1f/%.1f] C3[%.1f/%.1f]\n",
                  filt_temps[0],filt_temps[1],filt_temps[2],
                  filt_temps[3],filt_temps[4],filt_temps[5]);
    Serial.printf("        Pos=%.1f Neg=%.1f MAX=%.1f %s\n",
                  filt_temps[6],filt_temps[7],max_temp,
                  max_temp>ABS_TEMP_DANGER?"[!]":max_temp>ABS_TEMP_WARN?"[~]":"[OK]");
    Serial.println("------------------------------------------------------------");
    if(hist_count<MIN_HIST){
        Serial.printf("  GRADIENTS: collecting... %d/%d\n",hist_count,MIN_HIST);
    }else{
        Serial.printf("  dT/dt max: %+.3f C/min %s\n",max_dT_dt,
                      fabsf_l(max_dT_dt)>DT_DT_DANGER?"[DANGER]":fabsf_l(max_dT_dt)>DT_DT_WARN?"[WARN]":"[OK]");
        Serial.printf("  Rates: S0=%+.2f S1=%+.2f S2=%+.2f S3=%+.2f\n",dT_dt[0],dT_dt[1],dT_dt[2],dT_dt[3]);
        Serial.printf("         S4=%+.2f S5=%+.2f S6=%+.2f S7=%+.2f\n",dT_dt[4],dT_dt[5],dT_dt[6],dT_dt[7]);
        Serial.printf("  Intra: C1=%.2f C2=%.2f C3=%.2f %s\n",intra_grad[0],intra_grad[1],intra_grad[2],
                      (intra_grad[0]>INTRA_DANGER||intra_grad[1]>INTRA_DANGER||intra_grad[2]>INTRA_DANGER)?"[DANGER]":
                      (intra_grad[0]>INTRA_WARN||intra_grad[1]>INTRA_WARN||intra_grad[2]>INTRA_WARN)?"[WARN]":"[OK]");
        Serial.printf("  Terminal: %.2f C %s\n",term_grad,term_grad>TERM_DANGER?"[DANGER]":term_grad>TERM_WARN?"[WARN]":"[OK]");
        Serial.printf("  Inter: %.2f C %s\n",inter_grad,inter_grad>INTER_DANGER?"[DANGER]":inter_grad>INTER_WARN?"[WARN]":"[OK]");
    }
    Serial.println("------------------------------------------------------------");
    Serial.printf("  dV/dt: C1=%+.4f C2=%+.4f C3=%+.4f C4=%+.4f V/min\n",
                  dV_dt[0],dV_dt[1],dV_dt[2],dV_dt[3]);
    Serial.println("------------------------------------------------------------");
    Serial.printf("  EDGE-AI: class=%s conf=%.1f%% raw_risk=%d%% time=%dms\n",
                  aiClassLabel(ai_class), ai_confidence*100, ai_risk_score, ai_inference_ms);
    Serial.printf("  THERMAL GATE: %s (dT/dt=%.3f max_T=%.1f inter=%.1f)\n",
                  thermal_anomaly_flag ? "ANOMALY DETECTED" : "FLAT — discount applied",
                  fabsf_l(max_dT_dt), max_temp, inter_grad);
    Serial.printf("  PERSISTENCE: %d/%d frames %s\n",
                  warning_persist_count, WARNING_PERSIST_FRAMES,
                  ai_warning_confirmed ? "** CONFIRMED **" : "(not yet)");
    Serial.printf("  AI CORRECTED: %d%% (raw %d%% -> gate -> persist -> %d%%)\n",
                  ai_risk_corrected, ai_risk_score, ai_risk_corrected);
    Serial.printf("  THRESHOLD: %d%%  AI-CORRECTED: %d%%  COMBINED: %d%%\n",
                  risk_score, ai_risk_corrected, combined_risk);
    Serial.println("------------------------------------------------------------");
    char bar[21];int filled=combined_risk/5;
    for(int i=0;i<20;i++)bar[i]=(i<filled)?'#':'-';bar[20]='\0';
    Serial.printf("  RISK: [%s] %3d%% %s\n",bar,combined_risk,riskLabel(combined_risk));
    if(mins_to_crit<60.0f&&mins_to_crit>0)Serial.printf("  TIME TO CRITICAL: %.1f min\n",mins_to_crit);
    else if(mins_to_crit<=0)Serial.printf("  TIME TO CRITICAL: *** AT CRITICAL ***\n");
    else Serial.printf("  TIME TO CRITICAL: >60 min (stable)\n");
    Serial.printf("  RX:%lu hist=%d/%d OLED=page%d WiFi:%s UB:ok=%lu fail=%lu\n",
                  rx_count,hist_count,HISTORY_SIZE,oled_page,
                  wifi_connected?"ON":"OFF",ubidots_ok,ubidots_fail);
    Serial.printf("  V2X: %s | sent=%lu fail=%lu | trigger>=%d%%\n",
                  espnow_ready?"READY":"OFF", v2x_sent, v2x_fail, V2X_ALERT_THRESHOLD);
    Serial.println("============================================================\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup(){
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n############################################");
    Serial.println("# ESP32-S3 BMS INTELLIGENCE V30           #");
    Serial.println("#   OLED+UBIDOTS+EDGE-AI+V2X(ESP-NOW)    #");
    Serial.println("#   Thermal Gate + Persistence Filter      #");
    Serial.println("############################################\n");

    // ── SPI bus ─────────────────────────────────────────────────────────
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    pinMode(MCP_CS_PIN, OUTPUT); digitalWrite(MCP_CS_PIN, HIGH);
    pinMode(OLED_CS, OUTPUT); digitalWrite(OLED_CS, HIGH);
    pinMode(MCP_INT_PIN, INPUT);

    // ── OLED ────────────────────────────────────────────────────────────
    Serial.print("OLED SSD1306 (SPI)... ");
    if (display.begin(SSD1306_SWITCHCAPVCC)) {
        oled_ok = 1; Serial.println("OK!");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 5);  display.println("BMS Intelligence");
        display.setCursor(10, 18); display.println("V30 AI+V2X");
        display.setCursor(10, 35); display.println("Loading AI model...");
        display.display();
    } else { Serial.println("FAIL!"); }

    // ── CAN ─────────────────────────────────────────────────────────────
    Serial.print("MCP2515... ");
    uint8_t r = CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ);
    if (r == CAN_OK) { Serial.println("OK!"); }
    else {
        Serial.printf("FAIL(%d)\n", r);
        while (1) { delay(2000); Serial.println("Retry...");
            if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ)==CAN_OK) { Serial.println("OK!"); break; }
        }
    }
    CAN.setMode(MCP_NORMAL);

    // ── TFLite Model ────────────────────────────────────────────────────
    setupTFLite();

    // ── WiFi ────────────────────────────────────────────────────────────
    connectWiFi();

    // ── ESP-NOW V2X (must init AFTER WiFi) ──────────────────────────────
    setupESPNOW();

    memset(temp_history, 0, sizeof(temp_history));
    memset(volt_history, 0, sizeof(volt_history));

    Serial.printf("OLED: %s | CAN: OK | TFLite: %s | WiFi: %s | V2X: %s\n",
                  oled_ok?"OK":"FAIL", tfl_ready?"OK":"FAIL",
                  wifi_connected?"OK":"FAIL", espnow_ready?"OK":"FAIL");
    Serial.println("Waiting for CAN...\n");

    if (oled_ok) {
        display.clearDisplay();
        display.setCursor(5, 5);  display.println("BMS V30 Ready");
        display.setCursor(5, 16); display.printf("AI: %s", tfl_ready?"LOADED":"FAIL");
        display.setCursor(5, 27); display.printf("WiFi: %s", wifi_connected?"OK":"FAIL");
        display.setCursor(5, 38); display.printf("V2X: %s", espnow_ready?"READY":"FAIL");
        display.setCursor(5, 49); display.println("Waiting for CAN...");
        display.display();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop(){
    uint32_t now = millis();

    // ── CAN receive ─────────────────────────────────────────────────────
    if (digitalRead(MCP_INT_PIN)==LOW || CAN.checkReceive()==CAN_MSGAVAIL) {
        uint32_t id=0; uint8_t len=0, buf[8]={0};
        if (CAN.readMsgBuf(&id,&len,buf)==CAN_OK) {
            rx_count++; last_rx_ms=now;
            switch(id) {
                case 0x100:
                    cellV[0]=getU16(buf,0)/1000.0f; cellV[1]=getU16(buf,2)/1000.0f;
                    cellV[2]=getU16(buf,4)/1000.0f; cellV[3]=getU16(buf,6)/1000.0f;
                    got_v=1; break;
                case 0x101:
                    raw_temps[0]=getI16(buf,0)/10.0f; raw_temps[1]=getI16(buf,2)/10.0f;
                    raw_temps[2]=getI16(buf,4)/10.0f; raw_temps[3]=getI16(buf,6)/10.0f;
                    for(int i=0;i<4;i++) filterTemp(i,raw_temps[i]);
                    got_t1=1; break;
                case 0x102:
                    raw_temps[4]=getI16(buf,0)/10.0f; raw_temps[5]=getI16(buf,2)/10.0f;
                    raw_temps[6]=getI16(buf,4)/10.0f; raw_temps[7]=getI16(buf,6)/10.0f;
                    for(int i=4;i<8;i++) filterTemp(i,raw_temps[i]);
                    got_t2=1; break;
                case 0x103:
                    packV=getU16(buf,0)/1000.0f; minV=getU16(buf,2)/1000.0f;
                    bms_status=buf[4]; stm_loop=buf[5];
                    got_pack=1; break;
            }
        }
    }

    // ── Record 1Hz ──────────────────────────────────────────────────────
    if (now-last_sample_ms>=1000 && got_v && got_t1 && got_t2) {
        last_sample_ms = now;
        recordSample();
    }

    // ── Analysis + AI + Serial every 3s ─────────────────────────────────
    if (now-last_analysis_ms >= ANALYSIS_MS) {
        last_analysis_ms = now;
        if (rx_count == 0) {
            Serial.println("  NO CAN - check wiring\n");
            if (oled_ok) {
                display.clearDisplay();
                display.setCursor(0, 10); display.println("NO CAN DATA!");
                display.setCursor(0, 30); display.println("Check wiring");
                display.display();
            }
            return;
        }
        calculateGradients();
        risk_score = calculateRisk();
        mins_to_crit = estimateTimeToCrit();

        // Run Edge-AI inference (includes thermal gate + persistence)
        runInference();

        printAnalysis();
    }

    // ── OLED page cycle every 3s ────────────────────────────────────────
    if (now-last_oled_ms >= OLED_PAGE_MS && got_v) {
        last_oled_ms = now;
        updateOLED();
        oled_page = (oled_page + 1) % NUM_PAGES;
    }

    // ── Ubidots upload every 10s ────────────────────────────────────────
    if (now-last_ubidots_ms >= UBIDOTS_MS && got_v && got_t1) {
        last_ubidots_ms = now;
        sendToUbidots();
    }

    // ── V2X ESP-NOW broadcast every 3s (only when risk >= WARNING) ──────
    if (now-last_v2x_ms >= V2X_SEND_INTERVAL_MS && got_v && got_t1) {
        last_v2x_ms = now;
        sendV2XAlert();
    }
}
