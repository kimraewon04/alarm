// alarmi — CLOCK NODE (라운드 디스플레이 상태 노드)
// Hardware: XIAO ESP32S3 + Seeed Round Display (GC9A01, 240x240)
// FQBN: esp32:esp32:XIAO_ESP32S3:PSRAM=opi
// Reference: TFT_eSPI_Clock_ex2 (Sprite double buffering & 8bpp fallback)
// GitHub: https://github.com/kimraewon04/alarm.git
//
// UI 사양:
//   - 여백 공간: back.png (back.c) 올리브 그린 배경색(0x7426)
//   - start 화면: 순수 start.c 이미지 렌더링
//   - timer 화면: timer.c + 중앙 남은 시간(00:00)만 표출
//   - mission 화면: 지정/랜덤 미션 이미지 (cup.c, mouse.c, phone.c, shake.c, circle.c) + 진행 링
//   - success (goodjob.c): 5초 유지 + 노란색(C_WARN) 외곽 테두리 blink 애니메이션 -> 시스템 종료
//   - fail (retry.c): 3초 유지 + 파란색(C_ACCENT) 외곽 테두리 blink 애니메이션 -> 미션 재시도

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <JPEGDecoder.h>
#include "images.h"

// ---- Network & MQTT Config ----
const char *WIFI_SSID  = "projectbee";
const char *WIFI_PASS  = "honeybear!";
const char *MQTT_HOST  = "192.168.0.27";
const int   MQTT_PORT  = 1883;
const char *CMD_TOPIC  = "wearable/minseo/cmd";
const char *UI_TOPIC   = "wearable/minseo/ui";
const char *MQTT_ID    = "xiao-minseo-node-clock";   // 노드 고유 ID

#define SCR            240                 // 240x240 원형 디스플레이
#define CX             (SCR / 2)
#define CY             (SCR / 2)
#define FAIL_HOLD_MS   5000                // 실패 화면 (retry.c) 5초 표시
#define SUCCESS_HOLD_MS 5000               // 성공 화면 (goodjob.c) 5초 표시 후 종료
#define FRAME_MS       50                  // 20 fps (50ms)

#define minimum(a, b)  (((a) < (b)) ? (a) : (b))

// ---- Color System ----
// back.png 배경색: RGB(119, 135, 53) -> RGB565: 0x7426
#define C_BG       0x7426              // Olive Green (back.c color)
#define C_DIM      0x53E9              // Darker Olive/Gray
#define C_TXT      TFT_WHITE
#define C_ACCENT   0x455F              // Cyan Blue
#define C_OK       0x3606              // Emerald Green
#define C_WARN     0xFDA6              // Amber Orange / Yellow
#define C_FAIL     0xF9E7              // Vivid Red

TFT_eSPI  tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);   // 더블버퍼 스프라이트

WiFiClient net;
PubSubClient mqtt(net);

// ---- State Machine Variables ----
enum St { S_IDLE, S_ARMED, S_RINGING, S_MISSION, S_FAIL, S_SUCCESS, S_SHUTDOWN };
static St       st          = S_IDLE;
static char     mLabel[32]  = "";       // 미션 라벨 (shaking / mouse / …)
static char     mIcon[16]   = "";       // 아이콘 이름 (swing / spin / mouse / cup / phone)
static char     mKind[16]   = "";       // gesture | object
static uint32_t armedMs     = 0;        // ARMED 진입 시각 (millis)
static uint32_t armedTotal  = 0;        // 카운트다운 총 ms
static uint32_t holdUntil   = 0;        // 특수 화면(retry 3s / goodjob 5s) 유효 완료 시각
static St       pendingSt   = S_IDLE;   // 지연 화면 종료 후 전환할 상태
static uint32_t heldMs      = 0, needMs = 3000;   // ui 진행 링
static uint32_t lastUi      = 0;        // ui 타임아웃 (2초 초과 시 링 감춤)
static uint32_t lastFrame   = 0, lastHb = 0;

// ---- JPEG Decoder to Sprite Helper ----
static void renderJPEGToSprite(TFT_eSprite *targetSpr, int xpos, int ypos) {
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  max_x += xpos;
  max_y += ypos;

  targetSpr->setSwapBytes(true);

  while (JpegDec.read()) {
    pImg = JpegDec.pImage;
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    if (win_w != mcu_w) {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int hh = 1; hh < (int)win_h; hh++) {
        p += mcu_w;
        for (int w = 0; w < (int)win_w; w++) {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    if ((mcu_x + (int)win_w) <= targetSpr->width() && (mcu_y + (int)win_h) <= targetSpr->height()) {
      targetSpr->pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    } else if ((mcu_y + (int)win_h) >= targetSpr->height()) {
      JpegDec.abort();
    }
  }
  targetSpr->setSwapBytes(false);
}

static void loadBackgroundJpeg(const uint8_t arrayname[], uint32_t array_size) {
  JpegDec.decodeArray(arrayname, array_size);
  int x = ((int)spr.width()  - (int)JpegDec.width)  / 2;
  int y = ((int)spr.height() - (int)JpegDec.height) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  renderJPEGToSprite(&spr, x, y);
}

// ---- Lightweight JSON Parsers ----
static bool jsonStr(const char *buf, const char *key, char *out, size_t n) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\":\"", key);
  const char *p = strstr(buf, pat);
  if (!p) return false;
  p += strlen(pat);
  const char *e = strchr(p, '"');
  if (!e) return false;
  size_t len = (size_t)(e - p);
  if (len >= n) len = n - 1;
  memcpy(out, p, len);
  out[len] = 0;
  return true;
}

static bool jsonNum(const char *buf, const char *key, long *out) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\":", key);
  const char *p = strstr(buf, pat);
  if (!p) return false;
  *out = atol(p + strlen(pat));
  return true;
}

// ---- Text Helper (English only, safe scaling) ----
static void centerText(const char *s, int y, uint16_t c, int font, int size = 1) {
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(c);
  spr.setTextSize(size);
  spr.drawString(s, CX, y, font);
  spr.setTextSize(1);
}

// Countdown time using Font 4 (scaled x2) centered on screen
static void bigTime(uint32_t ms, int y, uint16_t c) {
  uint32_t s = (ms + 999) / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  centerText(buf, y, c, 4, 2);
}

// ---- Render Screens ----
// 1. Idle (Power On / Standby) -> start.c ONLY
static void screenIdle(uint32_t now) {
  loadBackgroundJpeg(start_map, 9093);
}

// 2. Armed (Alarm Countdown) -> timer.c + Center mm:ss
static void screenArmed(uint32_t now) {
  loadBackgroundJpeg(timer_map, 12140);
  uint32_t el = now - armedMs;
  uint32_t left = (el >= armedTotal) ? 0 : armedTotal - el;
  int deg = armedTotal ? (int)(360.0f * left / armedTotal) : 0;
  
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, C_DIM, C_BG);
  if (deg > 0) spr.drawSmoothArc(CX, CY, 114, 106, 0, deg, C_ACCENT, C_BG, true);
  
  bigTime(left, CY, C_TXT);
}

// 3. Ringing (Alarm Firing) -> timer.c + Red Flash Border
static void screenRinging(uint32_t now) {
  loadBackgroundJpeg(timer_map, 12140);
  bool on = (now / 350) % 2;
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, on ? C_FAIL : C_DIM, C_BG);
}

// 4. Mission (Random/Assigned Image + Progress Ring)
static void screenMission(uint32_t now) {
  if (!strcmp(mIcon, "cup") || !strcmp(mLabel, "cup")) {
    loadBackgroundJpeg(cup_map, 18816);
  } else if (!strcmp(mIcon, "mouse") || !strcmp(mLabel, "mouse")) {
    loadBackgroundJpeg(mouse_map, 21970);
  } else if (!strcmp(mIcon, "phone") || !strcmp(mLabel, "phonecase") || !strcmp(mLabel, "phone")) {
    loadBackgroundJpeg(phone_map, 24128);
  } else if (!strcmp(mIcon, "shake") || !strcmp(mIcon, "swing") || !strcmp(mLabel, "shaking")) {
    loadBackgroundJpeg(shake_map, 34503);
  } else if (!strcmp(mIcon, "circle") || !strcmp(mIcon, "spin") || !strcmp(mLabel, "spin")) {
    loadBackgroundJpeg(circle_map, 32588);
  } else {
    loadBackgroundJpeg(start_map, 9093); // Fallback
  }

  // Draw unlock progress arc ring (ui 5 Hz)
  spr.drawSmoothArc(CX, CY, 116, 108, 0, 359, C_DIM, C_BG);
  uint32_t need = needMs ? needMs : 1;
  int deg = (int)(360.0f * (heldMs > need ? need : heldMs) / need);
  bool fresh = (now - lastUi) < 2000;              // Erase gauge if ui > 2 sec stale
  if (fresh && deg > 0)
    spr.drawSmoothArc(CX, CY, 116, 108, 0, deg, deg >= 359 ? C_OK : C_WARN, C_BG, true);
}

// 5. Fail -> retry.c (Held for 3s + Blue Blink Border)
static void screenFail(uint32_t now) {
  loadBackgroundJpeg(retry_map, 41217);
  bool on = (now / 350) % 2;
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, on ? C_ACCENT : C_DIM, C_BG);
}

// 6. Success -> goodjob.c (Held for 5s + Yellow Blink Border -> Shutdown)
static void screenSuccess(uint32_t now) {
  loadBackgroundJpeg(goodjob_map, 39457);
  bool on = (now / 350) % 2;
  spr.drawSmoothArc(CX, CY, 114, 106, 0, 359, on ? C_WARN : C_DIM, C_BG);
}

// 7. Shutdown
static void screenShutdown() {
  spr.fillSprite(C_BG);
}

static void render() {
  uint32_t now = millis();

  // Hold timer check (Retry 3s / Goodjob 5s)
  if (holdUntil && now >= holdUntil) {
    holdUntil = 0;
    st = pendingSt;
  }

  // Fill background margin with back.c color (0x7426)
  spr.fillSprite(C_BG);

  switch (st) {
    case S_ARMED:    screenArmed(now);    break;
    case S_RINGING:  screenRinging(now);  break;
    case S_MISSION:  screenMission(now);  break;
    case S_FAIL:     screenFail(now);     break;
    case S_SUCCESS:  screenSuccess(now);  break;
    case S_SHUTDOWN: screenShutdown();    break;
    default:         screenIdle(now);     break;
  }

  if (!mqtt.connected()) centerText("offline", CY + 100, C_FAIL, 2);

  spr.pushSprite(0, 0);
}

// ---- MQTT Callback & State Control ----
static void setState(St s) {
  if (holdUntil) { pendingSt = s; return; }
  st = s;
}

void onMqtt(char *topic, byte *payload, unsigned int len) {
  static char buf[512];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = 0;

  if (!strcmp(topic, UI_TOPIC)) {
    long v;
    if (jsonNum(buf, "held_ms", &v)) heldMs = (uint32_t)(v < 0 ? 0 : v);
    if (jsonNum(buf, "need_ms", &v) && v > 0) needMs = (uint32_t)v;
    lastUi = millis();
    return;
  }

  // cmd topic processing
  if (!strstr(buf, "\"type\":\"alarm\"")) return;
  char state[24] = "";
  if (!jsonStr(buf, "state", state, sizeof(state))) return;
  Serial.printf("[CMD] %s\n", buf);

  if (!strcmp(state, "armed")) {
    long r = 0;
    jsonNum(buf, "remain_s", &r);
    armedMs = millis();
    armedTotal = (uint32_t)(r > 0 ? r : 0) * 1000UL;
    holdUntil = 0;
    setState(S_ARMED);
  } else if (!strcmp(state, "ringing")) {
    holdUntil = 0;
    setState(S_RINGING);
  } else if (!strcmp(state, "mission")) {
    jsonStr(buf, "label", mLabel, sizeof(mLabel));
    jsonStr(buf, "icon",  mIcon,  sizeof(mIcon));
    jsonStr(buf, "kind",  mKind,  sizeof(mKind));
    long h;
    if (jsonNum(buf, "hold_s", &h) && h > 0) needMs = (uint32_t)h * 1000UL;
    heldMs = 0;
    setState(S_MISSION);
  } else if (!strcmp(state, "fail")) {
    // Fail: display retry.c for 3 seconds (3000ms), then return to mission retry
    st = S_FAIL;
    pendingSt = S_MISSION;
    holdUntil = millis() + FAIL_HOLD_MS;
    heldMs = 0;
  } else if (!strcmp(state, "success")) {
    // Success: display goodjob.c for 5 seconds (5000ms), then shutdown
    st = S_SUCCESS;
    pendingSt = S_SHUTDOWN;
    holdUntil = millis() + SUCCESS_HOLD_MS;
    heldMs = 0;
  } else if (!strcmp(state, "shutdown")) {
    holdUntil = 0;
    st = S_SHUTDOWN;
  } else if (!strcmp(state, "idle")) {
    holdUntil = 0;
    st = S_IDLE;
    heldMs = 0;
  }
}

// Enforce 3-second reconnect interval to prevent socket depletion (rc=-2)
static uint32_t lastMqttTry = 0;
void ensureMqtt() {
  if (mqtt.connected()) return;
  uint32_t now = millis();
  if (lastMqttTry && now - lastMqttTry < 3000) return;
  lastMqttTry = now;
  if (mqtt.connect(MQTT_ID)) {
    Serial.println("[MQTT] connected");
    mqtt.subscribe(CMD_TOPIC);
    mqtt.subscribe(UI_TOPIC);
  } else {
    Serial.printf("[MQTT] connect failed rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(200);
  Serial.println("\n==== alarmi CLOCK NODE ====");

#ifdef D6
  pinMode(D6, OUTPUT);
  digitalWrite(D6, HIGH);
#endif
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  pinMode(43, OUTPUT);
  digitalWrite(43, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  // 16bpp sprite allocation fallback to 8bpp (TFT_eSPI_Clock_ex2 pattern)
  if (spr.createSprite(SCR, SCR) == nullptr) {
    Serial.println("[TFT] 16bpp failed - fallback to 8bpp");
    spr.setColorDepth(8);
    spr.createSprite(SCR, SCR);
  }
  spr.fillSprite(C_BG);

  render();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(1024);
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    ensureMqtt();
    mqtt.loop();
  }

  // 5-second Serial Heartbeat
  if (now - lastHb >= 5000) {
    lastHb = now;
    Serial.printf("[HB] up=%lus wifi=%d mqtt=%d(state %d) st=%d held=%lu/%lu heap=%u\n",
                  now / 1000, WiFi.status(), mqtt.connected(), mqtt.state(),
                  (int)st, heldMs, needMs, ESP.getFreeHeap());
  }

  if (now - lastFrame >= FRAME_MS) { lastFrame = now; render(); }
}
