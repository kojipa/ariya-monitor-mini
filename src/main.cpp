// Ariya Monitor Mini v0.1.0(旧アリア電池モニター Core2 版 0.1〜0.2.2 の後継。公開にあわせて番号を振り直した)
// 日産アリアのバッテリー温度・残エネルギーなどを、小さな M5 の画面で見る。
//
// 対応: M5StickS3(BLE / Wi-Fi のアダプター)、M5Stack Core2(Bluetooth Classic / BLE / Wi-Fi)
// 画面1: バッテリー温度(最高・最低)・冷却水温度・外気温
// 画面2: 残エネルギー・SOC・電力・12V
//
// ボタン(StickS3・Core2 共通)
//   A 押す=画面切替 / A 長押し=上下反転
//   B 押す=明るさ   / B 長押し=アダプターを選び直す
//   選ぶ画面: B=次へ / A=決定
//   StickS3 は A=正面のボタン、B=横のボタン。本体に A・B の印がないので、画面の端に
//   ボタンのある方向の矢印と案内を出す(選ぶ画面はずっと、値の画面はボタンを押したあと数秒)。
//   (Core2 は C=上下反転、選ぶ画面はタッチでも選べる)
//
// 車に送るのは読み取り(0x22)と ELM327 の設定命令だけ。下の isAllowed() を通らない命令は送らない。
// 番号と式は Ariya Monitor で確定済みのもの(作者の車 B6 2WD 初期型で確認)。

#include <M5Unified.h>
#include <Preferences.h>
#include <math.h>
#include "links.h"
#if HAS_CLASSIC
#include <esp_gap_bt_api.h>
#endif

// ===== 設定 =====
static const char *VERSION = "0.1.0";
static const char *DEVICE_NAME = "AriyaMonitorMini";
static const char *ADAPTER_PIN = "1234";            // Classic のペアリング用
static const uint32_t OUTSIDE_INTERVAL_MS = 60000;  // 外気温は 11bit の機器なので 1 分に 1 回だけ切り替えて読む
static const uint32_t V12_INTERVAL_MS = 10000;
static const uint32_t STALE_MS = 15000;             // これより古い値は灰色
static const int MAX_FOUND = 8;                     // 一覧に並べる数

Preferences prefs;
M5Canvas canvas(&M5.Display);

BleLink bleLink;
WifiLink wifiLink;
#if HAS_CLASSIC
BluetoothSerial SerialBT;
ClassicLink classicLink(SerialBT);
#endif
static Link *conn = nullptr;

// ===== 値 =====
struct Reading {
  volatile float value = NAN;
  volatile uint32_t at = 0;
  void set(float v) { value = v; at = millis(); }
  bool fresh() const { return !isnan(value) && millis() - at < STALE_MS; }
};
static Reading tMax, tMin, water, soc, energy, packV, packA, v12, outside;

enum LinkState { LINK_CONNECTING, LINK_INIT, LINK_READING, LINK_NO_REPLY, LINK_FAILED, LINK_SCANNING, LINK_PICK, LINK_NO_ADAPTER };
static volatile LinkState linkState = LINK_CONNECTING;
static volatile uint32_t cycleMs = 0;

// ===== アダプター(本体に覚えさせる) =====
static Adapter adapter;
static volatile bool hasAdapter = false;
static Adapter found[MAX_FOUND];
static volatile int foundCount = 0;
static volatile bool scanRequested = false;
static volatile int picked = -1;  // 選んだ番号。-2 はやめる
static volatile const char *scanStep = "";

static const char *OBD_WORDS[] = {"OBD", "ELM", "V-LINK", "VLINK", "VGATE", "ICAR", "KONNWEI", "VEEPEAK", "CAN"};

// 名前か、ELM327 によく使われるアドレスの頭(00:1D:A5)で OBD アダプターらしいかを見る
static bool looksLikeObd(const Adapter &a) {
  if (a.kind == KIND_CLASSIC && a.mac[0] == 0x00 && a.mac[1] == 0x1D && a.mac[2] == 0xA5) return true;
  String n(a.name);
  n.toUpperCase();
  for (auto key : OBD_WORDS) if (n.indexOf(key) >= 0) return true;
  return false;
}

static void saveAdapter() {
  prefs.putBytes("adp", &adapter, sizeof adapter);
}
static bool loadAdapter() {
  Adapter a;
  if (prefs.getBytes("adp", &a, sizeof a) != sizeof a) return false;
  if (a.kind == KIND_NONE || a.kind > KIND_WIFI) return false;
#if !HAS_CLASSIC
  if (a.kind == KIND_CLASSIC) return false;
#endif
  adapter = a;
  return true;
}

// ===== 送ってよい命令 =====
static bool isHex(const String &s) {
  for (size_t i = 0; i < s.length(); i++) if (!isxdigit(s[i])) return false;
  return true;
}
static bool isAllowed(const String &cmd) {
  static const char *fixed[] = {"ATZ", "ATD", "ATE0", "ATL0", "ATS0", "ATH0", "ATCAF1", "ATSP6", "ATSP7", "ATI", "ATRV",
                                "ATSHDADBF1", "ATCRA18DAF1DB", "ATSHDADAF1", "ATCRA18DAF1DA", "ATSH744", "ATCRA764"};
  for (auto f : fixed) if (cmd == f) return true;
  return cmd.length() == 6 && cmd.startsWith("22") && isHex(cmd);  // 読み取り(ReadDataByIdentifier)だけ
}

// ===== ELM327 とのやりとり =====
static String elm(const String &cmd, uint32_t timeoutMs = 1500) {
  if (!isAllowed(cmd)) {
    Serial.printf("送らない命令: %s\n", cmd.c_str());
    return "";
  }
  if (!conn || !conn->isOpen()) return "";
  conn->flushInput();
  conn->write(cmd + "\r");
  String reply;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    int c = conn->read();
    if (c < 0) { delay(2); continue; }
    if (c == '>') break;
    reply += (c == '\r' || c == '\n') ? ' ' : (char)c;
  }
  reply.trim();
  Serial.printf("%s -> %s\n", cmd.c_str(), reply.c_str());
  return reply;
}

static bool configure(const String &cmd) {
  String r = elm(cmd, cmd == "ATZ" ? 3000 : 1500);
  return cmd == "ATZ" || cmd == "ATI" || r.indexOf("OK") >= 0;
}

enum Ecu { ECU_NONE, ECU_DB, ECU_DA, ECU_744 };
static Ecu currentEcu = ECU_NONE;
static int currentProtocol = 0;

static bool selectEcu(Ecu ecu) {
  if (ecu == currentEcu) return true;
  int protocol = ecu == ECU_744 ? 6 : 7;
  if (protocol != currentProtocol) {
    if (!configure(protocol == 6 ? "ATSP6" : "ATSP7")) return false;
    currentProtocol = protocol;
  }
  bool ok;
  switch (ecu) {
    case ECU_DB: ok = configure("ATSHDADBF1") && configure("ATCRA18DAF1DB"); break;
    case ECU_DA: ok = configure("ATSHDADAF1") && configure("ATCRA18DAF1DA"); break;
    default:     ok = configure("ATSH744") && configure("ATCRA764"); break;
  }
  currentEcu = ok ? ecu : ECU_NONE;
  return ok;
}

// 22xxxx を送り、62xxxx の後ろ bytes バイトを整数で返す
static bool readDid(const char *did, int bytes, uint32_t &raw) {
  String reply = elm(String("22") + did);
  String hex;
  for (size_t i = 0; i < reply.length(); i++) if (isxdigit(reply[i])) hex += (char)toupper(reply[i]);
  int at = hex.indexOf(String("62") + did);
  if (at < 0 || (int)hex.length() < at + 6 + bytes * 2) return false;
  raw = strtoul(hex.substring(at + 6, at + 6 + bytes * 2).c_str(), nullptr, 16);
  return true;
}

static float temp16(uint32_t raw) { return raw * 0.0625f - 40.0f; }

// ===== 読み取りの1周 =====
static bool pollCycle(uint32_t &lastV12, uint32_t &lastOutside) {
  uint32_t raw;
  int ok = 0;
  if (selectEcu(ECU_DB)) {
    if (readDid("9014", 2, raw)) { tMax.set(temp16(raw)); ok++; }
    if (readDid("9013", 2, raw)) { tMin.set(temp16(raw)); ok++; }
    if (readDid("91CD", 2, raw)) { water.set(temp16(raw)); ok++; }
    if (readDid("9002", 2, raw)) { soc.set(raw * 0.01f); ok++; }
    if (readDid("91C8", 3, raw)) { energy.set(raw * 0.001f); ok++; }
  }
  if (selectEcu(ECU_DA)) {
    if (readDid("20FE", 2, raw)) { packV.set(raw * 0.1f); ok++; }
    if (readDid("21CC", 2, raw)) { packA.set((int16_t)raw * 0.25f); ok++; }
  }
  if (millis() - lastV12 > V12_INTERVAL_MS) {
    lastV12 = millis();
    String r = elm("ATRV");
    float v = r.toFloat();
    if (v > 5 && v < 20) v12.set(v);
  }
  if (millis() - lastOutside > OUTSIDE_INTERVAL_MS) {
    lastOutside = millis();
    if (selectEcu(ECU_744) && readDid("4008", 1, raw)) outside.set((float)raw - 40.0f);
    selectEcu(ECU_DB);
  }
  return ok > 0;
}

static bool initAdapter() {
  currentEcu = ECU_NONE;
  currentProtocol = 0;
  for (auto cmd : {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATCAF1"}) if (!configure(cmd)) return false;
  return true;
}

// ===== アダプターを探す =====
static Adapter candidates[24];
static bool candidateLocked[24];  // パスワード付きの Wi-Fi(一覧には残し、鍵マークを付ける)
static int candidateCount = 0;
static bool foundLocked[MAX_FOUND];

static void addCandidate(const Adapter &a, bool locked = false) {
  for (int i = 0; i < candidateCount; i++) {
    Adapter &c = candidates[i];
    bool same = c.kind == a.kind &&
                (a.kind == KIND_WIFI ? strcmp(c.name, a.name) == 0 : memcmp(c.mac, a.mac, 6) == 0);
    if (same) {
      if (a.kind != KIND_WIFI && a.name[0] && (c.name[0] == 0 || c.name[0] == '(')) strlcpy(c.name, a.name, sizeof c.name);
      return;
    }
  }
  if (candidateCount < (int)(sizeof candidates / sizeof candidates[0])) {
    candidateLocked[candidateCount] = locked;
    candidates[candidateCount++] = a;
  }
}

static void scanBle() {
  scanStep = "BLE";
  BLEScan *scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  BLEScanResults results = scan->start(5, false);
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice d = results.getDevice(i);
    if (!d.haveName() || d.getName().empty()) continue;  // 名前のない機器(スマホの近くの何か等)は並べない
    Adapter a;
    a.kind = KIND_BLE;
    memcpy(a.mac, *d.getAddress().getNative(), 6);
    a.addrType = d.getAddressType();
    strlcpy(a.name, d.getName().c_str(), sizeof a.name);
    addCandidate(a);
  }
  scan->clearResults();
}

static void scanWifi() {
  scanStep = "Wi-Fi";
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) continue;
    Adapter a;
    a.kind = KIND_WIFI;
    strlcpy(a.name, ssid.c_str(), sizeof a.name);
    addCandidate(a, WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
}

#if HAS_CLASSIC
static void scanClassic() {
  scanStep = "Bluetooth";
  // 一度ペアリングすると周りから見えなくなるアダプターがあるので、ペアリング済みの機器も並べる
  int bonded = esp_bt_gap_get_bond_device_num();
  if (bonded > 0) {
    esp_bd_addr_t list[8];
    int count = 8;
    if (esp_bt_gap_get_bond_device_list(&count, list) == ESP_OK) {
      for (int i = 0; i < count; i++) {
        Adapter a;
        a.kind = KIND_CLASSIC;
        memcpy(a.mac, list[i], 6);
        strlcpy(a.name, "(ペアリング済み)", sizeof a.name);
        addCandidate(a);
      }
    }
  }
  BTScanResults *results = SerialBT.discover(8000);
  for (int i = 0; results && i < results->getCount(); i++) {
    BTAdvertisedDevice *d = results->getDevice(i);
    Adapter a;
    a.kind = KIND_CLASSIC;
    BTAddress address = d->getAddress();
    memcpy(a.mac, *address.getNative(), 6);
    strlcpy(a.name, d->haveName() ? d->getName().c_str() : "(名前なし)", sizeof a.name);
    addCandidate(a);
  }
}
#endif

// 周りの機器を探して一覧にし、画面で選ばれるのを待つ
static void scanAndPick() {
  scanRequested = false;
  if (conn) conn->close();
  conn = nullptr;
  linkState = LINK_SCANNING;
  foundCount = 0;
  candidateCount = 0;
  scanBle();
  scanWifi();
#if HAS_CLASSIC
  scanClassic();
#endif
  // OBD アダプターらしいものを先に並べる
  int n = 0;
  for (int pass = 0; pass < 2; pass++)
    for (int i = 0; i < candidateCount && n < MAX_FOUND; i++)
      if (looksLikeObd(candidates[i]) == (pass == 0)) {
        foundLocked[n] = candidateLocked[i];
        found[n++] = candidates[i];
      }
  Serial.printf("見つかった機器 %d 件(一覧に %d 件)\n", candidateCount, n);
  foundCount = n;
  picked = -1;
  linkState = LINK_PICK;
  while (picked == -1 && !scanRequested) delay(50);
  if (picked >= 0) {
    adapter = found[picked];
    hasAdapter = true;
    saveAdapter();
  }
}

static Link *linkFor(uint8_t kind) {
  switch (kind) {
    case KIND_BLE: return &bleLink;
    case KIND_WIFI: return &wifiLink;
#if HAS_CLASSIC
    case KIND_CLASSIC: return &classicLink;
#endif
    default: return nullptr;
  }
}

static void obdTask(void *) {
  for (;;) {
    if (scanRequested) { scanAndPick(); continue; }
    if (!hasAdapter) { linkState = LINK_NO_ADAPTER; delay(200); continue; }
    linkState = LINK_CONNECTING;
    conn = linkFor(adapter.kind);
    if (!conn || !conn->open(adapter)) {
      if (conn) conn->close();
      linkState = LINK_FAILED;
      for (int i = 0; i < 30 && !scanRequested; i++) delay(100);
      continue;
    }
    linkState = LINK_INIT;
    if (!initAdapter()) { conn->close(); delay(2000); continue; }
    uint32_t lastV12 = 0, lastOutside = 0;
    int silent = 0;
    while (conn->isOpen() && !scanRequested) {
      uint32_t start = millis();
      bool any = pollCycle(lastV12, lastOutside);
      cycleMs = millis() - start;
      silent = any ? 0 : silent + 1;
      linkState = silent >= 3 ? LINK_NO_REPLY : LINK_READING;
      if (silent >= 3) delay(2000);                     // 車が眠っているときは間をあける
      if (silent >= 10) { initAdapter(); silent = 3; }  // しばらく返事がなければアダプターを初期化し直す
      delay(50);
    }
    conn->close();
    delay(1000);
  }
}

// ===== 画面 =====
static int W = 320, H = 240;
static bool small = false;  // StickS3 などの小さい画面
static int page = 0;
static int brightness = 2;
static const uint8_t BRIGHT[] = {40, 90, 160, 255};
static int rotation = 1;
static int cursor = 0;  // 選ぶ画面で今いる行

// ボタンのある画面の端(StickS3 だけ矢印で案内する)
enum Edge { EDGE_LEFT, EDGE_RIGHT, EDGE_TOP, EDGE_BOTTOM };
static bool arrows = false;
static Edge aEdge = EDGE_LEFT, bEdge = EDGE_BOTTOM;
static const int STRIP = 14;  // 選ぶ画面で案内に使う端の幅
static uint32_t hintUntil = 0;  // 値の画面で案内を出す期限

// StickS3 を横にして正面のボタンが左のとき(向きの番号 3)、横のボタンは下の辺
static void updateEdges() {
  arrows = M5.getBoard() == m5::board_t::board_M5StickS3;
  if (rotation == 1) { aEdge = EDGE_RIGHT; bEdge = EDGE_TOP; }
  else { aEdge = EDGE_LEFT; bEdge = EDGE_BOTTOM; }
}
static int padL() { return arrows && aEdge == EDGE_LEFT ? STRIP : 0; }
static int padR() { return arrows && aEdge == EDGE_RIGHT ? STRIP : 0; }
static int padT() { return arrows && bEdge == EDGE_TOP ? STRIP : 0; }
static int padB() { return arrows && bEdge == EDGE_BOTTOM ? STRIP : 0; }

// 外向きの三角(ボタンのある方を指す)
static void drawArrow(int cx, int cy, Edge e, uint16_t color) {
  switch (e) {
    case EDGE_LEFT:   canvas.fillTriangle(cx - 5, cy, cx + 4, cy - 5, cx + 4, cy + 5, color); break;
    case EDGE_RIGHT:  canvas.fillTriangle(cx + 5, cy, cx - 4, cy - 5, cx - 4, cy + 5, color); break;
    case EDGE_TOP:    canvas.fillTriangle(cx, cy - 5, cx - 5, cy + 4, cx + 5, cy + 4, color); break;
    case EDGE_BOTTOM: canvas.fillTriangle(cx, cy + 5, cx - 5, cy - 4, cx + 5, cy - 4, color); break;
  }
}

// UTF-8 の1文字ぶんのバイト数
static int utf8Len(uint8_t c) { return c < 0x80 ? 1 : c < 0xE0 ? 2 : c < 0xF0 ? 3 : 4; }

// 選ぶ画面の端に、矢印と短い案内(左右の端は縦書き)
static void drawEdgeHint(Edge e, const char *label, uint16_t color) {
  canvas.setFont(&fonts::lgfxJapanGothicP_12);
  canvas.setTextColor(color);
  canvas.setTextDatum(middle_center);
  if (e == EDGE_LEFT || e == EDGE_RIGHT) {
    int cx = e == EDGE_LEFT ? STRIP / 2 : W - STRIP / 2;
    int chars = 0;
    for (const char *p = label; *p; p += utf8Len(*p)) chars++;
    int total = 12 + chars * 13;
    int y = (H - total) / 2;
    drawArrow(cx, y + 5, e, color);
    y += 12;
    for (const char *p = label; *p; p += utf8Len(*p), y += 13) {
      char one[5] = {0};
      memcpy(one, p, utf8Len(*p));
      canvas.drawString(one, cx, y + 6);
    }
  } else {
    int cy = e == EDGE_TOP ? STRIP / 2 : H - STRIP / 2;
    int w = canvas.textWidth(label);
    int x = (W - (w + 14)) / 2;
    drawArrow(x + 5, cy, e, color);
    canvas.setTextDatum(middle_left);
    canvas.drawString(label, x + 14, cy);
  }
}

// 値の画面の上に重ねる案内(ボタンのある端に寄せる)
static void drawHintBox(Edge e, const char *l1, const char *l2) {
  canvas.setFont(&fonts::lgfxJapanGothicP_12);
  int w = max(canvas.textWidth(l1), canvas.textWidth(l2)) + 24, h = 34;
  int x, y;
  switch (e) {
    case EDGE_LEFT:  x = 0; y = (H - h) / 2; break;
    case EDGE_RIGHT: x = W - w; y = (H - h) / 2; break;
    case EDGE_TOP:   x = (W - w) / 2; y = 0; break;
    default:         x = (W - w) / 2; y = H - h; break;
  }
  canvas.fillRoundRect(x, y, w, h, 4, TFT_BLACK);
  canvas.drawRoundRect(x, y, w, h, 4, TFT_YELLOW);
  drawArrow(x + 10, y + h / 2, e, TFT_YELLOW);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(TFT_YELLOW);
  canvas.drawString(l1, x + 19, y + 3);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString(l2, x + 19, y + 18);
}

// 鍵マーク(パスワード付き)
static void drawLock(int x, int cy, uint16_t color) {
  canvas.drawRoundRect(x + 1, cy - 6, 6, 8, 3, color);
  canvas.fillRect(x, cy - 2, 8, 6, color);
}

// 画面の向きの番号は機種で意味が違うので、幅と高さを見て必ず横長にする
static void setLandscape(int preferred) {
  for (int i = 0; i < 4; i++) {
    int r = (preferred + i) % 4;
    M5.Display.setRotation(r);
    if (M5.Display.width() > M5.Display.height()) { rotation = r; return; }
  }
}

static uint16_t tempColor(float t) {  // 旧 M5Leaf と同じ温度帯
  if (t > 45) return TFT_RED;
  if (t > 37) return TFT_ORANGE;
  if (t > 33) return TFT_YELLOW;
  if (t > 10) return TFT_GREEN;
  return TFT_CYAN;
}

static const lgfx::IFont *labelFont() { return small ? &fonts::lgfxJapanGothicP_12 : &fonts::lgfxJapanGothicP_16; }

// ラベルと値を描く。値が maxW に入らなければ縮める
static void drawValue(int x, int y, int maxW, const char *label, const Reading &r, const char *fmt, const char *unit,
                      uint16_t color, const lgfx::IFont *bigFont) {
  canvas.setTextDatum(top_left);
  canvas.setFont(labelFont());
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString(label, x, y);
  int top = y + (small ? 14 : 20);
  char buf[16];
  float v = r.value;
  if (isnan(v)) snprintf(buf, sizeof buf, "--");
  else if (v <= -10 && strstr(fmt, ".1")) snprintf(buf, sizeof buf, "%.0f", v);  // 狭いので −10 以下は小数を省く
  else snprintf(buf, sizeof buf, fmt, v);
  const lgfx::IFont *unitFont = small ? &fonts::lgfxJapanGothicP_12 : &fonts::lgfxJapanGothicP_20;
  canvas.setFont(unitFont);
  int unitW = unit[0] ? canvas.textWidth(unit) + 3 : 0;
  canvas.setFont(bigFont);
  int w = canvas.textWidth(buf);
  float scale = 1.0f;
  if (w + unitW > maxW) scale = (float)(maxW - unitW) / w;
  canvas.setTextSize(scale);
  canvas.setTextColor(r.fresh() ? color : TFT_DARKGREY);
  canvas.drawString(buf, x, top);
  int bigH = canvas.fontHeight();
  w = canvas.textWidth(buf);
  canvas.setTextSize(1);
  if (unit[0]) {
    canvas.setFont(unitFont);
    canvas.drawString(unit, x + w + 3, top + bigH - canvas.fontHeight() - 1);
  }
}

static const char *statusText(uint16_t &color) {
  switch (linkState) {
    case LINK_READING:    color = TFT_DARKGREEN; return "読み取り中";
    case LINK_INIT:       color = TFT_NAVY; return "初期化中";
    case LINK_NO_REPLY:   color = TFT_MAROON; return small ? "車の応答なし" : "車の応答なし(電源OFF?)";
    case LINK_FAILED:     color = TFT_MAROON; return small ? "つながりません" : "つながりません(B長押しで選び直し)";
    case LINK_SCANNING:   color = TFT_NAVY; return "アダプターを探しています";
    case LINK_PICK:       color = TFT_NAVY; return "アダプターを選んでください";
    case LINK_NO_ADAPTER: color = TFT_MAROON; return "アダプター未選択";
    default:              color = TFT_NAVY; return "接続中…";
  }
}

static int headerH() { return small ? 16 : 24; }

static void drawHeader(bool choosingNow) {
  uint16_t color;
  const char *text = statusText(color);
  int hh = headerH();
  int y0 = choosingNow ? padT() : 0;  // 選ぶ画面で横のボタンが上にあるときは、案内の下に出す
  canvas.fillRect(0, y0, W, hh, color);
  canvas.setFont(small ? &fonts::lgfxJapanGothicP_12 : &fonts::lgfxJapanGothicP_16);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(middle_left);
  canvas.drawString(text, 4, y0 + hh / 2);
  if (choosingNow) return;
  canvas.setTextDatum(middle_right);
  char buf[32];
  snprintf(buf, sizeof buf, "%s %d/2", kindLabel(adapter.kind), page + 1);
  canvas.drawString(buf, W - 4, hh / 2);
}

static void drawFooter(const char *a, const char *b, const char *c) {
  if (small) return;  // 小さい画面には案内を出す場所がない
  canvas.setFont(&fonts::lgfxJapanGothicP_12);
  canvas.setTextColor(TFT_DARKGREY);
  canvas.setTextDatum(bottom_center);
  canvas.drawString(a, 53, H - 1);
  canvas.drawString(b, 160, H - 1);
  canvas.drawString(c, 267, H - 1);
}

// 選ぶ画面の行: 見つかった機器 + 「もう一度探す」 + (選び済みなら)「やめる」
static int pickRows() { return foundCount + 1 + (hasAdapter ? 1 : 0); }
static int rowH() { return small ? (arrows ? 20 : 22) : 30; }
static int listTop() { return padT() + headerH() + (small ? 2 : 4); }
static int visibleRows() { return (H - listTop() - padB() - (small ? 0 : 16)) / rowH(); }

static void drawMessage(const char *l1, const char *l2, const char *l3 = nullptr) {
  canvas.setTextDatum(top_left);
  int y = padT() + headerH() + (small ? 12 : 40);
  canvas.setFont(small ? &fonts::lgfxJapanGothicP_16 : &fonts::lgfxJapanGothicP_20);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString(l1, 8 + padL(), y);
  canvas.setFont(small ? &fonts::lgfxJapanGothicP_12 : &fonts::lgfxJapanGothicP_16);
  canvas.setTextColor(TFT_LIGHTGREY);
  if (l2) canvas.drawString(l2, 8 + padL(), y + (small ? 30 : 44));
  if (l3) canvas.drawString(l3, 8 + padL(), y + (small ? 48 : 68));
}

static void drawPickPage() {
  if (linkState == LINK_SCANNING) {
    char buf[48];
    snprintf(buf, sizeof buf, "%s を探しています…", (const char *)scanStep);
    drawMessage(buf, "OBDII を車に挿して、", "車の電源を入れておいてください");
    return;
  }
  if (linkState == LINK_NO_ADAPTER) {
    drawMessage(!arrows ? "B を長押しすると" : bEdge == EDGE_BOTTOM ? "下のボタンを長押しすると" : "上のボタンを長押しすると",
                "OBDII を探して選べます");
    return;
  }
  int rows = pickRows(), vis = visibleRows(), rh = rowH();
  int first = cursor - vis + 1;
  if (first < 0) first = 0;
  for (int k = 0; k < vis && first + k < rows; k++) {
    int i = first + k;
    int y = listTop() + k * rh;
    bool sel = i == cursor;
    int x0 = padL() + 2, x1 = W - padR() - 2;
    if (sel) canvas.fillRoundRect(x0, y, x1 - x0, rh - 2, 4, TFT_NAVY);
    else canvas.drawRoundRect(x0, y, x1 - x0, rh - 2, 4, TFT_DARKGREY);
    canvas.setFont(small ? &fonts::lgfxJapanGothicP_12 : &fonts::lgfxJapanGothicP_16);
    canvas.setTextDatum(middle_left);
    int cy = y + (rh - 2) / 2;
    if (i < foundCount) {
      canvas.setTextColor(looksLikeObd(found[i]) ? TFT_GREEN : TFT_WHITE);
      canvas.drawString(found[i].name, x0 + 6, cy);
      canvas.setTextDatum(middle_right);
      canvas.setTextColor(TFT_LIGHTGREY);
      canvas.drawString(kindLabel(found[i].kind), x1 - 6, cy);
      if (foundLocked[i]) drawLock(x1 - 6 - canvas.textWidth(kindLabel(found[i].kind)) - 13, cy, TFT_LIGHTGREY);
    } else {
      canvas.setTextColor(TFT_YELLOW);
      canvas.drawString(i == foundCount ? (foundCount ? "もう一度探す" : "見つかりません。もう一度探す") : "やめる(前のアダプターのまま)", x0 + 6, cy);
    }
  }
}

static bool choosing() {
  return linkState == LINK_SCANNING || linkState == LINK_PICK || linkState == LINK_NO_ADAPTER;
}

static void drawTemperaturePage() {
  int top = headerH() + 3;
  int half = W / 2;
  if (small) {
    drawValue(4, top, half - 8, "電池 最高 ℃", tMax, "%.1f", "", tempColor(tMax.value), &fonts::Font7);
    drawValue(half + 2, top, half - 6, "電池 最低 ℃", tMin, "%.1f", "", tempColor(tMin.value), &fonts::Font7);
    canvas.drawFastHLine(0, 86, W, TFT_DARKGREY);
    drawValue(4, 89, half - 8, "冷却水", water, "%.1f", "℃", TFT_WHITE, &fonts::Font4);
    drawValue(half + 2, 89, half - 6, "外気温", outside, "%.0f", "℃", TFT_WHITE, &fonts::Font4);
  } else {
    drawValue(8, 34, 150, "バッテリー温度 最高", tMax, "%.1f", "℃", tempColor(tMax.value), &fonts::Font7);
    drawValue(168, 34, 150, "最低", tMin, "%.1f", "℃", tempColor(tMin.value), &fonts::Font7);
    canvas.drawFastHLine(0, 132, W, TFT_DARKGREY);
    drawValue(8, 140, 150, "冷却水温度", water, "%.1f", "℃", TFT_WHITE, &fonts::Font4);
    drawValue(168, 140, 150, "外気温", outside, "%.0f", "℃", TFT_WHITE, &fonts::Font4);
  }
}

static void drawEnergyPage() {
  Reading power;
  if (packV.fresh() && packA.fresh()) power.set(packV.value * packA.value / 1000.0f);
  uint16_t pc = isnan(power.value) ? TFT_WHITE : (power.value >= 0 ? TFT_CYAN : TFT_MAGENTA);  // +充電・回生 / −走行
  if (small) {
    int top = headerH() + 3;
    drawValue(4, top, 150, "残エネルギー kWh", energy, "%.1f", "", TFT_WHITE, &fonts::Font7);
    drawValue(162, top, 74, "SOC", soc, "%.0f", "%", TFT_WHITE, &fonts::Font4);
    canvas.drawFastHLine(0, 86, W, TFT_DARKGREY);
    drawValue(4, 89, 150, "電力 +充電 -走行", power, "%+.1f", "kW", pc, &fonts::Font4);
    drawValue(162, 89, 74, "12V", v12, "%.1f", "V", TFT_WHITE, &fonts::Font4);
  } else {
    drawValue(8, 34, 190, "残エネルギー", energy, "%.1f", "kWh", TFT_WHITE, &fonts::Font7);
    drawValue(210, 34, 105, "SOC", soc, "%.0f", "%", TFT_WHITE, &fonts::Font4);
    canvas.drawFastHLine(0, 132, W, TFT_DARKGREY);
    drawValue(8, 140, 150, "電力(+充電 -走行)", power, "%+.1f", "kW", pc, &fonts::Font4);
    drawValue(168, 140, 75, "12V", v12, "%.1f", "V", TFT_WHITE, &fonts::Font4);
    drawValue(250, 140, 66, "外気", outside, "%.0f", "℃", TFT_WHITE, &fonts::Font4);
  }
}

static void redraw() {
  canvas.fillScreen(TFT_BLACK);
  bool ch = choosing();
  drawHeader(ch);
  if (ch) {
    drawPickPage();
    if (linkState == LINK_PICK) drawFooter("決定", "次へ", "");
    else drawFooter("", "長押しで探す", "");
    if (arrows && linkState == LINK_PICK) {
      drawEdgeHint(aEdge, "決定", TFT_YELLOW);
      drawEdgeHint(bEdge, "次へ", TFT_YELLOW);
    } else if (arrows && linkState == LINK_NO_ADAPTER) {
      drawEdgeHint(bEdge, "長押しで探す", TFT_YELLOW);
    }
  } else {
    if (page == 0) drawTemperaturePage(); else drawEnergyPage();
    drawFooter("画面", "明るさ/長押しで選ぶ", "反転");
    if (arrows && millis() < hintUntil) {
      drawHintBox(aEdge, "画面切替", "長押し:上下反転");
      drawHintBox(bEdge, "明るさ", "長押し:アダプター選び直し");
    }
  }
  canvas.pushSprite(0, 0);
}

static void beep(int hz, int ms) { M5.Speaker.tone(hz, ms); }

static void setupCanvas() {
  W = M5.Display.width();
  H = M5.Display.height();
  small = H < 200;
  canvas.deleteSprite();
  canvas.setColorDepth(16);
  canvas.createSprite(W, H);
  updateEdges();
}

static void flip() {
  setLandscape((rotation + 2) % 4);
  prefs.putInt("rot", rotation);
}

static void choose(int row) {
  if (row < foundCount) picked = row;
  else if (row == foundCount) scanRequested = true;
  else picked = -2;
  beep(2000, 30);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  prefs.begin("ariyamini", false);
  // 前回選んだ向きを使う。初めては、最初に見つかる横長の向きの反対から(Core2 で上下逆だったため)
  int saved = prefs.getInt("rot", -1);
  if (saved >= 0) setLandscape(saved);
  else { setLandscape(1); setLandscape((rotation + 2) % 4); }
  setupCanvas();
  // 前回選んだアダプター。初めては探す画面から始める
  hasAdapter = loadAdapter();
  scanRequested = !hasAdapter;
  M5.Display.setBrightness(BRIGHT[brightness]);
  canvas.fillScreen(TFT_BLACK);
  canvas.setFont(small ? &fonts::lgfxJapanGothicP_16 : &fonts::lgfxJapanGothicP_20);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString("Ariya Monitor Mini", W / 2, H / 2 - 10);
  canvas.setFont(&fonts::lgfxJapanGothicP_12);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString(String("v") + VERSION, W / 2, H / 2 + 14);
  canvas.pushSprite(0, 0);

#if HAS_CLASSIC
  SerialBT.begin(DEVICE_NAME, true);
  SerialBT.setPin(ADAPTER_PIN);
#endif
  BLEDevice::init(DEVICE_NAME);
  xTaskCreatePinnedToCore(obdTask, "obd", 8192, nullptr, 1, nullptr, 1);
}

void loop() {
  M5.update();
  if (linkState == LINK_PICK) {
    int rows = pickRows();
    if (cursor >= rows) cursor = 0;
    if (M5.BtnB.wasClicked()) { cursor = (cursor + 1) % rows; beep(1500, 15); }
    if (M5.BtnC.wasClicked()) { cursor = (cursor + rows - 1) % rows; beep(1500, 15); }
    if (M5.BtnA.wasClicked()) choose(cursor);
    if (M5.BtnB.wasHold()) { scanRequested = true; beep(1500, 80); }
    if (M5.Touch.isEnabled()) {
      auto t = M5.Touch.getDetail();
      if (t.wasPressed() && t.y >= listTop()) {
        int first = cursor - visibleRows() + 1;
        if (first < 0) first = 0;
        int i = first + (t.y - listTop()) / rowH();
        if (i < rows) { cursor = i; choose(i); }
      }
    }
  } else if (!choosing()) {
    // 値の画面に来たときと、ボタンを押したときは、しばらく操作の案内を出す
    static bool wasChoosing = true;
    if (wasChoosing) hintUntil = millis() + 5000;
    wasChoosing = false;
    if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) hintUntil = millis() + 4000;
    if (M5.BtnA.wasClicked()) { page = 1 - page; beep(2000, 30); }
    if (M5.BtnA.wasHold() || M5.BtnC.wasClicked()) { flip(); setupCanvas(); }
    if (M5.BtnB.wasClicked()) { brightness = (brightness + 1) % 4; M5.Display.setBrightness(BRIGHT[brightness]); }
    if (M5.BtnB.wasHold()) { scanRequested = true; cursor = 0; beep(1500, 80); wasChoosing = true; }
  } else if (linkState == LINK_NO_ADAPTER) {
    if (M5.BtnB.wasHold()) { scanRequested = true; cursor = 0; beep(1500, 80); }
  }
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > 250) { lastDraw = millis(); redraw(); }
  delay(10);
}
