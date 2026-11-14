// アダプターとのつなぎ方(BLE / Wi-Fi / Bluetooth Classic)
// どれも ELM327 との文字のやりとりは同じなので、つなぐ・送る・受け取るだけをそろえている。
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <freertos/stream_buffer.h>

#if defined(CONFIG_IDF_TARGET_ESP32)  // 無印 ESP32(Core2)だけが Bluetooth Classic を使える
#define HAS_CLASSIC 1
#include <BluetoothSerial.h>
#else
#define HAS_CLASSIC 0
#endif

enum AdapterKind : uint8_t { KIND_NONE = 0, KIND_CLASSIC = 1, KIND_BLE = 2, KIND_WIFI = 3 };

struct Adapter {
  uint8_t kind = KIND_NONE;
  uint8_t mac[6] = {0};
  uint8_t addrType = 0;  // BLE のアドレスの種類
  char name[33] = "";    // 表示名(Wi-Fi はネットワーク名そのもの)
};

static const char *kindLabel(uint8_t kind) {
  switch (kind) {
    case KIND_BLE: return "BLE";
    case KIND_WIFI: return "Wi-Fi";
    case KIND_CLASSIC: return "BT";
    default: return "";
  }
}

class Link {
 public:
  virtual ~Link() {}
  virtual bool open(const Adapter &a) = 0;
  virtual void close() = 0;
  virtual bool isOpen() = 0;
  virtual int read() = 0;  // 来ていなければ -1
  virtual void write(const String &s) = 0;
  void flushInput() { while (read() >= 0) {} }
};

// ===== BLE =====
// 製品ごとに窓口(サービス・特性)の番号が違うので、よく使われるものから順に試し、
// どれでもなければ「通知できる特性」と「書ける特性」を持つサービスを探す。
class BleLink : public Link {
  BLEClient *client = nullptr;
  BLERemoteCharacteristic *rx = nullptr;  // アダプター → こちら(通知)
  BLERemoteCharacteristic *tx = nullptr;  // こちら → アダプター
  bool withResponse = true;
  static StreamBufferHandle_t buf;
  static volatile bool dropped;

  static void onNotify(BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
    xStreamBufferSend(buf, data, len, 0);
  }
  class Callbacks : public BLEClientCallbacks {
    void onConnect(BLEClient *) override {}
    void onDisconnect(BLEClient *) override { BleLink::dropped = true; }
  };

  static bool canListen(BLERemoteCharacteristic *c) { return c && (c->canNotify() || c->canIndicate()); }
  static bool canSend(BLERemoteCharacteristic *c) { return c && (c->canWrite() || c->canWriteNoResponse()); }

  bool tryKnown(BLEUUID svcId, BLEUUID rxId, BLEUUID txId) {
    BLERemoteService *svc = client->getService(svcId);
    if (!svc) return false;
    BLERemoteCharacteristic *r = svc->getCharacteristic(rxId);
    BLERemoteCharacteristic *t = svc->getCharacteristic(txId);
    if (!canListen(r) || !canSend(t)) return false;
    rx = r;
    tx = t;
    return true;
  }

  bool findPair() {
    rx = tx = nullptr;
    if (tryKnown(BLEUUID((uint16_t)0xFFF0), BLEUUID((uint16_t)0xFFF1), BLEUUID((uint16_t)0xFFF2))) return true;
    if (tryKnown(BLEUUID((uint16_t)0xFFE0), BLEUUID((uint16_t)0xFFE1), BLEUUID((uint16_t)0xFFE1))) return true;
    if (tryKnown(BLEUUID("e7810a71-73ae-499d-8c15-faa9aef0c3f2"), BLEUUID("bef8d6c9-9c21-4c9e-b632-bd58c1009f9f"),
                 BLEUUID("bef8d6c9-9c21-4c9e-b632-bd58c1009f9f")))
      return true;  // Vgate
    if (tryKnown(BLEUUID((uint16_t)0x18F0), BLEUUID((uint16_t)0x2AF0), BLEUUID((uint16_t)0x2AF1))) return true;
    auto services = client->getServices();
    if (!services) return false;
    for (auto &s : *services) {
      BLEUUID id = s.second->getUUID();
      if (id.bitSize() == 16) {
        uint16_t v = id.getNative()->uuid.uuid16;
        if (v >= 0x1800 && v <= 0x18FF && v != 0x18F0) continue;  // 名前・電池などの決まったサービスは飛ばす
      }
      BLERemoteCharacteristic *r = nullptr, *t = nullptr;
      auto chars = s.second->getCharacteristics();
      if (!chars) continue;
      for (auto &c : *chars) {
        if (!r && canListen(c.second)) r = c.second;
        if (!t && canSend(c.second)) t = c.second;
      }
      if (r && t) { rx = r; tx = t; return true; }
    }
    return false;
  }

 public:
  bool open(const Adapter &a) override {
    if (!buf) buf = xStreamBufferCreate(2048, 1);
    xStreamBufferReset(buf);
    dropped = false;
    if (!client) {
      client = BLEDevice::createClient();
      client->setClientCallbacks(new Callbacks());
    }
    uint8_t mac[6];
    memcpy(mac, a.mac, 6);
    if (!client->connect(BLEAddress(mac), (esp_ble_addr_type_t)a.addrType)) return false;
    if (!findPair()) {
      Serial.println("BLE: 送受信の窓口が見つかりません");
      client->disconnect();
      return false;
    }
    Serial.printf("BLE: 受信 %s / 送信 %s\n", rx->getUUID().toString().c_str(), tx->getUUID().toString().c_str());
    withResponse = tx->canWrite();
    rx->registerForNotify(onNotify, rx->canNotify());
    return true;
  }
  void close() override {
    if (client && client->isConnected()) client->disconnect();
    rx = tx = nullptr;
  }
  bool isOpen() override { return client && tx && !dropped && client->isConnected(); }
  int read() override {
    uint8_t c;
    return buf && xStreamBufferReceive(buf, &c, 1, 0) == 1 ? c : -1;
  }
  void write(const String &s) override {
    if (!tx) return;
    const uint8_t *p = (const uint8_t *)s.c_str();
    size_t n = s.length();
    while (n) {  // 1回に送れるのは 20 バイトまで
      size_t k = n < 20 ? n : 20;
      tx->writeValue((uint8_t *)p, k, withResponse);
      p += k;
      n -= k;
    }
  }
};
StreamBufferHandle_t BleLink::buf = nullptr;
volatile bool BleLink::dropped = false;

// ===== Wi-Fi =====
// Wi-Fi 版のアダプターは自分がネットワークになり、たいてい 192.168.0.10 の 35000 番で待っている。
class WifiLink : public Link {
  WiFiClient sock;

 public:
  bool open(const Adapter &a) override {
    WiFi.mode(WIFI_STA);
    WiFi.begin(a.name);  // パスワードなしのネットワークだけ(Wi-Fi 版アダプターはほぼこれ)
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi: ネットワークにつながりません");
      WiFi.disconnect();
      return false;
    }
    struct { IPAddress ip; uint16_t port; } tries[] = {
      {WiFi.gatewayIP(), 35000}, {IPAddress(192, 168, 0, 10), 35000}, {WiFi.gatewayIP(), 23}, {IPAddress(192, 168, 0, 10), 23},
    };
    for (auto &t : tries) {
      if (t.ip == IPAddress(0, 0, 0, 0)) continue;
      if (sock.connect(t.ip, t.port, 3000)) {
        sock.setNoDelay(true);
        Serial.printf("Wi-Fi: %s:%u につながりました\n", t.ip.toString().c_str(), t.port);
        return true;
      }
    }
    Serial.println("Wi-Fi: アダプターの入口が見つかりません");
    WiFi.disconnect();
    return false;
  }
  void close() override {
    sock.stop();
    WiFi.disconnect();
  }
  bool isOpen() override { return WiFi.status() == WL_CONNECTED && sock.connected(); }
  int read() override { return sock.available() ? sock.read() : -1; }
  void write(const String &s) override { sock.print(s); }
};

// ===== Bluetooth Classic(Core2 だけ)=====
#if HAS_CLASSIC
class ClassicLink : public Link {
  BluetoothSerial &bt;

 public:
  explicit ClassicLink(BluetoothSerial &serial) : bt(serial) {}
  bool open(const Adapter &a) override {
    uint8_t mac[6];
    memcpy(mac, a.mac, 6);
    return bt.connect(mac);
  }
  void close() override {
    if (bt.connected()) bt.disconnect();
  }
  bool isOpen() override { return bt.connected(); }
  int read() override { return bt.available() ? bt.read() : -1; }
  void write(const String &s) override { bt.print(s); }
};
#endif
