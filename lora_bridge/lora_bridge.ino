/*
 * ============================================================================
 *  TTGO LoRa32 (SX1278 433 MHz) — przezroczysty most UART <-> LoRa <-> UART
 *  Przeznaczenie: przedłużenie łącza KISS TNC (np. VP-DIGI) przez LoRa.
 * ============================================================================
 *  Wymagane biblioteki:
 *    - RadioLib 7.x (sprawdzone względem 7.8.1). Przyszła wersja 8.x zmienia
 *      sygnaturę begin() — wtedy zostań przy 7.x w Menedżerze bibliotek.
 *    - Adafruit SSD1306 + Adafruit GFX
 *    - Rdzeń ESP32 Arduino 2.x lub 3.x (płytka: "TTGO LoRa32-OLED" / "ESP32 Dev Module")
 *
 *  Protokół radiowy (nagłówek 5 B):
 *    [MAGIC 0xA5][SENDER][TARGET][SEQ][FLAGS] + do 240 B danych
 *    FLAGS: bit0 = pierwszy fragment wiadomości, bit1 = ostatni fragment.
 *  Wiadomość z UART (ramka KISS, linia tekstu lub porcja po timeoucie) jest
 *  dzielona na fragmenty. Odbiorca składa je i wypycha na UART TYLKO całą
 *  wiadomość. Jeśli zginie którykolwiek fragment (luka w SEQ), cała wiadomość
 *  jest odrzucana — dzięki temu TNC nigdy nie dostanie uciętej ramki KISS.
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// PINOUT PŁYTKI TTGO LoRa32 (V1 / V2 / V2.1 - 433 MHz SX1278)
// ============================================================================
// Rewizja płytki jest rozpoznawana AUTOMATYCZNIE przy starcie (detectBoard()),
// więc w Arduino IDE można wybrać dowolną: "TTGO LoRa32-OLED" (każda Revision)
// albo "ESP32 Dev Module". Jeden plik .bin działa na wszystkich wersjach.
//
//   Rewizja          | Układ        | OLED SDA/SCL | OLED RST | LoRa RST | UART TX
//   V1               | ESP32-D0WDQ6 | 4 / 15       | 16       | 14       | 23
//   V2               | ESP32-D0WDQ6 | 21 / 22      | 16       | 12       | 15
//   V2.1 (T3 1.6.x)  | ESP32-PICO-D4| 21 / 22      | brak     | 23       | 15
//
// W ESP32-PICO-D4 GPIO16/17 są połączone z wewnętrzną pamięcią flash —
// sterowanie nimi zawiesza procesor (INT_WDT). Dlatego GPIO16 jest używany
// tylko na układach innych niż PICO.
#define P_LORA_SCK    5
#define P_LORA_MISO   19
#define P_LORA_MOSI   27
#define P_LORA_CS     18
#define P_LORA_DIO0   26
#define UART_RX_PIN   13   // <- TX VP-DIGI (wszystkie rewizje)
// UWAGA: w V2/V2.1 GPIO13/15 są też podpięte do slotu karty SD — nie wkładaj
// karty. Poziomy 3,3 V, wspólna masa z VP-DIGI!

struct BoardPins {
  const char* name;
  int8_t oledSda, oledScl, oledRst;   // oledRst = -1: brak
  int8_t loraRst;
  int8_t uartTx;
};
BoardPins bp = {"TTGO V2.1", 21, 22, -1, 23, 15};

#define BOARD_NAME   (bp.name)
#define P_LORA_RST   (bp.loraRst)
#define P_OLED_SDA   (bp.oledSda)
#define P_OLED_SCL   (bp.oledScl)
#define UART_TX_PIN  (bp.uartTx)

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// Historia terminala WWW (bufor kołowy w RAM). Typ musi być zdefiniowany PRZED
// pierwszą funkcją w pliku — Arduino IDE wstawia tam automatyczne prototypy.
#define LOG_ENTRIES   100     // ile ostatnich wiadomości/zdarzeń pamięta ESP32
#define LOG_DATA_MAX  400     // bajtów danych na wpis (ramka KISS z VP-Digi: maks. ~340 B)
struct LogEntry {
  uint32_t id;
  char     time[16];
  char     dir;               // 'T' = UART→LoRa, 'R' = LoRa→UART, 'E' = zdarzenie
  int16_t  rssi10, snr10;     // ×10, tylko dla 'R'
  uint16_t len;               // pełna długość wiadomości
  uint16_t stored;            // ile bajtów zapisano (≤ LOG_DATA_MAX)
  uint8_t  data[LOG_DATA_MAX];
};

#define TX_QUEUE_LEN      8
#define UART_RX_BUF       4096
#define BUSY_GIVEUP_MS    8000    // po tylu ms zajętego kanału nadajemy mimo wszystko

// ============================================================================
// KONFIGURACJA (NVS)
// ============================================================================
struct Config {
  uint8_t  nodeId;
  uint8_t  targetId;
  uint32_t uartBaud;
  uint16_t uartTimeout;
  bool     useUsbBridge;
  float    loraFreq;
  float    loraBw;
  uint8_t  loraSf;
  uint8_t  loraCr;
  uint8_t  loraPower;
  uint8_t  loraSync;
  char     wifiSsid[33];
  char     wifiPass[65];
};
Config cfg;
Config radioCfgGood;   // ostatnia konfiguracja radia, która zadziałała

// ============================================================================
// STATYSTYKI
// ============================================================================
struct Stats {
  uint32_t txPackets = 0, rxPackets = 0;   // ramki radiowe
  uint32_t txMsgs = 0,    rxMsgs = 0;      // całe wiadomości
  uint32_t txBytes = 0,   rxBytes = 0;     // bajty danych użytkownika
  uint32_t crcErrors = 0, cadWaits = 0, lostPackets = 0, droppedMsgs = 0;
  uint32_t txErrors = 0,  foreign = 0, notOurs = 0, airPackets = 0;
  uint32_t kissTx = 0,    kissRx = 0;      // poprawne ramki KISS/AX.25
  uint8_t  foreignFrom = 0, foreignTo = 0; // ostatnia ramka mostu z niepasującymi ID
  int16_t  radioState = 0;
  bool     radioOk = false;
  uint32_t toaMs = 0;
  float    lastRssi = 0.0f, lastSnr = 0.0f;
  char     lastRxTime[16] = "--:--:--";
  char     lastLogHex[96] = "Brak danych";
  char     lastLogTxt[80] = "Brak danych";
  char     lastAprsRx[120] = "Brak";
  char     lastAprsTx[120] = "Brak";
};
Stats stats;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// ==== BRIDGE_CORE_BEGIN ====
// Czysta logika (bez Arduino/RadioLib): ramkowanie KISS z UART, fragmentacja
// i składanie wiadomości radiowych, dekoder AX.25 do podglądu. Ten blok jest
// kompilowany i testowany także na PC (test z prawdziwymi ramkami APRS).
// ============================================================================
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define MAGIC_BYTE   0xA5
#define HEADER_SIZE  5       // MAGIC, SENDER, TARGET, SEQ, FLAGS
#define FRAG_MAX     240     // dane w jednej ramce LoRa (5 + 240 <= 255)
#define FLAG_FIRST   0x01
#define FLAG_LAST    0x02
#define MAX_MSG      1024    // maks. wiadomość (ramka KISS z APRS: zwykle < 400 B)
#define KISS_FEND    0xC0
#define KISS_FESC    0xDB
#define KISS_TFEND   0xDC
#define KISS_TFESC   0xDD

struct Message {
  uint16_t len;
  uint8_t  data[MAX_MSG];
};

// --- Ramkowanie strumienia z UART ------------------------------------------
// Ramka KISS (C0 ... C0) jest przekazywana w całości, bajt w bajt (z FEND-ami
// i sekwencjami FESC). Tekst (tryb monitor TNC) dzielony jest po '\n', a inne
// dane po ciszy na linii. Kolejne FEND-y (C0 C0) między ramkami są pomijane.
struct KissFramer {
  Message  msg;
  bool     inKiss = false;
  uint32_t lastByte = 0;
  uint32_t kissFrames = 0, otherMsgs = 0, overflows = 0;

  void reset() { msg.len = 0; inKiss = false; }

  template <typename Emit>
  void flush(Emit emit, bool isKiss) {
    if (msg.len == 0) return;
    if (isKiss) kissFrames++; else otherMsgs++;
    emit(msg);
    msg.len = 0;
  }

  template <typename Emit>
  void feed(uint8_t b, uint32_t now, Emit emit) {
    lastByte = now;
    if (b == KISS_FEND) {
      if (inKiss) {
        if (msg.len > 1) {                       // koniec ramki
          msg.data[msg.len++] = b;
          flush(emit, true);
          inKiss = false;
        }                                        // else: C0 C0 — pomiń
      } else {
        flush(emit, false);                      // tekst przed ramką
        msg.data[0] = b;
        msg.len = 1;
        inKiss = true;
      }
      return;
    }
    msg.data[msg.len++] = b;
    if (!inKiss && b == '\n') {
      flush(emit, false);
    } else if (msg.len >= MAX_MSG - 1) {         // zostaw miejsce na końcowy FEND
      overflows++;
      flush(emit, false);
      inKiss = false;
    }
  }

  // Wywoływać cyklicznie. baseMs = timeout ciszy z konfiguracji, baud = prędkość UART.
  template <typename Emit>
  void poll(uint32_t now, uint32_t baseMs, uint32_t baud, Emit emit) {
    if (msg.len == 0) return;
    uint32_t t = baseMs;
    uint32_t minT = 30000UL / (baud ? baud : 9600) + 2;   // ~3 znaki
    if (t < minT) t = minT;
    if (inKiss) { t *= 10; if (t < 500) t = 500; }      // nie tnij ramki KISS w środku
    if (now - lastByte >= t) {
      if (inKiss && msg.len == 1) msg.len = 0;          // samotny FEND
      else flush(emit, false);                          // niedokończona ramka / porcja danych
      inKiss = false;
    }
  }
};

// --- Fragment radiowy ---------------------------------------------------------
// Zwraca długość ramki LoRa zapisanej do out (HEADER_SIZE + dane) i przesuwa offset.
inline size_t buildFragment(const Message& m, uint16_t& offset, uint8_t from, uint8_t to,
                            uint8_t seq, uint8_t* out) {
  uint16_t remain = m.len - offset;
  uint16_t n = remain > FRAG_MAX ? FRAG_MAX : remain;
  uint8_t flags = 0;
  if (offset == 0) flags |= FLAG_FIRST;
  if (offset + n >= m.len) flags |= FLAG_LAST;
  out[0] = MAGIC_BYTE; out[1] = from; out[2] = to; out[3] = seq; out[4] = flags;
  memcpy(&out[HEADER_SIZE], &m.data[offset], n);
  offset += n;
  return HEADER_SIZE + n;
}

// --- Składanie wiadomości z fragmentów ------------------------------------------
// Wiadomość trafia na UART tylko w całości; luka w numeracji = odrzucenie całej.
struct Reassembler {
  uint8_t  buf[MAX_MSG];
  uint16_t len = 0;
  bool     active = false, haveLastSeq = false;
  uint8_t  lastSeq = 0;
  uint32_t packets = 0, lost = 0, dropped = 0, foreign = 0, notOurs = 0;
  uint8_t  foreignFrom = 0, foreignTo = 0;   // ostatnia ramka mostu z niepasującymi ID

  void reset() { active = false; haveLastSeq = false; len = 0; }

  // Zwraca true, gdy pakiet był ramką mostu przeznaczoną dla nas.
  template <typename Deliver>
  bool feed(const uint8_t* p, size_t n, uint8_t myId, uint8_t peerId, Deliver deliver) {
    if (n <= HEADER_SIZE || p[0] != MAGIC_BYTE) { notOurs++; return false; }   // np. LoRa APRS
    if (p[1] != peerId || p[2] != myId) {
      foreign++; foreignFrom = p[1]; foreignTo = p[2];
      return false;
    }
    uint8_t seq = p[3], flags = p[4];
    const uint8_t* payload = p + HEADER_SIZE;
    size_t plen = n - HEADER_SIZE;
    packets++;

    bool contiguous = haveLastSeq && seq == (uint8_t)(lastSeq + 1);
    if (haveLastSeq && !contiguous) lost += (uint8_t)(seq - lastSeq - 1);
    haveLastSeq = true;
    lastSeq = seq;

    if (flags & FLAG_FIRST) {
      if (active) dropped++;                    // poprzednia wiadomość urwana
      len = 0;
      active = true;
    } else if (!active || !contiguous) {
      if (!contiguous) dropped++;               // zgubiony fragment → odrzuć całość
      active = false;
      return true;
    }
    if (len + plen > MAX_MSG) { dropped++; active = false; return true; }
    memcpy(&buf[len], payload, plen);
    len += plen;
    if (flags & FLAG_LAST) {
      deliver(buf, len);
      active = false;
    }
    return true;
  }
};

// --- Dekoder KISS/AX.25 → tekst TNC2 (tylko do podglądu) ----------------------
// Przykład wyniku: "SQ3ABC-9>APRS,WIDE1-1*:!5212.34N/01700.00E>Test"
// Zwraca: 1 = ramka danych AX.25, 0 = nie-KISS/nieprawidłowa, 2 = komenda KISS.
inline int kissToTnc2(const uint8_t* k, size_t n, char* out, size_t outLen) {
  uint8_t f[MAX_MSG];          // na stosie: dekoder jest wołany z wątku radia i WWW
  size_t i = 0, fl = 0;
  if (outLen == 0) return 0;
  out[0] = 0;
  while (i < n && k[i] == KISS_FEND) i++;
  if (i == 0) { snprintf(out, outLen, "(brak FEND na poczatku - to nie KISS)"); return 0; }
  bool closed = false;
  for (; i < n; i++) {
    uint8_t b = k[i];
    if (b == KISS_FEND) { closed = true; break; }
    if (b == KISS_FESC && i + 1 < n) {
      uint8_t c = k[++i];
      b = (c == KISS_TFEND) ? KISS_FEND : (c == KISS_TFESC) ? KISS_FESC : c;
    }
    if (fl < sizeof(f)) f[fl++] = b;
  }
  if (!closed) { snprintf(out, outLen, "(niedomknieta ramka KISS, %u B)", (unsigned)fl); return 0; }
  if (fl < 1) { snprintf(out, outLen, "(pusta ramka KISS)"); return 0; }
  uint8_t cmd = f[0];
  if ((cmd & 0x0F) != 0) {
    snprintf(out, outLen, "KISS komenda 0x%02X (port %u), %u B", cmd & 0x0F, cmd >> 4, (unsigned)fl - 1);
    return 2;
  }
  const uint8_t* ax = f + 1;
  size_t al = fl - 1;
  if (al < 16) { snprintf(out, outLen, "(za krotka ramka AX.25: %u B)", (unsigned)al); return 0; }

  char calls[10][12];
  bool hbit[10];
  size_t na = 0, p = 0;
  bool last = false;
  while (!last && na < 10 && p + 7 <= al) {
    char c[7];
    size_t cl = 0;
    for (int j = 0; j < 6; j++) {
      char ch = (char)(ax[p + j] >> 1);
      if (ax[p + j] & 1) { snprintf(out, outLen, "(blad adresu AX.25)"); return 0; }
      if (ch == ' ') continue;
      if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9'))) {
        snprintf(out, outLen, "(niepoprawny znak w adresie AX.25)"); return 0;
      }
      c[cl++] = ch;
    }
    c[cl] = 0;
    uint8_t s = ax[p + 6];
    uint8_t ssid = (s >> 1) & 0x0F;
    if (ssid) snprintf(calls[na], sizeof(calls[na]), "%s-%u", c, ssid);
    else      snprintf(calls[na], sizeof(calls[na]), "%s", c);
    hbit[na] = (s & 0x80) != 0;
    last = (s & 0x01) != 0;
    na++;
    p += 7;
  }
  if (!last || na < 2) { snprintf(out, outLen, "(brak konca pola adresowego AX.25)"); return 0; }
  if (p + 2 > al) { snprintf(out, outLen, "(brak pol control/PID)"); return 0; }

  size_t o = (size_t)snprintf(out, outLen, "%s>%s", calls[1], calls[0]);
  for (size_t a = 2; a < na && o < outLen; a++)
    o += (size_t)snprintf(out + o, outLen - o, ",%s%s", calls[a], hbit[a] ? "*" : "");
  uint8_t ctrl = ax[p], pid = ax[p + 1];
  if (ctrl != 0x03 || pid != 0xF0) {
    if (o < outLen) snprintf(out + o, outLen - o, " [ctrl 0x%02X pid 0x%02X]", ctrl, pid);
    return 1;
  }
  if (o < outLen) out[o++] = ':';
  for (size_t q = p + 2; q < al && o + 1 < outLen; q++) {
    uint8_t ch = ax[q];
    out[o++] = (ch >= 32 && ch < 127) ? (char)ch : '.';
  }
  out[o < outLen ? o : outLen - 1] = 0;
  return 1;
}
// ==== BRIDGE_CORE_END ====

#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"
#define NTP_SERVER_1 "tempus1.gum.gov.pl"
#define NTP_SERVER_2 "pool.ntp.org"


KissFramer  framer;
Reassembler reasm;

// ============================================================================
// OBIEKTY GLOBALNE
// ============================================================================
Module*  radioModule = nullptr;
SX1278*  radio = nullptr;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  // reset OLED robimy sami (detectBoard)
WebServer server(80);
Preferences prefs;
QueueHandle_t txQueue;

volatile bool dio0Flag = false;            // RxDone / TxDone / CadDone
volatile bool reconfigureRadioFlag = false;
volatile bool txActive = false;
bool oledAvailable = false;
bool ntpSynced = false;

void IRAM_ATTR onDio0() { dio0Flag = true; }

HardwareSerial& bridgePort() { return cfg.useUsbBridge ? Serial : Serial2; }

// ============================================================================
// POMOCNICZE
// ============================================================================
void formatTime(char* out, size_t n) {
  struct tm ti;
  if (getLocalTime(&ti, 0)) {
    ntpSynced = true;
    strftime(out, n, "%H:%M:%S", &ti);
  } else {
    unsigned long s = millis() / 1000;
    snprintf(out, n, "UP %02lu:%02lu:%02lu", (s / 3600) % 100, (s / 60) % 60, s % 60);
  }
}

void setPreview(const char* prefix, const uint8_t* buf, size_t len) {
  char hex[sizeof(stats.lastLogHex)];
  char txt[sizeof(stats.lastLogTxt)];

  size_t p = snprintf(hex, sizeof(hex), "%s [%u B] ", prefix, (unsigned)len);
  size_t n = len < 16 ? len : 16;
  for (size_t i = 0; i < n && p + 4 < sizeof(hex); i++)
    p += snprintf(hex + p, sizeof(hex) - p, "%02X ", buf[i]);
  if (len > n && p + 4 < sizeof(hex)) strcpy(hex + p, "...");

  size_t q = snprintf(txt, sizeof(txt), "%s ", prefix);
  for (size_t i = 0; i < len && q + 5 < sizeof(txt); i++) {
    uint8_t c = buf[i];
    if (c == KISS_FEND) { memcpy(txt + q, "[C0]", 4); q += 4; }
    else txt[q++] = (c >= 32 && c <= 126) ? (char)c : '.';
  }
  txt[q] = 0;

  portENTER_CRITICAL(&statsMux);
  memcpy(stats.lastLogHex, hex, sizeof(hex));
  memcpy(stats.lastLogTxt, txt, sizeof(txt));
  portEXIT_CRITICAL(&statsMux);
}

String jsonEscape(const char* s) {
  String o;
  o.reserve(strlen(s) + 8);
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", (uint8_t)c); o += b; }
    else o += c;
  }
  return o;
}

// ============================================================================
// HISTORIA DLA TERMINALA WWW
// ============================================================================
static LogEntry   logBuf[LOG_ENTRIES];
static uint32_t   logNextId = 1;
static uint32_t   logBootId = 0;       // zmienia się po restarcie — przeglądarka czyści widok
SemaphoreHandle_t logMutex = nullptr;

void formatTimeMs(char* out, size_t n) {
  struct timeval tv;
  struct tm ti;
  if (ntpSynced && gettimeofday(&tv, nullptr) == 0 && localtime_r(&tv.tv_sec, &ti)) {
    size_t k = strftime(out, n, "%H:%M:%S", &ti);
    snprintf(out + k, n - k, ".%03ld", (long)(tv.tv_usec / 1000));
  } else {
    unsigned long ms = millis();
    unsigned long s = ms / 1000;
    snprintf(out, n, "+%02lu:%02lu:%02lu.%03lu", (s / 3600) % 100, (s / 60) % 60, s % 60, ms % 1000);
  }
}

void logAdd(char dir, const uint8_t* d, size_t n, float rssi = 0, float snr = 0) {
  if (!logMutex) return;
  char t[16];
  formatTimeMs(t, sizeof(t));
  xSemaphoreTake(logMutex, portMAX_DELAY);
  LogEntry& e = logBuf[logNextId % LOG_ENTRIES];
  e.id = logNextId++;
  memcpy(e.time, t, sizeof(t));
  e.dir = dir;
  e.rssi10 = (int16_t)lroundf(rssi * 10);
  e.snr10  = (int16_t)lroundf(snr * 10);
  e.len = (uint16_t)n;
  e.stored = (uint16_t)(n < LOG_DATA_MAX ? n : LOG_DATA_MAX);
  memcpy(e.data, d, e.stored);
  xSemaphoreGive(logMutex);
}

void logEvent(const char* fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  logAdd('E', (const uint8_t*)b, strlen(b));
}

// Tekst czytelny dla człowieka: znaki sterujące jako <CR>, <LF>, <C0>, <1B>...
void asciiRender(const uint8_t* d, size_t n, String& o) {
  for (size_t i = 0; i < n; i++) {
    uint8_t c = d[i];
    if (c == '\r') o += "<CR>";
    else if (c == '\n') o += "<LF>";
    else if (c >= 32 && c < 127) o += (char)c;
    else { char b[6]; snprintf(b, sizeof(b), "<%02X>", c); o += b; }
  }
}

// ============================================================================
// WALIDACJA I NVS
// ============================================================================
bool isValidBaud(uint32_t b) {
  const uint32_t v[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  for (uint32_t x : v) if (x == b) return true;
  return false;
}

bool isValidBw(float bw) {
  const float v[] = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};
  for (float x : v) if (fabsf(x - bw) < 0.01f) return true;
  return false;
}

void sanitizeConfig(Config& c) {
  if (c.nodeId < 1 || c.nodeId > 254) c.nodeId = 1;
  if (c.targetId < 1 || c.targetId > 254 || c.targetId == c.nodeId) c.targetId = (c.nodeId == 1) ? 2 : 1;
  if (!isValidBaud(c.uartBaud)) c.uartBaud = 9600;
  if (c.uartTimeout < 5) c.uartTimeout = 5;
  if (c.uartTimeout > 1000) c.uartTimeout = 1000;
  if (isnan(c.loraFreq) || c.loraFreq < 410.0f || c.loraFreq > 525.0f) c.loraFreq = 434.000f;
  if (!isValidBw(c.loraBw)) c.loraBw = 125.0f;
  if (c.loraSf < 7 || c.loraSf > 12) c.loraSf = 8;
  if (c.loraCr < 5 || c.loraCr > 8) c.loraCr = 5;
  if (c.loraPower < 2) c.loraPower = 2;
  if (c.loraPower > 17 && c.loraPower != 20) c.loraPower = 17;
  if (c.loraSync == 0) c.loraSync = 0x4B;
  c.wifiSsid[sizeof(c.wifiSsid) - 1] = 0;
  c.wifiPass[sizeof(c.wifiPass) - 1] = 0;
}

void loadConfig() {
  prefs.begin("lora_bridge", true);
  cfg.nodeId       = prefs.getUChar("nodeId", 1);
  cfg.targetId     = prefs.getUChar("targetId", 2);
  cfg.uartBaud     = prefs.getUInt("uartBaud", 9600);
  cfg.uartTimeout  = prefs.getUShort("uartTime", 15);
  cfg.useUsbBridge = prefs.getBool("usbBridge", false);
  // Domyślnie NIE 433.775 — to kanał sieci LoRa APRS. Dobierz zgodnie z bandplanem.
  cfg.loraFreq     = prefs.getFloat("freq", 434.000f);
  cfg.loraBw       = prefs.getFloat("bw", 125.0f);
  cfg.loraSf       = prefs.getUChar("sf", 8);
  cfg.loraCr       = prefs.getUChar("cr", 5);
  cfg.loraPower    = prefs.getUChar("pwr", 17);
  cfg.loraSync     = prefs.getUChar("sync", 0x4B);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  ssid.toCharArray(cfg.wifiSsid, sizeof(cfg.wifiSsid));
  pass.toCharArray(cfg.wifiPass, sizeof(cfg.wifiPass));
  prefs.end();
  sanitizeConfig(cfg);
}

void saveConfig() {
  prefs.begin("lora_bridge", false);
  prefs.putUChar("nodeId", cfg.nodeId);
  prefs.putUChar("targetId", cfg.targetId);
  prefs.putUInt("uartBaud", cfg.uartBaud);
  prefs.putUShort("uartTime", cfg.uartTimeout);
  prefs.putBool("usbBridge", cfg.useUsbBridge);
  prefs.putFloat("freq", cfg.loraFreq);
  prefs.putFloat("bw", cfg.loraBw);
  prefs.putUChar("sf", cfg.loraSf);
  prefs.putUChar("cr", cfg.loraCr);
  prefs.putUChar("pwr", cfg.loraPower);
  prefs.putUChar("sync", cfg.loraSync);
  prefs.putString("ssid", cfg.wifiSsid);
  prefs.putString("pass", cfg.wifiPass);
  prefs.end();
}

// ============================================================================
// RADIO
// ============================================================================
int16_t applyLoRaConfig() {
  if (!radio) return RADIOLIB_ERR_CHIP_NOT_FOUND;
  radio->clearDio0Action();
  int16_t st = radio->begin(cfg.loraFreq, cfg.loraBw, cfg.loraSf, cfg.loraCr,
                            cfg.loraSync, (int8_t)cfg.loraPower);
  if (st == RADIOLIB_ERR_NONE) st = radio->setCRC(true);
  if (st == RADIOLIB_ERR_NONE) {
    radio->setDio0Action(onDio0, RISING);
    dio0Flag = false;
    stats.toaMs = (uint32_t)(radio->getTimeOnAir(HEADER_SIZE + FRAG_MAX) / 1000);
    st = radio->startReceive();
  }
  stats.radioState = st;
  stats.radioOk = (st == RADIOLIB_ERR_NONE);
  if (stats.radioOk) radioCfgGood = cfg;
  return st;
}

bool initRadio() {
  // Najpierw pin RST z wybranej rewizji, potem pozostałe znane warianty
  // (gdyby wybrano złą rewizję). Pomijamy pin używany przez UART.
  const int rstPins[] = {(int)P_LORA_RST, 23, 14, 12};
  for (size_t i = 0; i < sizeof(rstPins) / sizeof(rstPins[0]); i++) {
    int pin = rstPins[i];
    if (pin < 0 || pin == 16 || pin == 17) continue;      // nigdy piny flasha PICO-D4
    if (pin == UART_TX_PIN || pin == UART_RX_PIN) continue;
    if (pin == P_OLED_SDA || pin == P_OLED_SCL) continue;
    if (i > 0 && pin == P_LORA_RST) continue;
    radioModule = new Module(P_LORA_CS, P_LORA_DIO0, pin, RADIOLIB_NC, SPI);
    radio = new SX1278(radioModule);
    int16_t st = applyLoRaConfig();
    if (st != RADIOLIB_ERR_CHIP_NOT_FOUND) return st == RADIOLIB_ERR_NONE;
  }
  radio = nullptr;
  stats.radioOk = false;
  return false;
}

// ============================================================================
// WĄTEK 1: UART -> wiadomości (ramkowanie: KissFramer z BRIDGE_CORE)
// ============================================================================
void uartRxTask(void*) {
  framer.reset();
  auto emit = [](const Message& m) {
    xQueueSend(txQueue, &m, portMAX_DELAY);   // przy pełnej kolejce czekamy, UART buforuje 4 KB
  };
  for (;;) {
    HardwareSerial& port = bridgePort();
    while (port.available() > 0) framer.feed((uint8_t)port.read(), millis(), emit);
    framer.poll(millis(), cfg.uartTimeout, cfg.uartBaud, emit);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================================================================
// WĄTEK 2: radio LoRa (jedyny wątek dotykający SX1278)
// ============================================================================
static Message  txMsg;
static uint16_t txOffset = 0;
static uint8_t  txSeq = 0;
static uint8_t  txBuf[HEADER_SIZE + FRAG_MAX];
static uint8_t  rxBuf[256];

// Dekoduje ramkę KISS do postaci TNC2 i zapisuje do podglądu WWW.
void setAprsPreview(bool rx, const uint8_t* buf, size_t len) {
  char t[sizeof(stats.lastAprsRx)];
  int r = kissToTnc2(buf, len, t, sizeof(t));
  if (r == 1) { if (rx) stats.kissRx++; else stats.kissTx++; }
  if (r == 0 && buf[0] != KISS_FEND) return;     // zwykły tekst — bez podglądu AX.25
  portENTER_CRITICAL(&statsMux);
  memcpy(rx ? stats.lastAprsRx : stats.lastAprsTx, t, sizeof(t));
  portEXIT_CRITICAL(&statsMux);
}

static float curRssi = 0, curSnr = 0;   // sygnał bieżącego pakietu (do logu)

void deliverToUart(const uint8_t* d, size_t n) {
  bridgePort().write(d, n);
  logAdd('R', d, n, curRssi, curSnr);
  stats.rxMsgs++;
  stats.rxBytes += n;
  setPreview("RX:", d, n);
  setAprsPreview(true, d, n);
}

void handleRadioRx() {
  dio0Flag = false;
  size_t len = radio->getPacketLength();
  if (len > sizeof(rxBuf)) len = sizeof(rxBuf);
  int16_t st = radio->readData(rxBuf, len);
  float rssi = radio->getRSSI();
  float snr  = radio->getSNR();
  radio->startReceive();

  if (st == RADIOLIB_ERR_CRC_MISMATCH || st == RADIOLIB_ERR_LORA_HEADER_DAMAGED) {
    stats.crcErrors++;
    logEvent("Bledny pakiet LoRa (CRC), %u B, RSSI %.1f dBm, SNR %.1f dB", (unsigned)len, rssi, snr);
    return;
  }
  if (st != RADIOLIB_ERR_NONE) return;

  // Każdy poprawnie odebrany pakiet (także obcy) — żeby było widać, że radio słyszy.
  char t[16];
  formatTime(t, sizeof(t));
  stats.airPackets++;
  portENTER_CRITICAL(&statsMux);
  stats.lastRssi = rssi;
  stats.lastSnr = snr;
  memcpy(stats.lastRxTime, t, sizeof(t));
  portEXIT_CRITICAL(&statsMux);

  curRssi = rssi;
  curSnr = snr;
  uint32_t dropBefore = reasm.dropped, foreignBefore = reasm.foreign, otherBefore = reasm.notOurs;
  if (reasm.feed(rxBuf, len, cfg.nodeId, cfg.targetId, deliverToUart)) stats.rxPackets++;
  if (reasm.dropped != dropBefore)
    logEvent("Zgubiony fragment - wiadomosc odrzucona (pakiet #%u)", rxBuf[3]);
  if (reasm.foreign != foreignBefore)
    logEvent("Pakiet mostu dla innego ID: od N%u do N%u (ten wezel: N%u, czeka na N%u)",
             reasm.foreignFrom, reasm.foreignTo, cfg.nodeId, cfg.targetId);
  if (reasm.notOurs != otherBefore)
    logEvent("Obcy pakiet LoRa (inny protokol), %u B, RSSI %.1f dBm", (unsigned)len, rssi);
  stats.lostPackets = reasm.lost;
  stats.droppedMsgs = reasm.dropped;
  stats.foreign     = reasm.foreign;
  stats.notOurs     = reasm.notOurs;
  stats.foreignFrom = reasm.foreignFrom;
  stats.foreignTo   = reasm.foreignTo;
}

// true = kanał zajęty. Gdy wolny, radio zostaje w standby (gotowe do TX).
// CAD robione nieblokująco przez DIO0 + rejestr IRQ (scanChannel() z RadioLib 7
// wykrywa preambułę tylko przez DIO1, którego tu nie używamy).
bool channelBusy() {
  if (radio->getModemStatus() & 0x0A) return true;   // trwa odbiór (sync / nagłówek ważny)
  if (dio0Flag) return true;                         // właśnie przyszła ramka — obsłuż ją najpierw
  if (radio->startChannelScan() != RADIOLIB_ERR_NONE) return false;
  uint32_t t0 = millis();
  while (!dio0Flag && millis() - t0 < 50) vTaskDelay(1);   // CAD trwa ~2 symbole
  dio0Flag = false;                                  // to był CadDone
  if (radio->getChannelScanResult() != RADIOLIB_PREAMBLE_DETECTED) return false;
  radio->startReceive();
  return true;
}

void sendNextFragment() {
  size_t n = buildFragment(txMsg, txOffset, cfg.nodeId, cfg.targetId, txSeq++, txBuf);

  uint32_t timeout = (uint32_t)(radio->getTimeOnAir(n) / 1000) + 200;
  dio0Flag = false;
  int16_t st = radio->startTransmit(txBuf, n);
  if (st == RADIOLIB_ERR_NONE) {
    uint32_t t0 = millis();
    while (!dio0Flag && millis() - t0 < timeout) vTaskDelay(1);  // nie blokuje CPU
    bool done = dio0Flag;
    dio0Flag = false;
    radio->finishTransmit();
    if (done) stats.txPackets++;
    else { stats.txErrors++; logEvent("Blad nadawania: brak potwierdzenia TxDone"); }
  } else {
    stats.txErrors++;
    logEvent("Blad nadawania: startTransmit = %d", st);
  }
  radio->startReceive();

  if (txOffset >= txMsg.len) {
    stats.txMsgs++;
    stats.txBytes += txMsg.len;
    setPreview("TX:", txMsg.data, txMsg.len);
    setAprsPreview(false, txMsg.data, txMsg.len);
    logAdd('T', txMsg.data, txMsg.len);
    txActive = false;
  }
}

void loraTask(void*) {
  uint32_t nextTryAt = 0;
  uint32_t busySince = 0;
  bool waitingBusy = false;

  for (;;) {
    if (!radio) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

    if (reconfigureRadioFlag) {
      reconfigureRadioFlag = false;
      reasm.reset();
      if (applyLoRaConfig() != RADIOLIB_ERR_NONE) {
        // Przywróć ostatnie działające parametry radia
        cfg.loraFreq = radioCfgGood.loraFreq; cfg.loraBw = radioCfgGood.loraBw;
        cfg.loraSf = radioCfgGood.loraSf;     cfg.loraCr = radioCfgGood.loraCr;
        cfg.loraPower = radioCfgGood.loraPower; cfg.loraSync = radioCfgGood.loraSync;
        applyLoRaConfig();
      }
    }

    if (dio0Flag) handleRadioRx();

    if (!txActive) {
      if (xQueueReceive(txQueue, &txMsg, pdMS_TO_TICKS(3)) == pdPASS && txMsg.len > 0) {
        txActive = true;
        txOffset = 0;
        waitingBusy = false;
        nextTryAt = millis();
      }
      continue;
    }

    if ((int32_t)(millis() - nextTryAt) >= 0 && !dio0Flag) {
      if (!waitingBusy) { busySince = millis(); }
      bool giveUp = waitingBusy && (millis() - busySince > BUSY_GIVEUP_MS);
      if (giveUp) logEvent("Kanal zajety ponad %u s - nadaje mimo to", BUSY_GIVEUP_MS / 1000);
      if (!giveUp && channelBusy()) {
        waitingBusy = true;
        stats.cadWaits++;
        nextTryAt = millis() + random(40, 160);
      } else {
        waitingBusy = false;
        sendNextFragment();
        nextTryAt = millis();
      }
    }
    vTaskDelay(1);
  }
}

// ============================================================================
// INTERFEJS WWW
// ============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LoRa UART Bridge</title>
<style>
  :root { --bg:#0f172a; --card:#1e293b; --accent:#38bdf8; --ok:#22c55e; --bad:#f87171; --txt:#f1f5f9; --muted:#94a3b8; }
  body { font-family:system-ui,-apple-system,sans-serif; background:var(--bg); color:var(--txt); margin:0; padding:15px; }
  .container { max-width:1280px; margin:0 auto; }
  .header { display:flex; justify-content:space-between; align-items:center; gap:10px; background:var(--card); padding:15px 20px; border-radius:10px; border-left:5px solid var(--accent); margin-bottom:15px; }
  .header h1 { margin:0; font-size:1.2rem; }
  .badge { background:#334155; padding:6px 12px; border-radius:6px; font-family:monospace; color:var(--accent); white-space:nowrap; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(290px,1fr)); gap:15px; margin-bottom:15px; }
  .card { background:var(--card); padding:18px; border-radius:10px; }
  .card h2 { margin-top:0; font-size:1.05rem; color:var(--accent); border-bottom:1px solid #334155; padding-bottom:8px; }
  .row { display:flex; justify-content:space-between; gap:8px; padding:5px 0; border-bottom:1px dashed #334155; font-size:0.9rem; }
  .v { font-weight:bold; font-family:monospace; text-align:right; }
  .log { background:#090d16; padding:10px; border-radius:6px; font-family:monospace; font-size:0.8rem; color:#a7f3d0; overflow-x:auto; white-space:nowrap; margin-top:6px; border:1px solid #334155; }
  label { display:block; font-size:0.85rem; color:var(--muted); margin:10px 0 4px; }
  input,select { width:100%; padding:8px 10px; background:#0f172a; border:1px solid #334155; border-radius:6px; color:#fff; box-sizing:border-box; font-size:0.9rem; }
  .btn { background:var(--accent); color:#0f172a; border:none; padding:10px 16px; border-radius:6px; font-weight:bold; cursor:pointer; width:100%; margin-top:14px; }
  .btn-sec { background:#475569; color:#fff; }
  .ok { color:var(--ok); } .bad { color:var(--bad); }
  small.hint { color:var(--muted); font-size:0.78rem; }
  .term-card { margin-bottom:15px; }
  .term-head { display:flex; flex-wrap:wrap; justify-content:space-between; align-items:center; gap:10px; border-bottom:1px solid #334155; padding-bottom:10px; margin-bottom:10px; }
  .term-head h2 { margin:0; border:0; padding:0; }
  .tools { display:flex; flex-wrap:wrap; align-items:center; gap:8px; }
  .tools select { width:auto; padding:6px 8px; }
  .tools button { background:#334155; color:#fff; border:1px solid #475569; border-radius:6px; padding:6px 12px; cursor:pointer; font-size:0.85rem; }
  .tools button:hover { background:#475569; }
  .tools button.on { background:var(--accent); color:#0f172a; border-color:var(--accent); }
  .chk { display:inline-flex; align-items:center; gap:4px; margin:0; font-size:0.85rem; color:var(--txt); cursor:pointer; }
  .chk input { width:auto; margin:0; }
  .term { font-family:ui-monospace,Consolas,"Courier New",monospace; font-size:13px; line-height:1.4; height:calc(50 * 1.4em); min-height:12em;
          resize:vertical; overflow:auto; background:#05080f; border:1px solid #334155; border-radius:6px; padding:8px 10px;
          color:#d1fae5; white-space:pre-wrap; word-break:break-all; }
  .term .ln { display:block; }
  .term .hT { color:#fbbf24; font-weight:bold; }
  .term .hR { color:#4ade80; font-weight:bold; }
  .term .hE { color:#fca5a5; }
  .term .meta { color:#64748b; font-weight:normal; }
  .term .aprs { color:#e2e8f0; padding-left:2ch; }
  .term .hex { color:#7dd3fc; padding-left:2ch; }
  .term .asc { color:#cbd5e1; padding-left:2ch; }
  .term .note { color:#94a3b8; font-style:italic; }
  .term .sep { display:block; height:0.5em; }
  .txrow { display:flex; gap:8px; margin-top:8px; }
  .txrow select { width:auto; padding:8px; }
  .txrow input { flex:1; min-width:0; font-family:ui-monospace,Consolas,"Courier New",monospace; font-size:14px; background:#05080f; border-color:#475569; }
  .txrow input:focus { outline:none; border-color:var(--accent); }
  .txrow button { background:var(--accent); color:#0f172a; border:none; border-radius:6px; padding:8px 18px; font-weight:bold; cursor:pointer; }
  .txstat { font-size:0.8rem; margin-top:4px; min-height:1.1em; color:var(--muted); }
  @media (max-width:600px) { .txrow { flex-wrap:wrap; } .txrow input { flex-basis:100%; order:-1; } }
  .term-foot { display:flex; justify-content:space-between; color:var(--muted); font-size:0.8rem; margin-top:6px; }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div><h1>TTGO LoRa32 &mdash; most UART &harr; LoRa</h1>
    <small style="color:var(--muted)"><span id="board"></span> &middot; Radio: <span id="radio">...</span></small></div>
    <div class="badge" id="clock">--:--:--</div>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Statystyki (na żywo)</h2>
      <div class="row"><span>Wiadomości TX / RX</span><span class="v" id="msgs">0 / 0</span></div>
      <div class="row"><span>Ramki KISS/AX.25 TX / RX</span><span class="v" id="kiss">0 / 0</span></div>
      <div class="row"><span>Ramki radiowe TX / RX (nasze)</span><span class="v" id="pkts">0 / 0</span></div>
      <div class="row"><span>Słyszane w eterze (wszystkie)</span><span class="v" id="air">0</span></div>
      <div class="row"><span>Obce: inne ID / inny protokół</span><span class="v" id="foreign">0 / 0</span></div>
      <div class="row"><span>Bajty TX / RX</span><span class="v" id="bytes">0 / 0</span></div>
      <div class="row"><span>RSSI / SNR</span><span class="v" id="sig">--</span></div>
      <div class="row"><span>Odczekania (kanał zajęty)</span><span class="v" id="cad">0</span></div>
      <div class="row"><span>CRC / zgubione ramki / odrzucone wiad.</span><span class="v" id="errs">0 / 0 / 0</span></div>
      <div class="row"><span>Błędy TX / w kolejce</span><span class="v" id="txq">0 / 0</span></div>
      <div class="row"><span>Czas nadawania pełnej ramki</span><span class="v" id="toa">-- ms</span></div>
      <div class="row"><span>Ostatni odbiór</span><span class="v" id="rxTime">--:--:--</span></div>
    </div>

    <div class="card">
      <h2>Monitor i test</h2>
      <div id="warn" style="display:none;background:#7f1d1d;color:#fee2e2;padding:10px;border-radius:6px;font-size:0.85rem;margin-bottom:8px"></div>
      <p class="hint" style="color:var(--muted);font-size:0.85rem;margin-top:0">Pełna treść przesyłanych danych jest w <b>terminalu</b> poniżej.</p>
      <p class="hint" style="color:var(--muted);font-size:0.85rem">Tekst do wysłania wpisujesz w polu pod terminalem.</p>
    </div>
  </div>

  <div class="card term-card">
    <div class="term-head">
      <h2>Terminal danych</h2>
      <div class="tools">
        <select id="tView" title="Sposób wyświetlania">
          <option value="all">APRS + HEX</option>
          <option value="aprs">Tylko APRS / tekst</option>
          <option value="hex">Tylko HEX</option>
          <option value="ascii">Surowe ASCII</option>
        </select>
        <label class="chk"><input type="checkbox" id="fT" checked> TX</label>
        <label class="chk"><input type="checkbox" id="fR" checked> RX</label>
        <label class="chk"><input type="checkbox" id="fE" checked> Zdarzenia</label>
        <label class="chk"><input type="checkbox" id="tAuto" checked> Auto-przewijanie</label>
        <button type="button" id="tPause">Pauza</button>
        <button type="button" id="tClear">Wyczyść</button>
        <button type="button" id="tSave">Pobierz .txt</button>
      </div>
    </div>
    <div id="term" class="term"></div>
    <form id="txForm" class="txrow" autocomplete="off">
      <select id="txMode" title="Format wpisywanych danych">
        <option value="text">Tekst</option>
        <option value="hex">HEX</option>
      </select>
      <input type="text" id="txIn" maxlength="2000" spellcheck="false"
             placeholder="Wpisz tekst i naciśnij Enter — pójdzie przez LoRa na UART drugiej płytki">
      <select id="txEol" title="Zakończenie linii dodawane do tekstu">
        <option value="crlf">CR+LF</option>
        <option value="cr">CR</option>
        <option value="lf">LF</option>
        <option value="none">bez końca linii</option>
      </select>
      <button type="submit">Wyślij</button>
    </form>
    <div class="txstat" id="txStat">Enter = wyślij · ↑/↓ = poprzednie wpisy · Uwaga: VP-Digi traktuje tekst zakończony Enterem jako polecenie (np. „config” wyłącza tryb KISS).</div>
    <div class="term-foot">
      <span><span style="color:#fbbf24">■ TX</span> UART → LoRa &nbsp; <span style="color:#4ade80">■ RX</span> LoRa → UART &nbsp; <span style="color:#fca5a5">■</span> zdarzenia &nbsp;·&nbsp; okno można powiększyć, ciągnąc za prawy dolny róg</span>
      <span id="tInfo"></span>
    </div>
  </div>

  <form action="/save" method="POST">
    <div class="grid">
      <div class="card">
        <h2>1. Węzeł i UART</h2>
        <label>ID tego urządzenia (1-254):</label>
        <input type="number" name="nodeId" id="nodeId" min="1" max="254">
        <label>ID urządzenia docelowego (1-254):</label>
        <input type="number" name="targetId" id="targetId" min="1" max="254">
        <label>Prędkość UART:</label>
        <select name="uartBaud" id="uartBaud">
          <option value="1200">1200</option><option value="2400">2400</option>
          <option value="4800">4800</option><option value="9600">9600</option>
          <option value="19200">19200</option><option value="38400">38400</option>
          <option value="57600">57600</option><option value="115200">115200</option>
        </select>
        <label>Timeout ciszy UART (ms):</label>
        <input type="number" name="uartTimeout" id="uartTimeout" min="5" max="1000">
        <small class="hint">Dla ramek KISS timeout jest automatycznie wydłużany.</small>
        <label>Port mostu:</label>
        <select name="useUsbBridge" id="useUsbBridge">
          <option value="0" id="uartOpt">UART2 (zewnętrzne piny)</option>
          <option value="1">USB (wirtualny COM do PC)</option>
        </select>
      </div>

      <div class="card">
        <h2>2. Radio LoRa (SX1278)</h2>
        <label>Częstotliwość (MHz):</label>
        <input type="number" step="0.001" name="loraFreq" id="loraFreq" min="410" max="525">
        <label>Szerokość pasma:</label>
        <select name="loraBw" id="loraBw">
          <option value="62.5">62.5 kHz</option><option value="125.0">125 kHz</option>
          <option value="250.0">250 kHz</option><option value="500.0">500 kHz</option>
        </select>
        <label>Spreading Factor:</label>
        <select name="loraSf" id="loraSf">
          <option value="7">SF7 (najszybciej)</option><option value="8">SF8</option>
          <option value="9">SF9</option><option value="10">SF10</option>
          <option value="11">SF11</option><option value="12">SF12 (najdalej)</option>
        </select>
        <label>Coding Rate:</label>
        <select name="loraCr" id="loraCr">
          <option value="5">4/5</option><option value="6">4/6</option>
          <option value="7">4/7</option><option value="8">4/8</option>
        </select>
        <label>Moc (dBm, 2-17 lub 20):</label>
        <input type="number" name="loraPower" id="loraPower" min="2" max="20">
        <label>SyncWord (dziesiętnie, 75 = 0x4B):</label>
        <input type="number" name="loraSync" id="loraSync" min="1" max="255">
        <small class="hint">Oba węzły muszą mieć identyczne parametry radia.</small>
      </div>

      <div class="card">
        <h2>3. WiFi i zegar</h2>
        <label>SSID:</label>
        <input type="text" name="wifiSsid" id="wifiSsid" maxlength="32" placeholder="Puste = tylko punkt dostępowy">
        <label>Hasło WiFi:</label>
        <input type="password" name="wifiPass" id="wifiPass" maxlength="64" placeholder="Zostaw puste, aby nie zmieniać">
        <p class="hint" style="font-size:0.8rem;color:var(--muted)">Zegar z <b>tempus1.gum.gov.pl</b>. Płytka zawsze wystawia AP
        <b>LoRa-Bridge-Node-X</b> (hasło 12345678), chyba że połączy się z WiFi — wtedy oba naraz.</p>
        <button type="submit" class="btn">Zapisz i zastosuj</button>
        <button type="button" class="btn btn-sec" onclick="if(confirm('Zrestartować?')) fetch('/reboot',{method:'POST'});">Restart ESP32</button>
      </div>
    </div>
  </form>
</div>

<script>
const $ = id => document.getElementById(id);
function updateStats() {
  fetch('/api/status').then(r => r.json()).then(d => {
    $('clock').innerText = d.time + (d.ntp ? ' (NTP)' : '');
    $('radio').innerHTML = d.radioOk ? '<span class="ok">OK</span>' : '<span class="bad">BŁĄD (' + d.radioState + ')</span>';
    $('msgs').innerText = d.txMsg + ' / ' + d.rxMsg;
    $('pkts').innerText = d.txPkt + ' / ' + d.rxPkt;
    $('kiss').innerText = d.kissTx + ' / ' + d.kissRx;
    $('air').innerText = d.air;
    $('foreign').innerText = d.foreign + ' / ' + d.notOurs;
    const w = $('warn');
    if (d.foreign > 0 && d.rxPkt == 0) {
      w.style.display = 'block';
      w.innerText = 'Słychać drugi most, ale ID się nie zgadzają: ostatnia ramka była od węzła ' + d.ffrom +
        ' do węzła ' + d.fto + ', a ten węzeł ma ID ' + d.node + ' i czeka na ramki od ' + d.peer +
        '. Ustaw na drugiej płytce odwrotne ID (np. tu 1→2, tam 2→1).';
    } else if (d.notOurs > 0 && d.air == d.notOurs) {
      w.style.display = 'block';
      w.innerText = 'Odbierane są tylko pakiety innego systemu LoRa (np. LoRa APRS) — drugi most nie jest słyszany. Sprawdź częstotliwość, SF, BW, CR i SyncWord na obu płytkach.';
    } else { w.style.display = 'none'; }
    $('bytes').innerText = d.txB + ' / ' + d.rxB + ' B';
    $('sig').innerText = d.air > 0 ? (d.rssi + ' dBm / ' + d.snr + ' dB') : '--';
    $('cad').innerText = d.cad;
    $('errs').innerText = d.crc + ' / ' + d.lost + ' / ' + d.drop;
    $('txq').innerText = d.txErr + ' / ' + d.queue;
    $('toa').innerText = d.toa + ' ms';
    $('rxTime').innerText = d.rxTime;
  }).catch(e => {});
}
function loadCurrentConfig() {
  fetch('/api/config').then(r => r.json()).then(c => {
    $('nodeId').value = c.nodeId; $('targetId').value = c.targetId;
    $('uartBaud').value = c.uartBaud; $('uartTimeout').value = c.uartTimeout;
    $('useUsbBridge').value = c.useUsbBridge ? '1' : '0';
    $('loraFreq').value = c.loraFreq.toFixed(3); $('loraBw').value = c.loraBw.toFixed(1);
    $('loraSf').value = c.loraSf; $('loraCr').value = c.loraCr;
    $('loraPower').value = c.loraPower; $('loraSync').value = c.loraSync;
    $('wifiSsid').value = c.wifiSsid;
    $('wifiPass').placeholder = c.wifiPassSet ? 'Ustawione — puste = bez zmian' : 'Brak hasła';
    $('uartOpt').text = 'UART2 (GPIO' + c.uartRx + ' = RX, GPIO' + c.uartTx + ' = TX)';
    $('board').innerText = c.board;
  });
}
// ---------------- Terminal danych ----------------
const T = { entries: [], last: 0, boot: 0, busy: false, paused: false, max: 3000 };
const term = $('term');

function hexDump(h) {
  const out = [];
  for (let off = 0; off < h.length / 2; off += 16) {
    let hx = '', as = '';
    for (let i = 0; i < 16; i++) {
      const p = (off + i) * 2;
      if (p < h.length) {
        const b = parseInt(h.substr(p, 2), 16);
        hx += h.substr(p, 2) + ' ';
        as += (b >= 32 && b < 127) ? String.fromCharCode(b) : '.';
      } else hx += '   ';
      if (i == 7) hx += ' ';
    }
    out.push(off.toString(16).toUpperCase().padStart(4, '0') + '  ' + hx + ' |' + as + '|');
  }
  return out;
}

// Zwraca listę linii [klasa, tekst] dla wpisu w wybranym widoku.
function entryLines(e, view) {
  const L = [];
  const dirTxt = { T: 'TX  UART → LoRa', R: 'RX  LoRa → UART', E: '**  zdarzenie' }[e.d] || e.d;
  let meta = '';
  if (e.d != 'E') {
    meta = e.n + ' B';
    if (e.k == 1) meta += ' · APRS/AX.25';
    else if (e.k == 2) meta += ' · komenda KISS';
    if (e.d == 'R') meta += ' · RSSI ' + e.r.toFixed(1) + ' dBm · SNR ' + e.s.toFixed(1) + ' dB';
    if (e.h.length / 2 < e.n) meta += ' · pokazano ' + (e.h.length / 2) + ' B';
  }
  L.push(['h' + e.d, e.t + '  ' + dirTxt + (e.d == 'E' ? '  ' + e.a : ''), meta, '#' + e.i]);
  if (e.d == 'E') return L;
  const decoded = (e.k == 1 || e.k == 2) ? e.a : null;
  if (view == 'all') {
    L.push(decoded ? ['aprs', decoded] : ['asc', e.x]);
    hexDump(e.h).forEach(x => L.push(['hex', x]));
  } else if (view == 'aprs') {
    L.push(decoded ? ['aprs', decoded] : ['asc', e.x]);
  } else if (view == 'hex') {
    hexDump(e.h).forEach(x => L.push(['hex', x]));
  } else {
    L.push(['asc', e.x]);
  }
  return L;
}

function shown(e) {
  return e.note || (e.d == 'T' && $('fT').checked) || (e.d == 'R' && $('fR').checked) || (e.d == 'E' && $('fE').checked);
}

function renderEntry(e, frag) {
  const view = $('tView').value;
  if (e.note) {
    const d = document.createElement('span'); d.className = 'ln note'; d.textContent = '— ' + e.note + ' —';
    frag.appendChild(d); return;
  }
  entryLines(e, view).forEach((l, idx) => {
    const d = document.createElement('span');
    d.className = 'ln ' + l[0];
    if (idx == 0) {
      d.textContent = l[1];
      if (l[2] || l[3]) {
        const m = document.createElement('span'); m.className = 'meta';
        m.textContent = (l[2] ? '  [' + l[2] + ']' : '') + '  ' + l[3];
        d.appendChild(m);
      }
    } else d.textContent = l[1];
    frag.appendChild(d);
  });
  if (view != 'aprs' && view != 'ascii') { const sp = document.createElement('span'); sp.className = 'sep'; frag.appendChild(sp); }
}

function nearBottom() { return term.scrollHeight - term.scrollTop - term.clientHeight < 40; }
function scrollIfNeeded(stick) { if ($('tAuto').checked && stick) term.scrollTop = term.scrollHeight; }

function renderAll() {
  const frag = document.createDocumentFragment();
  T.entries.filter(shown).forEach(e => renderEntry(e, frag));
  term.textContent = '';
  if (!T.entries.length) {
    const d = document.createElement('span'); d.className = 'ln note';
    d.textContent = 'Czekam na dane… (tu pojawi się każda wiadomość przesłana przez most w obu kierunkach)';
    frag.appendChild(d);
  }
  term.appendChild(frag);
  term.scrollTop = term.scrollHeight;
  updateInfo();
}

function addEntries(list) {
  if (!list.length) return;
  const stick = nearBottom();
  if (!T.entries.length) term.textContent = '';
  const frag = document.createDocumentFragment();
  list.forEach(e => { T.entries.push(e); if (shown(e)) renderEntry(e, frag); });
  term.appendChild(frag);
  if (T.entries.length > T.max) { T.entries.splice(0, T.entries.length - T.max); renderAll(); return; }
  scrollIfNeeded(stick);
  updateInfo();
}

function updateInfo() {
  const n = T.entries.filter(e => !e.note).length;
  $('tInfo').textContent = n + ' wpisów' + (T.paused ? ' · PAUZA' : '');
}

function pollLog() {
  if (T.busy || T.paused) return;
  T.busy = true;
  fetch('/api/log?since=' + T.last + '&boot=' + T.boot).then(r => r.json()).then(d => {
    const add = [];
    if (T.boot && d.boot != T.boot) add.push({ note: 'ESP32 uruchomił się ponownie — historia od nowa' });
    else if (d.gap) add.push({ note: 'część wpisów pominięta (ESP32 pamięta ' + 100 + ' ostatnich)' });
    T.boot = d.boot;
    if (d.e.length) T.last = d.e[d.e.length - 1].i;
    else if (d.last < T.last) T.last = d.last;
    addEntries(add.concat(d.e));
  }).catch(e => {}).finally(() => { T.busy = false; });
}

$('tView').onchange = renderAll;
['fT', 'fR', 'fE'].forEach(id => $(id).onchange = renderAll);
$('tPause').onclick = () => {
  T.paused = !T.paused;
  $('tPause').textContent = T.paused ? 'Wznów' : 'Pauza';
  $('tPause').classList.toggle('on', T.paused);
  updateInfo();
  if (!T.paused) pollLog();
};
$('tClear').onclick = () => { T.entries = []; renderAll(); };
$('tSave').onclick = () => {
  const view = $('tView').value, out = [];
  T.entries.filter(shown).forEach(e => {
    if (e.note) { out.push('--- ' + e.note + ' ---'); return; }
    entryLines(e, view).forEach((l, i) => out.push(i == 0 ? l[1] + (l[2] ? '  [' + l[2] + ']' : '') + '  ' + l[3] : '  ' + l[1]));
    out.push('');
  });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([out.join('\r\n')], { type: 'text/plain' }));
  a.download = 'lora_bridge_' + new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19) + '.txt';
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};

// ---------------- Wysyłanie z klawiatury ----------------
const TX = { hist: [], pos: -1, draft: '' };
try { TX.hist = JSON.parse(localStorage.getItem('txHist') || '[]'); } catch (e) {}
try { const v = localStorage.getItem('txEol'); if (v) $('txEol').value = v; } catch (e) {}

function txStatus(msg, ok) {
  $('txStat').textContent = msg;
  $('txStat').style.color = ok === undefined ? 'var(--muted)' : (ok ? 'var(--ok)' : 'var(--bad)');
}
function txModeChanged() {
  const hex = $('txMode').value == 'hex';
  $('txEol').disabled = hex;
  $('txIn').placeholder = hex ? 'Bajty HEX, np. C0 00 82 A0 ... C0 — wysyłane dokładnie tak, jak wpisane'
                              : 'Wpisz tekst i naciśnij Enter — pójdzie przez LoRa na UART drugiej płytki';
}
$('txMode').onchange = () => { txModeChanged(); $('txIn').focus(); };
$('txEol').onchange = () => { try { localStorage.setItem('txEol', $('txEol').value); } catch (e) {} $('txIn').focus(); };

$('txForm').onsubmit = (ev) => {
  ev.preventDefault();
  const v = $('txIn').value;
  const mode = $('txMode').value;
  if (!v.length && (mode == 'hex' || $('txEol').value == 'none')) { txStatus('Nic do wysłania.', false); return; }
  const body = new URLSearchParams({ msg: v, mode: mode, eol: $('txEol').value });
  $('txIn').disabled = true;
  fetch('/api/send', { method: 'POST', body: body })
    .then(r => r.text().then(t => {
      if (r.ok) {
        txStatus('Wysłano: ' + t, true);
        if (v.length && TX.hist[TX.hist.length - 1] !== v) {
          TX.hist.push(v);
          if (TX.hist.length > 50) TX.hist.shift();
          try { localStorage.setItem('txHist', JSON.stringify(TX.hist)); } catch (e) {}
        }
        $('txIn').value = '';
        TX.pos = -1;
        pollLog();
      } else txStatus('Błąd: ' + t, false);
    }))
    .catch(() => txStatus('Błąd: brak połączenia z płytką.', false))
    .finally(() => { $('txIn').disabled = false; $('txIn').focus(); });
};

// Historia wpisów strzałkami ↑/↓ (jak w terminalu)
$('txIn').onkeydown = (ev) => {
  if (ev.key != 'ArrowUp' && ev.key != 'ArrowDown') return;
  if (!TX.hist.length) return;
  ev.preventDefault();
  if (TX.pos == -1) { TX.draft = $('txIn').value; }
  if (ev.key == 'ArrowUp') TX.pos = TX.pos == -1 ? TX.hist.length - 1 : Math.max(0, TX.pos - 1);
  else TX.pos = (TX.pos == -1 || TX.pos >= TX.hist.length - 1) ? -1 : TX.pos + 1;
  $('txIn').value = TX.pos == -1 ? TX.draft : TX.hist[TX.pos];
  const n = $('txIn').value.length;
  $('txIn').setSelectionRange(n, n);
};

// Kliknięcie w terminal przenosi kursor do pola wpisywania (chyba że zaznaczasz tekst)
term.addEventListener('mouseup', () => {
  if (!window.matchMedia('(pointer: fine)').matches) return;     // na telefonie nie wysuwaj klawiatury
  if (!window.getSelection || window.getSelection().toString() === '') $('txIn').focus();
});
txModeChanged();

loadCurrentConfig();
updateStats();
renderAll();
pollLog();
setInterval(updateStats, 1500);
setInterval(pollLog, 1000);
</script>
</body>
</html>
)rawliteral";

long argLong(const char* name, long def) {
  return server.hasArg(name) ? server.arg(name).toInt() : def;
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", INDEX_HTML);
  });

  server.on("/api/status", HTTP_GET, []() {
    Stats s;
    portENTER_CRITICAL(&statsMux);
    s = stats;
    portEXIT_CRITICAL(&statsMux);
    char t[16];
    formatTime(t, sizeof(t));
    unsigned queued = (unsigned)uxQueueMessagesWaiting(txQueue) + (txActive ? 1 : 0);

    char num[720];
    snprintf(num, sizeof(num),
      "{\"time\":\"%s\",\"ntp\":%s,\"radioOk\":%s,\"radioState\":%d,"
      "\"txPkt\":%lu,\"rxPkt\":%lu,\"txMsg\":%lu,\"rxMsg\":%lu,\"txB\":%lu,\"rxB\":%lu,"
      "\"rssi\":%.1f,\"snr\":%.1f,\"cad\":%lu,\"crc\":%lu,\"lost\":%lu,\"drop\":%lu,"
      "\"txErr\":%lu,\"queue\":%u,\"toa\":%lu,\"air\":%lu,\"foreign\":%lu,\"notOurs\":%lu,"
      "\"ffrom\":%u,\"fto\":%u,\"node\":%u,\"peer\":%u,\"kissTx\":%lu,\"kissRx\":%lu,",
      t, ntpSynced ? "true" : "false", s.radioOk ? "true" : "false", (int)s.radioState,
      (unsigned long)s.txPackets, (unsigned long)s.rxPackets,
      (unsigned long)s.txMsgs, (unsigned long)s.rxMsgs,
      (unsigned long)s.txBytes, (unsigned long)s.rxBytes,
      s.lastRssi, s.lastSnr, (unsigned long)s.cadWaits, (unsigned long)s.crcErrors,
      (unsigned long)s.lostPackets, (unsigned long)s.droppedMsgs,
      (unsigned long)s.txErrors, queued, (unsigned long)s.toaMs,
      (unsigned long)s.airPackets, (unsigned long)s.foreign, (unsigned long)s.notOurs,
      s.foreignFrom, s.foreignTo, cfg.nodeId, cfg.targetId,
      (unsigned long)s.kissTx, (unsigned long)s.kissRx);

    String j = num;
    j += "\"rxTime\":\"";  j += jsonEscape(s.lastRxTime);
    j += "\",\"logTxt\":\""; j += jsonEscape(s.lastLogTxt);
    j += "\",\"logHex\":\""; j += jsonEscape(s.lastLogHex);
    j += "\",\"aprsRx\":\""; j += jsonEscape(s.lastAprsRx);
    j += "\",\"aprsTx\":\""; j += jsonEscape(s.lastAprsTx);
    j += "\"}";
    server.send(200, "application/json", j);
  });

  // Historia: /api/log?since=<ostatnie id w przeglądarce>&boot=<id startu>
  server.on("/api/log", HTTP_GET, []() {
    uint32_t since = server.hasArg("since") ? strtoul(server.arg("since").c_str(), nullptr, 10) : 0;
    uint32_t boot  = server.hasArg("boot")  ? strtoul(server.arg("boot").c_str(), nullptr, 10) : 0;
    xSemaphoreTake(logMutex, portMAX_DELAY);
    uint32_t last = logNextId - 1;
    xSemaphoreGive(logMutex);
    if (boot != logBootId || since > last) since = 0;          // restart ESP32 → pełna historia
    uint32_t oldest = last >= LOG_ENTRIES ? last - LOG_ENTRIES + 1 : 1;
    uint32_t first = since + 1 < oldest ? oldest : since + 1;

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");
    char head[96];
    snprintf(head, sizeof(head), "{\"boot\":%lu,\"last\":%lu,\"gap\":%s,\"e\":[",
             (unsigned long)logBootId, (unsigned long)last, (since && since + 1 < oldest) ? "true" : "false");
    server.sendContent(head);

    static LogEntry e;            // kopia wpisu (poza blokadą formatujemy JSON)
    static char dec[400];
    bool firstOut = true;
    for (uint32_t id = first; id <= last; id++) {
      xSemaphoreTake(logMutex, portMAX_DELAY);
      e = logBuf[id % LOG_ENTRIES];
      xSemaphoreGive(logMutex);
      if (e.id != id) continue;   // nadpisany w międzyczasie

      String j;
      j.reserve(e.stored * 4 + 200);
      char hdr[160];
      snprintf(hdr, sizeof(hdr), "%s{\"i\":%lu,\"t\":\"%s\",\"d\":\"%c\",\"n\":%u,\"r\":%.1f,\"s\":%.1f,",
               firstOut ? "" : ",", (unsigned long)e.id, e.time, e.dir, e.len, e.rssi10 / 10.0f, e.snr10 / 10.0f);
      j += hdr;
      firstOut = false;

      String txt;
      int kind = 0;               // 0 = tekst/dane, 1 = APRS/AX.25, 2 = komenda KISS, 3 = zdarzenie
      if (e.dir == 'E') {
        kind = 3;
        txt.reserve(e.stored);
        for (uint16_t q = 0; q < e.stored; q++) txt += (char)e.data[q];
      } else if (e.stored && e.data[0] == KISS_FEND) {
        int r = kissToTnc2(e.data, e.stored, dec, sizeof(dec));
        kind = (r == 1) ? 1 : (r == 2) ? 2 : 0;
        txt = dec;
      }
      String asc;
      asciiRender(e.data, e.stored, asc);
      j += "\"k\":"; j += kind;
      j += ",\"a\":\""; j += jsonEscape(txt.c_str());
      j += "\",\"x\":\""; j += jsonEscape(asc.c_str());
      j += "\",\"h\":\"";
      static const char HEX_DIGITS[] = "0123456789ABCDEF";
      for (uint16_t q = 0; q < e.stored; q++) { j += HEX_DIGITS[e.data[q] >> 4]; j += HEX_DIGITS[e.data[q] & 15]; }
      j += "\"}";
      server.sendContent(j);
    }
    server.sendContent("]}");
    server.sendContent("");       // koniec odpowiedzi
  });

  server.on("/api/config", HTTP_GET, []() {
    char num[384];
    snprintf(num, sizeof(num),
      "{\"nodeId\":%u,\"targetId\":%u,\"uartBaud\":%lu,\"uartTimeout\":%u,"
      "\"useUsbBridge\":%s,\"loraFreq\":%.3f,\"loraBw\":%.2f,\"loraSf\":%u,\"loraCr\":%u,"
      "\"loraPower\":%u,\"loraSync\":%u,\"wifiPassSet\":%s,"
      "\"uartRx\":%d,\"uartTx\":%d,\"board\":\"%s\",",
      cfg.nodeId, cfg.targetId, (unsigned long)cfg.uartBaud, cfg.uartTimeout,
      cfg.useUsbBridge ? "true" : "false", cfg.loraFreq, cfg.loraBw, cfg.loraSf, cfg.loraCr,
      cfg.loraPower, cfg.loraSync, strlen(cfg.wifiPass) ? "true" : "false",
      UART_RX_PIN, UART_TX_PIN, BOARD_NAME);
    String j = num;
    j += "\"wifiSsid\":\""; j += jsonEscape(cfg.wifiSsid); j += "\"}";
    server.send(200, "application/json", j);
  });

  // Wysyłanie z klawiatury (terminal WWW) → LoRa → UART drugiej płytki.
  // Parametry (POST): msg, mode = text|hex, eol = crlf|cr|lf|none
  server.on("/api/send", HTTP_POST, []() {
    if (!server.hasArg("msg")) { server.send(400, "text/plain", "brak tresci"); return; }
    static Message m;
    String msg  = server.arg("msg");
    String mode = server.arg("mode");
    m.len = 0;

    if (mode == "hex") {
      // "C0 00 82 A0", "c0,00,82", "0xC0 0x00" — spacje, przecinki, ':' i '0x' są pomijane
      int nib = -1;
      for (size_t i = 0; i < msg.length(); i++) {
        char c = msg[i];
        if (c == ' ' || c == ',' || c == ':' || c == '\t' || c == '-') continue;
        if (c == '0' && i + 1 < msg.length() && (msg[i + 1] == 'x' || msg[i + 1] == 'X') && nib < 0) { i++; continue; }
        int v = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 :
                (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
        if (v < 0) { server.send(400, "text/plain", String("niedozwolony znak w HEX: '") + c + "'"); return; }
        if (nib < 0) nib = v;
        else {
          if (m.len >= MAX_MSG) { server.send(400, "text/plain", "za duzo danych (maks. 1024 B)"); return; }
          m.data[m.len++] = (uint8_t)((nib << 4) | v);
          nib = -1;
        }
      }
      if (nib >= 0) { server.send(400, "text/plain", "nieparzysta liczba cyfr HEX"); return; }
    } else {
      String eol = server.arg("eol");
      if (eol == "cr") msg += "\r";
      else if (eol == "lf") msg += "\n";
      else if (eol != "none") msg += "\r\n";            // domyślnie CR+LF
      if (msg.length() > MAX_MSG) { server.send(400, "text/plain", "za dlugi tekst (maks. 1024 B)"); return; }
      m.len = (uint16_t)msg.length();
      memcpy(m.data, msg.c_str(), m.len);
    }

    if (m.len == 0) { server.send(400, "text/plain", "nic do wyslania"); return; }
    if (xQueueSend(txQueue, &m, pdMS_TO_TICKS(50)) != pdPASS) {
      server.send(503, "text/plain", "kolejka nadawania pelna - sprobuj za chwile");
      return;
    }
    server.send(200, "text/plain", String(m.len) + " B w kolejce do nadania");
  });

  server.on("/save", HTTP_POST, []() {
    Config n = cfg;
    n.nodeId       = (uint8_t)constrain(argLong("nodeId", cfg.nodeId), 0, 255);
    n.targetId     = (uint8_t)constrain(argLong("targetId", cfg.targetId), 0, 255);
    n.uartBaud     = (uint32_t)argLong("uartBaud", cfg.uartBaud);
    n.uartTimeout  = (uint16_t)constrain(argLong("uartTimeout", cfg.uartTimeout), 5, 1000);
    n.useUsbBridge = argLong("useUsbBridge", cfg.useUsbBridge ? 1 : 0) == 1;
    if (server.hasArg("loraFreq")) n.loraFreq = server.arg("loraFreq").toFloat();
    if (server.hasArg("loraBw"))   n.loraBw   = server.arg("loraBw").toFloat();
    n.loraSf    = (uint8_t)constrain(argLong("loraSf", cfg.loraSf), 0, 255);
    n.loraCr    = (uint8_t)constrain(argLong("loraCr", cfg.loraCr), 0, 255);
    n.loraPower = (uint8_t)constrain(argLong("loraPower", cfg.loraPower), 0, 255);
    n.loraSync  = (uint8_t)constrain(argLong("loraSync", cfg.loraSync), 0, 255);

    bool wifiChanged = false;
    if (server.hasArg("wifiSsid")) {
      String ssid = server.arg("wifiSsid");
      String pass = server.arg("wifiPass");
      if (ssid != String(cfg.wifiSsid)) wifiChanged = true;
      ssid.toCharArray(n.wifiSsid, sizeof(n.wifiSsid));
      if (pass.length() > 0) {
        if (pass != String(cfg.wifiPass)) wifiChanged = true;
        pass.toCharArray(n.wifiPass, sizeof(n.wifiPass));
      } else if (ssid.length() == 0) {
        n.wifiPass[0] = 0;
      }
    }

    sanitizeConfig(n);

    bool radioChanged = n.loraFreq != cfg.loraFreq || n.loraBw != cfg.loraBw ||
                        n.loraSf != cfg.loraSf || n.loraCr != cfg.loraCr ||
                        n.loraPower != cfg.loraPower || n.loraSync != cfg.loraSync;
    bool portChanged  = n.uartBaud != cfg.uartBaud || n.useUsbBridge != cfg.useUsbBridge;

    cfg = n;
    saveConfig();

    if (portChanged) {
      Serial2.updateBaudRate(cfg.uartBaud);
      Serial.updateBaudRate(cfg.useUsbBridge ? cfg.uartBaud : 115200);
    }
    if (radioChanged) reconfigureRadioFlag = true;

    server.sendHeader("Location", "/");
    server.send(303);

    if (wifiChanged) {
      delay(500);
      ESP.restart();
    }
  });

  server.on("/reboot", HTTP_POST, []() {
    server.send(200, "text/plain", "Restart...");
    delay(500);
    ESP.restart();
  });

  server.begin();
}

// ============================================================================
// WĄTEK 3: OLED + WWW
// ============================================================================
void systemUiTask(void*) {
  uint32_t lastOled = 0;
  for (;;) {
    server.handleClient();

    if (oledAvailable && millis() - lastOled >= 500) {
      lastOled = millis();
      char t[16];
      formatTime(t, sizeof(t));

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);

      display.setCursor(0, 0);
      display.printf("N%u->N%u", cfg.nodeId, cfg.targetId);
      display.setCursor(128 - (int)strlen(t) * 6, 0);
      display.print(t);
      display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

      display.setCursor(0, 12);
      if (WiFi.status() == WL_CONNECTED) display.printf("IP:%s", WiFi.localIP().toString().c_str());
      else display.printf("AP:%s", WiFi.softAPIP().toString().c_str());

      display.setCursor(0, 22);
      if (stats.radioOk) display.printf("%.3f SF%u %lu", cfg.loraFreq, cfg.loraSf, (unsigned long)cfg.uartBaud);
      else display.print("RADIO: BLAD!");

      display.setCursor(0, 33);
      display.printf("TX:%lu (%luB)", (unsigned long)stats.txMsgs, (unsigned long)stats.txBytes);
      display.setCursor(0, 43);
      display.printf("RX:%lu (%luB)", (unsigned long)stats.rxMsgs, (unsigned long)stats.rxBytes);

      display.setCursor(0, 54);
      if (stats.foreign > 0 && stats.rxPackets == 0)
        display.printf("ZLE ID! slysze N%u>N%u", stats.foreignFrom, stats.foreignTo);
      else if (stats.airPackets > 0) display.printf("RSSI:%.0f SNR:%.1f", stats.lastRssi, stats.lastSnr);
      else display.print("Czekam na RX...");

      display.display();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================================
// WIFI + NTP
// ============================================================================
void initWiFiAndNTP() {
  String apName = "LoRa-Bridge-Node-" + String(cfg.nodeId);
  WiFi.persistent(false);

  if (strlen(cfg.wifiSsid) > 0) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName.c_str(), "12345678");
    WiFi.begin(cfg.wifiSsid, cfg.wifiPass);

    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Laczenie z WiFi:");
      display.println(cfg.wifiSsid);
      display.display();
    }

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) delay(250);

    if (WiFi.status() != WL_CONNECTED) {
      // Szukanie sieci przez STA zrywa klientów AP — zostaw sam stabilny AP.
      WiFi.disconnect(true);
      WiFi.mode(WIFI_AP);
      WiFi.softAP(apName.c_str(), "12345678");
    } else {
      WiFi.setAutoReconnect(true);
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str(), "12345678");
  }
  WiFi.setSleep(false);
  configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (wlaczenie zasilania)";
    case ESP_RST_EXT:       return "EXT (pin EN)";
    case ESP_RST_SW:        return "SW (ESP.restart)";
    case ESP_RST_PANIC:     return "PANIC (wyjatek / Guru Meditation)";
    case ESP_RST_INT_WDT:   return "INT_WDT (watchdog przerwan)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (watchdog zadan)";
    case ESP_RST_WDT:       return "WDT (inny watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (spadek napiecia zasilania!)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "NIEZNANY";
  }
}

// Diagnostyka na USB (115200) — tylko gdy USB NIE jest portem mostu.
#define DBG(...) do { if (!cfg.useUsbBridge) { Serial.printf(__VA_ARGS__); Serial.flush(); } } while (0)

// Czy pod adresem 0x3C na danych pinach odpowiada OLED? Przy braku — zwalnia I2C.
bool probeOled(int sda, int scl) {
  Wire.begin(sda, scl, 400000);
  Wire.setTimeOut(20);
  Wire.beginTransmission(0x3C);
  bool ok = (Wire.endTransmission() == 0);
  if (!ok) Wire.end();
  return ok;
}

// Rozpoznanie rewizji płytki po typie układu i położeniu OLED.
// Zwraca true, jeśli OLED został znaleziony (I2C zostaje wtedy aktywne).
bool detectBoard() {
  String model = ESP.getChipModel();
  bool pico = model.indexOf("PICO") >= 0;
  DBG("[2] Uklad: %s, flash %u MB, PSRAM: %s\n", model.c_str(),
      (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)), psramFound() ? "tak" : "nie");

  if (pico) {
    // T3 V1.6.x: OLED bez linii reset, GPIO16/17 = flash — nie ruszamy ich.
    bp = {"TTGO V2.1 (PICO-D4)", 21, 22, -1, 23, 15};
    return probeOled(21, 22);
  }

  // Układ z zewnętrznym flashem i bez PSRAM: GPIO16 jest bezpieczny (reset OLED w V1/V2).
  if (!psramFound()) {
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);
    delay(20);
    digitalWrite(16, HIGH);
    delay(20);
  }
  if (probeOled(21, 22)) { bp = {"TTGO V2", 21, 22, 16, 12, 15}; return true; }
  if (probeOled(4, 15))  { bp = {"TTGO V1", 4, 15, 16, 14, 23};  return true; }

  // Nie znaleziono OLED — przyjmij układ V2.1, radio i tak spróbuje kilku pinów RST.
  bp = {"nieznana (bez OLED)", 21, 22, -1, 23, 15};
  return false;
}

void setup() {
  loadConfig();

  Serial.setRxBufferSize(UART_RX_BUF);
  Serial.begin(cfg.useUsbBridge ? cfg.uartBaud : 115200);
  delay(50);
  DBG("\n\n=== LoRa UART Bridge ===\n");
  DBG("[0] Poprzedni reset: %s\n", resetReasonName(esp_reset_reason()));
  DBG("[1] Konfiguracja: node %u -> %u, UART %lu bd, %.3f MHz SF%u BW%.1f %u dBm\n",
      cfg.nodeId, cfg.targetId, (unsigned long)cfg.uartBaud, cfg.loraFreq,
      cfg.loraSf, cfg.loraBw, cfg.loraPower);

  bool oledFound = detectBoard();
  DBG("[3] Plytka: %s | OLED SDA %d SCL %d: %s | LoRa RST %d\n", BOARD_NAME,
      P_OLED_SDA, P_OLED_SCL, oledFound ? "znaleziony" : "BRAK", P_LORA_RST);

  Serial2.setRxBufferSize(UART_RX_BUF);
  Serial2.begin(cfg.uartBaud, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  DBG("[4] UART2 OK (RX GPIO%d, TX GPIO%d)\n", UART_RX_PIN, UART_TX_PIN);

  if (oledFound && display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("LoRa UART Bridge");
    display.println(BOARD_NAME);
    display.printf("UART RX%d TX%d\n", UART_RX_PIN, UART_TX_PIN);
    display.println("Inicjalizacja...");
    display.display();
  }
  DBG("[5] OLED: %s\n", oledAvailable ? "OK" : "BRAK");

  txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(Message));
  logMutex = xSemaphoreCreateMutex();
  logBootId = esp_random() & 0x7FFFFFFF;
  logEvent("Start mostu: %s, wezel N%u -> N%u, %.3f MHz SF%u BW%.1f, UART %lu Bd",
           BOARD_NAME, cfg.nodeId, cfg.targetId, cfg.loraFreq, cfg.loraSf, cfg.loraBw,
           (unsigned long)cfg.uartBaud);
  randomSeed(esp_random());
  DBG("[6] Kolejka TX: %s, wolny heap %u B\n", txQueue ? "OK" : "BLAD", (unsigned)ESP.getFreeHeap());

  SPI.begin(P_LORA_SCK, P_LORA_MISO, P_LORA_MOSI, P_LORA_CS);
  DBG("[7] Start radia SX1278...\n");
  bool radioInit = initRadio();
  DBG("[8] Radio: %s (kod %d)\n", radioInit ? "OK" : "BLAD", stats.radioState);
  if (!radioInit && oledAvailable) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BLAD RADIA SX1278!");
    display.printf("Kod: %d\n", stats.radioState);
    display.display();
    delay(2000);
  }
  // Nawet przy błędzie radia WWW działa — można zdiagnozować / zmienić ustawienia.

  DBG("[9] Start WiFi...\n");
  initWiFiAndNTP();
  DBG("[10] WiFi: %s, AP %s\n",
      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "brak STA",
      WiFi.softAPIP().toString().c_str());
  setupWebServer();
  DBG("[11] Serwer WWW OK\n");

  BaseType_t t1 = xTaskCreatePinnedToCore(uartRxTask,   "UART_RX", 4096, NULL, 2, NULL, 1);
  BaseType_t t2 = xTaskCreatePinnedToCore(loraTask,     "LORA_IO", 8192, NULL, 3, NULL, 1);
  BaseType_t t3 = xTaskCreatePinnedToCore(systemUiTask, "SYS_UI",  8192, NULL, 1, NULL, 0);
  DBG("[12] Watki: %s %s %s — start zakonczony, wolny heap %u B\n",
      t1 == pdPASS ? "OK" : "BLAD", t2 == pdPASS ? "OK" : "BLAD", t3 == pdPASS ? "OK" : "BLAD",
      (unsigned)ESP.getFreeHeap());
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
