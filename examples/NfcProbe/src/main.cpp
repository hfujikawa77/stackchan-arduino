/*
 * NfcProbe — Stack-chan(CoreS3)内蔵 NFC(ST25R3916) 検証スケッチ
 *
 * Issue #25 / Step A:
 *   HermesLLM へ NFC 機能を統合する前に、内蔵リーダーが
 *   M5UnitUnified(M5Unit-NFC)で正しく読めるかを単体で確認する。
 *
 * 対応カード(モード切替式):
 *   - FeliCa(NFC-F) … Suica/PASMO/ICOCA 等の交通系IC → IDm + 残高(認証不要領域)
 *   - NFC-A(Type2/MIFARE) … NTAG215 等 → UID + NDEF(受付番号/氏名/会社名 など)
 *       例: AWS Summit Japan 2026 パス = NTAG215
 *
 *   仕組み(自動デュアル待ち受け): A/F はRF技術が異なり、configureNFCMode() だけのランタイム
 *   切替では FeliCa の受信が壊れる(polling は通るが requestService が無応答)。そこで切替時は
 *   unit.begin() で完全再初期化し、F/A を一定時間ずつ自動で行き来する。各モードは「単一モード」
 *   として安定動作するので、タップ不要でどちらのカードでも反応する。
 *   ※ FeliCa の IDm は polling() で取得(detect は RequestResponse 無応答でカードを捨てるため)。
 *
 * 使い方:
 *   `pio run -e m5stack-cores3 -t upload && pio device monitor`
 *   Suica/PASMO でも AWS パスでも、かざせば種別に応じて自動で表示される(切替操作不要)。
 *
 * ハード前提(実機で裏取り済み):
 *   - 内蔵 ST25R3916 は内部I2C(M5.In_I2C)、I2Cアドレス 0x50。
 *   - HermesLLM の MAVLink は Port A の UART なので I2C 競合なし。
 *   - 交通系ICの残高/履歴は CJRC 規格の「認証不要サービス領域」にあり鍵なしで読める。
 */
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedNFC.h>
#include <M5Utility.h>
#include <vector>

#define NFC_BUS  M5.In_I2C    // 内蔵NFCは内部I2C(0x50)。切り分け時は M5.Ex_I2C へ。

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNFC     unit{};   // 内蔵 ST25R3916(I2C, 0x50)
m5::nfc::NFCLayerA    nfc_a{unit};
m5::nfc::NFCLayerF    nfc_f{unit};

// 交通系IC(CJRC規格準拠)
constexpr uint16_t jtic_system_code[] = {
    0x0003,  // Suica / PASMO / ICOCA / PiTaPa / TOICA / manaca / SUGOCA / nimoca / Kitaca ...
    0x80DE,  // IruCa
};
constexpr uint16_t service_balance = 0x008B;  // 認証不要: 残高

void banner(const char* msg, uint16_t color = TFT_WHITE) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(color, TFT_BLACK);
    lcd.setCursor(0, 0);
    lcd.println(msg);
}

// 現在のモード(起動時 FeliCa)
m5::nfc::NFC g_mode = m5::nfc::NFC::F;

// モードを適用する。configureNFCMode() だけのランタイム切替では FeliCa の受信が壊れる
// (polling は通るが requestService/RequestResponse が無応答になる)ため、
// unit.config(mode) + unit.begin() で「完全再初期化」する。これは初回 begin と同じ
// 経路なので各モードが単一モードとして確実に動作する。自動デュアルでも数十msで済む。
bool apply_mode(m5::nfc::NFC mode) {
    auto cfg = unit.config();
    cfg.mode = mode;
    unit.config(cfg);
    bool ok = unit.begin();
    if (!ok) M5.Log.printf("[mode] %s begin FAILED\n", mode == m5::nfc::NFC::F ? "FeliCa" : "NFC-A");
    return ok;
}

// ── FeliCa(交通系IC): IDm + 残高 ───────────────────────────────────────────
//   IDm は polling() で確実に取得する。detect() は内部の RequestResponse が無応答だと
//   カードを丸ごと捨てるため、RF が弱いと IDm すら出ない。polling は WUPF 応答(=IDm)を
//   そのまま返すので、ゴール「IDを表示」を安定して満たせる。残高は best-effort。
bool poll_felica() {
    using namespace m5::nfc;
    using namespace m5::nfc::f;

    PICC picc{};
    // 0xFFFF = ワイルドカード system code(任意の FeliCa を検出)
    if (!nfc_f.polling(picc, 0xFFFF, RequestCode::None, TimeSlot::Slot1)) {
        return false;
    }
    M5.Speaker.tone(3000, 10);

    // polling() は type を埋めないが、requestService() は type==FeliCaStandard を要求する。
    // 交通系は FeliCaStandard なので明示セット(detect も Unknown 時はこれにフォールバック)。
    picc.type = Type::FeliCaStandard;

    // 残高: activate(=対象IDmを選択)→requestService→read16。
    long yen = -1;
    if (nfc_f.activate(picc)) {
        uint16_t key_version = 0;
        if (nfc_f.requestService(key_version, service_balance) && key_version != 0xFFFF) {
            uint8_t buf[16] = {0};
            if (nfc_f.read16(buf, block_t(0), service_balance)) {
                yen = ((uint16_t)buf[12] << 8) | buf[11];  // LE
            }
        }
        nfc_f.deactivate();
    }
    M5.Log.printf("FeliCa IDm:%s balance:%ld\n", picc.idmAsString().c_str(), yen);

    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    lcd.setCursor(4, 6);
    lcd.println("FeliCa");
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setCursor(4, 30);
    lcd.printf("IDm: %s\n", picc.idmAsString().c_str());
    lcd.setTextColor(yen >= 0 ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(4, 64);
    if (yen >= 0) {
        lcd.setTextSize(2);
        lcd.printf("%ld yen", yen);
        lcd.setTextSize(1);
    } else {
        lcd.println("(no balance)");
    }

    nfc_f.deactivate();
    return true;
}

// ── NFC-A(Type2/MIFARE): UID + NDEF ────────────────────────────────────────
bool poll_nfca() {
    using namespace m5::nfc;
    using namespace m5::nfc::a;
    using namespace m5::nfc::ndef;

    PICC picc{};
    if (!nfc_a.detect(picc)) return false;
    if (!(nfc_a.identify(picc) && nfc_a.reactivate(picc))) {
        nfc_a.deactivate();
        return false;
    }

    M5.Speaker.tone(3000, 10);
    M5.Log.printf("PICC UID:%s type:%s area:%u/%u\n",
                  picc.uidAsString().c_str(), picc.typeAsString().c_str(),
                  picc.userAreaSize(), picc.totalSize());

    lcd.fillScreen(TFT_BLACK);
    lcd.setCursor(4, 4);
    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    lcd.printf("UID: %s\n", picc.uidAsString().c_str());
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.printf("type: %s\n\n", picc.typeAsString().c_str());

    if (picc.supportsNDEF()) {
        bool valid = false;
        TLV msg{};
        if (nfc_a.ndefIsValidFormat(valid) && valid && nfc_a.ndefRead(msg) && msg.isMessageTLV()) {
            int idx = 0;
            for (auto&& r : msg.records()) {
                // 生ペイロードを hex でダンプ(未知フォーマット解析用)
                M5.Log.printf("  NDEF[%d] hex:", idx);
                const uint8_t* p = r.payload();
                for (uint32_t i = 0; i < r.payloadSize(); ++i) M5.Log.printf(" %02X", p[i]);
                M5.Log.printf("\n");

                if (r.tnf() == TNF::Wellknown) {
                    const auto s = r.payloadAsString();
                    M5.Log.printf("  NDEF[%d] T:%s [%s]\n", idx, r.type(), s.c_str());
                    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
                    lcd.printf("%s\n", s.c_str());
                } else {
                    M5.Log.printf("  NDEF[%d] TNF:%u T:%s sz:%u\n", idx, r.tnf(), r.type(), r.payloadSize());
                    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
                    lcd.printf("[%s %ub]\n", r.type(), r.payloadSize());
                }
                ++idx;
            }
        } else {
            M5.Log.printf("  NDEF: not formatted / empty\n");
            lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
            lcd.println("(no NDEF data)");
        }
    } else {
        M5.Log.printf("  NDEF not supported by this PICC\n");
        lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
        lcd.println("(NDEF not supported)");
    }

    nfc_a.deactivate();
    return true;
}
}  // namespace

void setup() {
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    if (lcd.height() > lcd.width()) lcd.setRotation(1);
    lcd.setFont(&fonts::efontJA_16);  // NDEF の氏名/会社名(日本語)を表示するため
    lcd.setTextSize(1);

    banner("NfcProbe: initializing NFC...");
    M5.Log.printf("NfcProbe: toggle-mode (FeliCa <-> NFC-A) bus=internal-I2C(0x50)\n");

    // begin 内の configureNFCMode(_cfg.mode) で初期モードが決まる。起動時は FeliCa。
    {
        auto cfg = unit.config();
        cfg.mode = g_mode;  // = FeliCa
        unit.config(cfg);
    }

    bool unit_ready = Units.add(unit, NFC_BUS) && Units.begin();
    if (!unit_ready) {
        banner("NFC begin FAILED\n\nST25R3916 not found on\ninternal I2C(0x50).\nTry NFC_BUS = M5.Ex_I2C", TFT_RED);
        M5.Log.printf("Failed to begin NFC. Check I2C bus / address(0x50) / wiring.\n");
        return;  // 無限停止せずログを残してループ継続(切り分けしやすく)
    }

    M5.Log.printf("M5UnitUnified initialized\n%s\n", Units.debugInfo().c_str());
    banner("Scanning... Tap Suica/PASMO\nor AWS pass (auto-detect)");
}

void loop() {
    M5.update();
    Units.update();

    // 自動デュアル待ち受け: F/A を一定時間ずつ自動で切り替える。
    // 切替は apply_mode()=完全再初期化(unit.begin())なので、各モードは単一モードとして
    // 確実に動作する。タップ不要で Suica でも AWS パスでもかざせば反応する。
    static constexpr uint32_t DWELL_MS = 800;  // 各モードの待ち受け時間
    static int8_t   mode        = -1;          // -1:未初期化 / 0:FeliCa / 1:NFC-A
    static uint32_t dwell_until = 0;

    uint32_t now = millis();
    if (mode < 0 || now >= dwell_until) {
        mode  = (mode == 0) ? 1 : 0;  // F⇄A をトグル(初回は F)
        g_mode = (mode == 0) ? m5::nfc::NFC::F : m5::nfc::NFC::A;
        apply_mode(g_mode);
        dwell_until = now + DWELL_MS;
    }

    bool read = (mode == 0) ? poll_felica() : poll_nfca();
    if (read) {
        delay(1200);                          // 読めたら結果を保持
        dwell_until = millis() + DWELL_MS;    // 直後は同じモードを少し継続
    } else {
        delay(20);
    }
}
