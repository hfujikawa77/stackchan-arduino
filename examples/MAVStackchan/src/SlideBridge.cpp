#include "SlideBridge.h"

#include <mbedtls/sha1.h>
#include <string.h>

namespace {

constexpr uint32_t kIdleTimeoutMs = 90 * 1000;
constexpr size_t kMaxPayload = 440;

// SHA-1 の結果（20 バイト）を Base64 にする（出力は NUL 終端を含めて 29 バイト）
void base64Encode20(const uint8_t* in, char* out) {
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t j = 0;
    for (size_t i = 0; i < 20; i += 3) {
        const uint32_t a = in[i];
        const uint32_t b = (i + 1 < 20) ? in[i + 1] : 0;
        const uint32_t c = (i + 2 < 20) ? in[i + 2] : 0;
        const uint32_t t = (a << 16) | (b << 8) | c;
        out[j++] = tbl[(t >> 18) & 0x3F];
        out[j++] = tbl[(t >> 12) & 0x3F];
        out[j++] = (i + 1 < 20) ? tbl[(t >> 6) & 0x3F] : '=';
        out[j++] = (i + 2 < 20) ? tbl[t & 0x3F] : '=';
    }
    out[j] = '\0';
}

// Sec-WebSocket-Accept を作る（RFC 6455）
String acceptKey(const String& key) {
    static const char kGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    const String src = key + kGuid;
    uint8_t sha[20];
    mbedtls_sha1_ret(reinterpret_cast<const unsigned char*>(src.c_str()), src.length(), sha);
    char b64[32];
    base64Encode20(sha, b64);
    return String(b64);
}

}  // namespace

void SlideBridge::begin(uint16_t port) {
    (void)port;  // WiFiServer のポートはコンストラクタの 80 を使う
    server_.begin();
}

bool SlideBridge::connected() {
    return active_ && client_.connected();
}

void SlideBridge::stop() {
    if (client_) client_.stop();
    active_ = false;
}

bool SlideBridge::readExact(uint8_t* out, size_t len, uint32_t timeout_ms) {
    const uint32_t start = millis();
    size_t got = 0;
    while (got < len) {
        if (!client_.connected()) return false;
        const int n = client_.read(out + got, len - got);
        if (n > 0) {
            got += n;
            continue;
        }
        if (millis() - start > timeout_ms) return false;
        delay(1);
    }
    return true;
}

bool SlideBridge::handshake(WiFiClient& client) {
    client.setTimeout(500);
    const String request = client.readStringUntil('\n');
    String req = request;
    req.trim();
    const bool is_servo_ws = req.startsWith("GET /servo-ws");

    String key;
    while (client.connected()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) break;
        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("sec-websocket-key:")) key = line.substring(line.indexOf(':') + 1);
    }
    key.trim();

    if (!is_servo_ws || key.isEmpty()) {
        client.print("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        client.stop();
        return false;
    }
    client.print("HTTP/1.1 101 Switching Protocols\r\n");
    client.print("Upgrade: websocket\r\n");
    client.print("Connection: Upgrade\r\n");
    client.print("Sec-WebSocket-Accept: ");
    client.print(acceptKey(key));
    client.print("\r\n\r\n");
    return true;
}

bool SlideBridge::sendText(const char* text) {
    if (!connected()) return false;
    const size_t n = strlen(text);
    if (n > 125) return false;
    uint8_t hdr[2] = {0x81, static_cast<uint8_t>(n)};  // サーバからのフレームはマスクしない
    client_.write(hdr, 2);
    client_.write(reinterpret_cast<const uint8_t*>(text), n);
    return true;
}

void SlideBridge::tick(uint32_t now_ms, Handler handler, void* ctx) {
    if (!active_) {
        WiFiClient incoming = server_.available();
        if (!incoming) return;
        if (handshake(incoming)) {
            client_ = incoming;
            active_ = true;
            last_ms_ = now_ms;
        }
        return;
    }
    if (!client_.connected() || (now_ms - last_ms_) > kIdleTimeoutMs) {
        stop();
        return;
    }

    while (client_.available() >= 2) {
        uint8_t hdr[2];
        if (!readExact(hdr, 2)) return;
        const uint8_t opcode = hdr[0] & 0x0F;
        const bool masked = (hdr[1] & 0x80) != 0;
        size_t len = hdr[1] & 0x7F;
        if (len == 126) {
            uint8_t ext[2];
            if (!readExact(ext, 2)) return;
            len = (static_cast<size_t>(ext[0]) << 8) | ext[1];
        } else if (len == 127) {
            stop();
            return;
        }
        if (!masked || len > kMaxPayload) {
            stop();
            return;
        }
        uint8_t mask[4];
        if (!readExact(mask, 4)) return;
        static char payload[kMaxPayload + 1];  // static: 大きめのバッファをループのスタックに置かない
        if (!readExact(reinterpret_cast<uint8_t*>(payload), len)) return;
        for (size_t i = 0; i < len; ++i) payload[i] ^= mask[i & 3];
        payload[len] = '\0';

        if (opcode == 0x8) {  // close
            stop();
            return;
        }
        if (opcode == 0x9) {  // ping → pong（同じ内容）
            uint8_t ph[2] = {0x8A, static_cast<uint8_t>(len)};
            client_.write(ph, 2);
            client_.write(reinterpret_cast<const uint8_t*>(payload), len);
        }
        if (opcode == 0x1) {
            last_ms_ = now_ms;
            if (payload[0] == '{' && handler) handler(payload, ctx);
        }
    }
}
