/* 极简 MQTT 3.1.1 客户端 (ARM Linux, 零依赖)
 * 仅实现 CONNECT + PUBLISH + PINGREQ, 不走 TLS
 */
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <cstdint>
#include <cstring>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

class MqttClient
{
public:
    MqttClient() : sock_(-1), connected_(false), seq_(0) {}
    ~MqttClient() { disconnect(); }

    bool connect(const char* host, int port,
                 const char* client_id,
                 const char* username,
                 const char* password,
                 int keepalive = 60)
    {
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) return false;

        struct hostent* he = gethostbyname(host);
        if (!he) { close(sock_); sock_ = -1; return false; }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        memcpy(&addr.sin_addr, he->h_addr, he->h_length);

        struct timeval tv = {5, 0};  // 5秒超时
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (::connect(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            close(sock_); sock_ = -1; return false;
        }

        // 构造 CONNECT 包
        uint8_t buf[256];
        int pos = 0;

        // 固定头: CONNECT(0x10), 剩余长度编码在变长
        // 可变头: Protocol Name "MQTT" + Protocol Level 4 + Connect Flags + Keep Alive
        // 载荷: Client ID + Username + Password

        int clid_len  = strlen(client_id);
        int user_len  = username ? strlen(username) : 0;
        int pass_len  = password ? strlen(password) : 0;

        uint8_t flags = 0x02; // Clean Session
        if (username) flags |= 0x80;
        if (password) flags |= 0x40;

        int remaining = 10 + 2 + clid_len;
        if (username) remaining += 2 + user_len;
        if (password) remaining += 2 + pass_len;

        // 固定头
        buf[pos++] = 0x10;  // CONNECT
        pos += encodeLength(buf + pos, remaining);

        // 可变头 - Protocol Name "MQTT"
        buf[pos++] = 0; buf[pos++] = 4;
        buf[pos++] = 'M'; buf[pos++] = 'Q'; buf[pos++] = 'T'; buf[pos++] = 'T';
        buf[pos++] = 4;   // Protocol Level (3.1.1)
        buf[pos++] = flags;
        buf[pos++] = (keepalive >> 8) & 0xFF;
        buf[pos++] = keepalive & 0xFF;

        // 载荷
        pos = writeString(buf, pos, client_id, clid_len);
        if (username) pos = writeString(buf, pos, username, user_len);
        if (password) pos = writeString(buf, pos, password, pass_len);

        if (::send(sock_, buf, pos, 0) < 0)
        {
            close(sock_); sock_ = -1; return false;
        }

        // 读 CONNACK
        uint8_t resp[4];
        int n = recv(sock_, resp, 4, 0);
        if (n < 4 || resp[0] != 0x20 || resp[3] != 0x00)
        {
            close(sock_); sock_ = -1; return false;
        }

        connected_ = true;
        return true;
    }

    bool publish(const char* topic, const char* payload)
    {
        if (!connected_ || sock_ < 0) return false;

        int topic_len = strlen(topic);
        int payl_len  = strlen(payload);
        int remaining = 2 + topic_len + payl_len + 2;  // +2 for packet ID

        int total = 1 + 4 + remaining;  // conservative
        uint8_t* buf = new uint8_t[total];
        int pos = 0;

        buf[pos++] = 0x32;  // PUBLISH QoS 1 (at least once delivery)
        pos += encodeLength(buf + pos, remaining);

        pos = writeString(buf, pos, topic, topic_len);

        seq_++;
        buf[pos++] = (seq_ >> 8) & 0xFF;
        buf[pos++] = seq_ & 0xFF;

        memcpy(buf + pos, payload, payl_len);
        pos += payl_len;

        int ret = ::send(sock_, buf, pos, 0);
        delete[] buf;

        if (ret < 0) { connected_ = false; return false; }

        // 等待 PUBACK (QoS 1)
        uint8_t ack[4];
        int n = recv(sock_, ack, 4, 0);
        if (n != 4 || ack[0] != 0x40) { connected_ = false; return false; }

        return true;
    }

    bool ping()
    {
        if (!connected_ || sock_ < 0) return false;
        uint8_t buf[2] = { 0xC0, 0x00 };
        if (::send(sock_, buf, 2, 0) < 0) { connected_ = false; return false; }
        // PINGRESP
        uint8_t resp[2];
        int n = recv(sock_, resp, 2, 0);
        if (n != 2 || resp[0] != 0xD0) { connected_ = false; return false; }
        return true;
    }

    void disconnect()
    {
        if (sock_ >= 0)
        {
            if (connected_)
            {
                uint8_t buf[2] = { 0xE0, 0x00 };
                ::send(sock_, buf, 2, 0);
            }
            close(sock_);
            sock_ = -1;
            connected_ = false;
        }
    }

    bool isConnected() const { return connected_; }

private:
    int  sock_;
    bool connected_;
    uint16_t seq_;

    int encodeLength(uint8_t* buf, int len)
    {
        int pos = 0;
        do {
            uint8_t d = len % 128;
            len /= 128;
            if (len > 0) d |= 0x80;
            buf[pos++] = d;
        } while (len > 0);
        return pos;
    }

    int writeString(uint8_t* buf, int pos, const char* s, int len)
    {
        buf[pos++] = (len >> 8) & 0xFF;
        buf[pos++] = len & 0xFF;
        memcpy(buf + pos, s, len);
        return pos + len;
    }
};

#endif
