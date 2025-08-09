#ifndef MQTTINFO_H
#define MQTTINFO_H

#include <string>

struct MqttInfo
{
    std::string server;
    int port;
    std::string topic;
    std::string clientName;

    MqttInfo(const std::string &s, int p, const std::string &t, const std::string &c = "randomClient")
        : server(s), port(p), topic(t), clientName(c)
    {
    }
};

#endif
