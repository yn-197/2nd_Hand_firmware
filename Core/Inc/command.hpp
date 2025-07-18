#pragma once
#include <string>
#include <functional>
#include <vector>
#include <cstdint>


class CommandHandler {
public:
    using Callback = std::function<void(char mode, int value1, int value2)>;

    CommandHandler();
    void setCallback(Callback cb);
    void onUartReceive(const uint8_t* data, size_t len);

private:
    Callback callback_;
    std::vector<uint8_t> buffer_;
    void parseCommand(const std::string& cmd);
};

#ifdef __cplusplus
extern "C" {
#endif

void ProcessCommand(char mode, int value1, int value2);

#ifdef __cplusplus
}
#endif