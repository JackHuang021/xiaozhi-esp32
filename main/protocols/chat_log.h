/**
 * @file chat_log.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <string>
#include <vector>

using std::string;
using std::vector;

struct chat_msg {
    string role;
    string time;
    string text; 
};

class ChatLog {
public:
    ChatLog(const string &dir = "/sdcard/chat/");
    bool saveMessage(const char *role, const char *content);

private:
    string base_dir_;
    std::vector<string> logs_;
    int file_index_;
    int line_index_;

    std::string getToadyFile();

};


