/**
 * @file chat_log.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "chat_log.h"
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>

ChatLog::ChatLog(const string &dir) : base_dir_(dir)
{
    file_index_ = -1;
    line_index_ = 0;
}

std::string ChatLog::getToadyFile()
{

}
