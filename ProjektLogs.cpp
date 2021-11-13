#include "ProjektLogs.h"

std::shared_ptr<spdlog::logger> logger = spdlog::basic_logger_mt("basic_logger", "logs/basic-log.txt", true);
