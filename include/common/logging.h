#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <chrono>
inline uint32_t millis() {
    using namespace std::chrono;
    static auto start = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}
#endif

// Log Levels
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

// Default log level if not set
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#if (LOG_LEVEL >= LOG_LEVEL_ERROR)
#define LOG_ERROR(tag, fmt, ...) printf("[%8lu][E][%s] " fmt "\n", (unsigned long)millis(), tag, ##__VA_ARGS__)
#else
#define LOG_ERROR(tag, fmt, ...) ((void)0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_WARN)
#define LOG_WARN(tag, fmt, ...)  printf("[%8lu][W][%s] " fmt "\n", (unsigned long)millis(), tag, ##__VA_ARGS__)
#else
#define LOG_WARN(tag, fmt, ...)  ((void)0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_INFO)
#define LOG_INFO(tag, fmt, ...)  printf("[%8lu][I][%s] " fmt "\n", (unsigned long)millis(), tag, ##__VA_ARGS__)
#else
#define LOG_INFO(tag, fmt, ...)  ((void)0)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
#define LOG_DEBUG(tag, fmt, ...) printf("[%8lu][D][%s] " fmt "\n", (unsigned long)millis(), tag, ##__VA_ARGS__)
#else
#define LOG_DEBUG(tag, fmt, ...) ((void)0)
#endif
