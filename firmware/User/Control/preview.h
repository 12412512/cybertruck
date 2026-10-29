#ifndef PREVIEW_H
#define PREVIEW_H

#include <stdint.h>

/* 主机模型专用。Keil不定义EXEC_PREVIEW，因此不提供这些入口。 */
#ifdef EXEC_PREVIEW
#if defined(_WIN32)
#define PREVIEW_API __declspec(dllexport)
#else
#define PREVIEW_API
#endif
PREVIEW_API void Preview_Init(uint64_t boot);
PREVIEW_API int Preview_Load(uint32_t zone, const uint8_t *goals, uint32_t count);
PREVIEW_API void Preview_Push(const uint8_t *data, uint32_t size);
PREVIEW_API uint16_t Preview_Read(uint8_t *data);
PREVIEW_API void Preview_Tick(uint32_t tick, const int64_t *counts, const int16_t *rpm,
                              int16_t yaw, int16_t gyro, uint8_t key, uint8_t select);
PREVIEW_API void Preview_Target(int16_t *rpm);
PREVIEW_API void Preview_Fault(uint32_t reason, uint32_t drop);
#endif
#endif
