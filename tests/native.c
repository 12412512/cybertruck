/* 在主机测试正式C协议，不链接HAL，不替代实机UART验证。 */
#include "comm.h"
#include <string.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

EXPORT void Native_Init(uint64_t boot, uint32_t now)
{
    Comm_Init(boot, now);
}

EXPORT void Native_Tick(uint32_t now)
{
    Comm_Tick(now, 0);
}

EXPORT void Native_Fault(uint32_t reason)
{
    Comm_Fault(reason);
}

EXPORT void Native_Push(const uint8_t *data, uint16_t size, uint32_t received, uint32_t now)
{
    unsigned index;
    for (index = 0; index < size; index++)
    {
        Comm_Feed(data[index], received, now);
    }
}

EXPORT uint16_t Native_Read(uint8_t *data)
{
    return Comm_Read(data);
}

EXPORT uint16_t Native_RoundTrip(const uint8_t *data, uint16_t size, uint8_t *out)
{
    unsigned index;
    FrameRx receiver;
    Frame frame;
    memset(&receiver, 0, sizeof(receiver));
    for (index = 0; index < size; index++)
    {
        if (Frame_Push(&receiver, data[index], 0, &frame))
        {
            return Frame_Encode(&frame, out);
        }
    }
    return 0;
}
