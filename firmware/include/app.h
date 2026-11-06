#ifndef APP_H
#define APP_H
#include <stdint.h>
enum {
    APP_FAULT_SENSOR=1u, APP_FAULT_OVERVOLTAGE=2u,
    APP_FAULT_BUS=4u, APP_FAULT_CONTROL=8u,
    APP_FAULT_COMMUNICATION=16u, APP_FAULT_PLATFORM=32u,
    APP_FAULT_STALE=64u, APP_FAULT_CONFIG=128u
};
typedef struct {
    float v1,v2,vbus,reference,duty;
    uint32_t fault, armed, sample_count;
} AppStatus;
void app_init(void);
void app_poll(void);
/* One complete, NUL-terminated ASCII command without newline. */
void app_command(const char *line);
void app_status(AppStatus *status);
#endif
