#ifndef BUS_CONFIG_H
#define BUS_CONFIG_H

/* 沿用CARCAN13实发节拍；不是车辆几何或允许行驶速度。 */
#define CAN_RING_SIZE 16U
#define CAN_TX_TIMEOUT_MS 50U
#define CAN_TX_TURNAROUND_MS 2U
#define BUS_QUERY_INTERVAL_MS 5U
/* 等待匹配回包包含原50ms发送期限；不延长250ms反馈有效期。 */
#define BUS_REPLY_TIMEOUT_MS 100U
/* 上电后先保持STOP，给驱动器启动留出有界准备期；不改变后续查询/反馈期限。
 * 1000ms为保守候选值，待本车冷启动只读验收，不代表已经测得驱动器就绪。 */
#define BUS_STARTUP_SETTLE_MS 1000U
#define BUS_QUERIES_PER_BATCH 2U
#define BUS_STOP_RETRY_MS 10U
#define MOTOR_CHECK_BYTE 0x6BU
#define MOTOR_MAX_COMMAND_RPM 3000U
#define MOTOR_MAX_REPLY_RPM 5000U

#endif
