#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Init time service (SNTP). Safe to call multiple times.
 * Call this when network is available (WiFi/ETH up).
 */
void gw_time_init(void);

/**
 * @brief Returns true if SNTP time is synced (epoch looks valid).
 */
bool gw_time_is_synced(void);

/**
 * @brief Get current timestamp in seconds.
 * - If time synced: returns Unix epoch seconds (e.g. 1730970000)
 * - If not synced: returns fallback seconds (uptime seconds by default)
 */
uint32_t gw_time_unix(void);

/**
 * @brief Option: set timezone string (POSIX TZ). Example for Vietnam: "UTC-7"
 * Not required for unix time, only for localtime formatting/logging.
 */
void gw_time_set_tz(const char *tz);

#ifdef __cplusplus
}
#endif
