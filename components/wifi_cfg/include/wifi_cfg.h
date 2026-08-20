#ifndef __WIFI_H
#define __WIFI_H

#include <stdbool.h>

/**
 * @brief Smart Wi-Fi initialisation (non-blocking, no reboot).
 *
 * Tries saved NVS credentials first.  Falls back to SoftAP + HTTP config
 * portal if no credentials exist or STA connection times out.
 * After the user submits credentials, the portal stays up until POST /exit,
 * then the AP is stopped and STA connects without rebooting.
 */
void wifi_init(void);

/** @brief SoftAP 配网页是否正在运行 */
bool wifi_is_provisioning(void);

#endif /* __WIFI_H */
