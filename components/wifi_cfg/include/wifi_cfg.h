#ifndef __WIFI_H
#define __WIFI_H

/**
 * @brief Smart Wi-Fi initialisation.
 *
 * Tries saved NVS credentials first (STA mode).  Falls back to SoftAP +
 * HTTP config portal if no credentials exist or STA connection times out.
 * In provisioning mode the function blocks until the user submits new
 * credentials via the web page, then reboots automatically.
 */
void wifi_init(void);

#endif /* __WIFI_H */
