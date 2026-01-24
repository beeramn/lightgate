#include "sdkconfig.h"
#if !(CONFIG_ROLE_TX ^ CONFIG_ROLE_RX)
#error "Select exactly one device role"
#endif

void app_role_start(void);

void app_main(void) {
    app_role_start();
}
