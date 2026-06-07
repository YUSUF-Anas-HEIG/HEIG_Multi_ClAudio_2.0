#ifndef APP_STATE_H_
#define APP_STATE_H_
 
#include <stdbool.h>
 

int app_state_init(void);
 

void app_state_wait_for_start(void);

void app_state_notify_connected(void);
 

void app_state_notify_scan_timeout(void);
 

bool app_state_disconnect_requested(void);
 
#endif 