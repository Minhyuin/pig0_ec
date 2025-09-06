#include "nav.h"

nav_req_t nav_req_data = {0};
nav_ctrl_t nav_ctrl_data = {0};

uint8_t parse_nav_data(uint8_t *buf, uint16_t len) {return 0;}
void send_nav_request(void *data) {}
nav_ctrl_t *get_nav_ctrl_data_ptr(void) { return &nav_ctrl_data; }
