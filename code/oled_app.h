#ifndef CODE_OLED_APP_H_
#define CODE_OLED_APP_H_


#include "zf_common_headfile.h"

void ips_proc(void);
void ips_init(void);
void ips_sprintf(uint8_t y,uint8_t x,char* format,...);

#endif /* CODE_OLED_APP_H_ */
