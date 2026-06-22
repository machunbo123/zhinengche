#ifndef CODE_KEY_APP_H_
#define CODE_KEY_APP_H_

#include "zf_common_headfile.h"

void key_app_init(void);
void key_app_task(void);

extern uint8_t choice_para;//按键调参选择的参数，默认为速度

#endif /* CODE_KEY_APP_H_ */
