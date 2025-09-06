#ifndef UI_CTRL_H_
#define UI_CTRL_H_

void UI_Init(void);
void UI_Send(void);
void UI_Update(void);

#include "APP/behaviour.h"
void update_Ctag_ui_CBK(chassis_tag_t tag);
void update_Gtag_ui_CBK(gimbal_tag_t tag);
void update_Stag_ui_CBK(shooter_tag_t tag);

#endif // !UI_CTRL_H_
