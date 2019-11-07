#include "pwm_ctr.h"
const float array[][4] = 
{
	{ 0.175, 0.125, 0.0, 50.0 }, //@2.5A 50V 
	{ 0.235, 0.240, 0.0, 63.0 }, //@3.5A 63V
	{ 0.045, 0.025, 0.0, 80.0 }, //@4.5A 80V 5A sure overload!
};
const float ( *p4a )[4];
uint16_t ov_cnt = 0;
void pwm_ctr_handler(vol_cur_t *pDat);
void pwm_ctr_handler(vol_cur_t *pDat)
{
	uint16_t overload = pDat->data.overload;
	uint16_t current = pDat->data.cur * 330 / 4096 ; //电压值放大了100倍 比如2.5A=1.84v = 184;
	float vol = (float)pDat->data.vol / ADC_REF;
	if(overload == 1) //发生过载事件
	{
		ov_cnt++;
		if(ov_cnt == 1)
		{
			pwm_set_84KHz();
		}
		else if(ov_cnt == 2)
		{
			ov_cnt = 0;
			pwm_set_83KHz();
		}
		vTaskDelay( pdMS_TO_TICKS(2000) ); //保持2秒钟
	}
	else //未发和过载
	{
		if( current < 184 ) //@2.5A
		{
		}
		else if( current < 191 ) //@3.5A
		{
		}
		else //>3.5A
		{
		}
	}
}
