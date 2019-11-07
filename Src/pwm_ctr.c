#include "pwm_ctr.h"
const float array[][4] = 
{
	{ 0.175, 0.125, 0.0, 50.0 }, //@2.5A 50V 
	{ 0.235, 0.240, 0.0, 63.0 }, //@3.5A 63V
	{ 0.045, 0.025, 0.0, 80.0 }, //@4.5A 80V 5A sure overload!
};
#define PID_P_INDEX 	0
#define PID_I_INDEX 	1
#define PID_D_INDEX 	2
#define PID_VOL_INDEX 	3

#define CUR_2A5_CNT 	184
#define CUR_3A5_CNT 	191

typedef enum
{
	CUR_2A5 = 0,
	CUR_3A5,
	CUR_3A5_UP
}CUR_STA_t;
const float ( *p4a )[4];
uint16_t ov_cnt = 0;
void pwm_ctr_handler( vol_cur_t *pDat, inc_pid_controller_t *pMyPID, );
void pwm_pid_setting(inc_pid_controller_t *pMyPID, const float (*p)[4])
{
	pMyPID = inc_pid_controller_create( *( *p + PID_P_INDEX ), *( *p + PID_I_INDEX ), *( *p + PID_D_INDEX ), 30 );
	pMyPID->controller.target = *( *p + PID_VOL_INDEX );
	pMyPID->maximum = TIM1_PERIOD_MIN;//TIM1_PERIOD_MIN (768UL) --> 83.3KHz 640/0.833 = 768,作用：加大功率！
	pMyPID->minimum = TIM1_PERIOD_MAX;//TIM1_PERIOD_MAX (727UL) //88KHz 640/0.88 = 727，作用，减小功率！
	pMyPID->controller.enable = 1;
}
void pwm_ctr_handler(vol_cur_t *pDat, inc_pid_controller_t *pMyPID )
{
	uint16_t overload = pDat->data.overload;
	uint16_t current = pDat->data.cur * 330 / 4096 ; //电压值放大了100倍 比如2.5A=1.84v = 184;
	float vol = (float)pDat->data.vol / ADC_REF;
	p4a = array;
	static CUR_STA_t last_CUR_STA = CUR_2A5;
	CUR_STA_t this_CUR_STA;
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
		if( current < CUR_2A5_CNT ) //@2.5A
		{
			p4a = p4a + CUR_2A5;
			this_CUR_STA = CUR_2A5;
		}
		else if( current < CUR_3A5_CNT ) //@3.5A
		{
			p4a = p4a + CUR_3A5;
			this_CUR_STA = CUR_2A5;
		}
		else //>3.5A
		{
			p4a = p4a + CUR_3A5_UP;
			this_CUR_STA = CUR_2A5;
		}
		
		if( last_CUR_STA != this_CUR_STA )
		{
			pwm_pid_setting( pMyPID, p4a );
		}
		
		if( pMyPID->controller.update( pMyPID, vol ) == RT_EOK ) //成功读取调用更新
		{
			pwm_set_update(pMyPID->controller.output);
		}
	}
}
