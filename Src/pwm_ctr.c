#include "pwm_ctr.h"
#include "stdio.h"
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

static uint8_t overload_flag = 0;
static void pwm_pid_setting(inc_pid_controller_t pMyPID, const float (*p)[4])
{
	pMyPID = inc_pid_controller_create( *( *p + PID_P_INDEX ), *( *p + PID_I_INDEX ), *( *p + PID_D_INDEX ), 30 );
	pMyPID->controller.target = *( *p + PID_VOL_INDEX );
	pMyPID->maximum = TIM1_PERIOD_MIN;	//TIM1_PERIOD_MIN (768UL) --> 83.3KHz 640/0.833 = 768,作用：加大功率！
	pMyPID->minimum = TIM1_PERIOD_MAX;	//TIM1_PERIOD_MAX (727UL) //88KHz 640/0.88 = 727，作用，减小功率！
	pMyPID->controller.enable = 1;
}
static void ov_judge(uint16_t *ovflag) //when occour overload.
{
	static uint16_t ovcnt = 0;
	static uint16_t mode = 0;
	static TickType_t xTimeLast = 0;
	TickType_t xTimeNow, xTimeBetween;
	
	xTimeNow = xTaskGetTickCount();
	xTimeBetween = xTimeNow - xTimeLast;
	
	
	if(*ovflag != 0) ovcnt++;
	
	switch(mode)
	{
		case 0:
			if( ovcnt == 1 ) //first overload
			{
				mode = 1;
				xTimeLast = xTimeNow;
				overload_flag = 1;
				//printf("case0if\r\n");
			}
			break;
		case 1:
			
			if( xTimeBetween > 120 && ovcnt > 5 ) // occour 2 times ov in 0.1 second 
			{
				mode = 2;
				xTimeLast = xTimeNow;
				//printf("case1 if > 120\r\n");
			}
			else if( xTimeBetween > 1000) 			// in 1 second still not occour ov, reset flag.
			{
				mode = 3;
				//printf("case1 if > 1000\r\n");
				
			}
			break;
		case 2:
			if( xTimeBetween < 2000) 				//keep 2second
			{
				pwm_set_83KHz();
				//printf("xTime = %d\r\n", xTimeNow);
			}
			else 									//after keep 2 seconds
			{
				mode = 3;
				//printf("case 2 mode = 3;\r\n");
			}
			break;
		case 3:
			overload_flag = 0;
			mode = 0;
			ovcnt = 0;
			*ovflag = 0;
		//printf("Between = %d\r\n", xTimeBetween);
			break;
		default:
			break;
	}
	
}
void pwm_ctr_handler(vol_cur_t *pDat, inc_pid_controller_t pMyPID )
{
	static CUR_STA_t last_CUR_STA = CUR_2A5;
	static uint16_t ov_cnt = 0;
	p4a = array;
	CUR_STA_t this_CUR_STA;
	
	uint16_t overload = pDat->data.overload;
	uint16_t current = pDat->data.cur * 330 / 4096 ; //电压值放大了100倍 比如2.5A=1.84v = 184;
	float vol = (float)pDat->data.vol / ADC_REF;
	
	if(overload == 1) //发生过载事件
	{
		ov_cnt++;
	}
	else if (overload_flag == 0)	//未发和过载
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
		last_CUR_STA = this_CUR_STA;
		if( pMyPID->controller.update( pMyPID, vol ) == RT_EOK ) //成功读取调用更新
		{
			pwm_set_update(pMyPID->controller.output);
		}
	}
	ov_judge(&ov_cnt);
	
}
