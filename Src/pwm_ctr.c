#include "pwm_ctr.h"
#include "stdio.h"
#define CRT_CNT 	20
const float array[][6] = 
{
	{ 0.175, 0.125, 0.0, 50.0  , TIM1_PERIOD_MAX,           TIM1_PERIOD_85KHz    }, //@2.5A 50V 
	{ 0.235, 0.240, 0.0, 63.0  , TIM1_PERIOD_85KHz + 2,     TIM1_PERIOD_MIN - 2  }, //@3.5A 63V
	{ 0.045, 0.025, 0.0, 82.0  , TIM1_PERIOD_MIN - CRT_CNT, TIM1_PERIOD_MIN      }, //@4.5A 80V 5A sure overload!
};
#define PID_P_INDEX 			0
#define PID_I_INDEX 			1
#define PID_D_INDEX 			2
#define PID_VOL_INDEX 			3
#define PID_PERIOD_UP_INDEX 	4
#define PID_PERIOD_DOWN_INDEX 	5

#define CUR_2A5_CNT 	175
#define CUR_3A5_CNT 	183

typedef enum
{
	CUR_2A5 = 0,
	CUR_3A5,
	CUR_3A5_UP
}CUR_STA_t;
const float ( *p4a )[6];

static uint8_t overload_flag = 0;
static void pwm_pid_setting(inc_pid_controller_t pMyPID, const float (*p)[6])
{
	pMyPID = inc_pid_controller_create( *( *p + PID_P_INDEX ), *( *p + PID_I_INDEX ), *( *p + PID_D_INDEX ), 30 );
	pMyPID->controller.target = *( *p + PID_VOL_INDEX );
	pMyPID->maximum = *( *p + PID_PERIOD_DOWN_INDEX ); //TIM1_PERIOD_MIN;	//TIM1_PERIOD_MIN (768UL) --> 83.3KHz 640/0.833 = 768,作用：加大功率！
	pMyPID->minimum = *( *p + PID_PERIOD_UP_INDEX );  //TIM1_PERIOD_MAX;	//TIM1_PERIOD_MAX (727UL) //88KHz 640/0.88 = 727，作用，减小功率！
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
		case 0: //首次进入
			
			if( ovcnt == 1 ) //first overload
			{
				mode = 1;
				xTimeLast = xTimeNow;
				overload_flag = 1;
			}
			break;
			
		case 1:
			//弟二次进入，大于0。12秒钟有2次以上，则为有效，跳到模式2
			if( xTimeBetween > 30 && ovcnt >= 4 ) // occour 2 times ov in 0.1 second 
			{
				mode = 2;
				xTimeLast = xTimeNow;
			}
			//第二次进入时，1秒钟没发生过2次以上中断，则为误触发，模式为3，准备复位
			else if( xTimeBetween > 1000) 			// in 1 second still not occour ov, reset flag.
			{
				mode = 3;
			}
			break;
			
		case 2:
			//2秒内设定pwm最大功率输出，即保持2秒钟的最大功率输出
			if( xTimeBetween < 1000 ) 				//keep 2second
			{
				pwm_set_84KHz();
			}
			else if( xTimeBetween < 600000 )
			{
				if(htim1.State == HAL_TIM_STATE_READY)
				{
					htim1.Instance->ARR = TIM1_PERIOD_MIN - CRT_CNT;
					htim1.Instance->CCR1 = htim1.Instance->ARR / 2;
				}
			}
			//2秒后，进到模式3，准备复位
			else 									//after keep 2 seconds
			{
				mode = 3;
			}
			break;
			
		case 3:
			overload_flag = 0;
			mode = 0;
			ovcnt = 0;
			*ovflag = 0;
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
