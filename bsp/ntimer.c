#include "ti_msp_dl_config.h"
#include "system.h"

#include "ntimer.h"



systime timer_t1a,timer_t5a,timer_t6a,timer_t7a;
extern void maple_duty_200hz(void);
extern void duty_1000hz(void);


void timer_irq_config(void)
{
  //pwm
  DL_TimerA_startCounter(PWM_0_INST);

	
  NVIC_EnableIRQ(TIMER_G0_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_G0_INST);

	
  NVIC_EnableIRQ(TIMER_G6_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_G6_INST);
	
  NVIC_EnableIRQ(TIMER_G12_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_G12_INST);
}

void timer_pwm_config(void)
{
  //pwm
  DL_TimerA_startCounter(PWM_0_INST);//A0
}





systime t_g0;
void TIMER_G0_INST_IRQHandler(void)
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G0_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
      get_systime(&t_g0);
      maple_duty_200hz();
      static uint32_t _cnt = 0; _cnt++;
      if(_cnt % 50 == 0)	DL_GPIO_togglePins(GPIO_RGB_PORT, GPIO_RGB_GREEN_PIN);
    }
    break;

    default:
      break;
  }
}






systime t_g12;
void TIMER_G12_INST_IRQHandler(void)
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G12_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
      get_systime(&t_g12);
			duty_1000hz();
    }
    break;
    default:
      break;
  }
}




systime t_g1;
void TIMER_G6_INST_IRQHandler(void)//地面站数据发送中断函数
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G6_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
      get_systime(&t_g1);
    }
    break;

    default:
      break;
  }
}

systime t_g8;
void TIMER_G8_INST_IRQHandler(void)//地面站数据发送中断函数
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G8_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
      get_systime(&t_g8);
    }
    break;
    default:
      break;
  }
}
