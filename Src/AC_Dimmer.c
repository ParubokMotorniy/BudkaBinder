//17.11.2022
//by Reptiloid software <- fucking legend
#include "AC_Dimmer.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include <inttypes.h>

#define DIMM_MAX 5
#define AC_FREQ 50
#define MAX_HEAT_DIV 10

//======================================
#define TRIAC_PULSE_TIME 30
#define ZERO_VAL 19850
#define K 77

#define SSR  0
#define DIMM 1
#define HEAT 2

extern TIM_HandleTypeDef htim1;
//=======================================

struct dimmer
  {
	  uint16_t flag;
      uint16_t val;
      uint16_t pin;
      uint16_t SSR_state;
      uint16_t heat_val;
  };
  
struct sort
  {
	  uint16_t val;
      uint16_t pin;
  };
  

//==========================================
  
 volatile unsigned char cnt;
 volatile unsigned char dims;
 volatile unsigned char lim;
 volatile unsigned char not_z;
 volatile unsigned char heat_cnt = 0;
 volatile unsigned char dim_val_func_running = 0;
 volatile unsigned char dim_mode_only = 1;
 
  struct dimmer dimmerArr[DIMM_MAX];
  struct sort sort_vals[DIMM_MAX];
  struct sort sort[DIMM_MAX*2];

//===============================================


 

void Dimmer_init_begin() //Done
{
//    TCCR1A=0;
//	TCCR1B=2;
//	TIMSK1=1;
//
//	EICRA=3;
//	EIMSK=1;
//	EIFR=1;
   
    for(unsigned char i=0; i<DIMM_MAX; i++)
    {
        dimmerArr[i].pin = 0xff;
        dimmerArr[i].val = 0;
        dimmerArr[i].flag = SSR;
        dimmerArr[i].SSR_state = 0;
    }
    
     for(unsigned char i=0; i<DIMM_MAX*2; i++)
    {
        sort[i].pin = 0xff;
    }
}

void Dimmer_pin_assign(unsigned char dim_num, unsigned char dim_pin) //Done
{
	dimmerArr[dim_num].pin = dim_pin;
}


void Dimmer_init_end() //Done
{
    unsigned char j=0;
    
    for(unsigned char i=0; i<DIMM_MAX; i++)
    {
        if(dimmerArr[i].pin != 0xff) //count how many dimmers
        { 
            sort_vals[j].pin = dimmerArr[i].pin;
            j++; 
//            GPIO_InitTypeDef GPIO_InitStruct = {0};
//            __HAL_RCC_GPIOE_CLK_ENABLE();
//            /* Configure the GPIO pin */
//            GPIO_InitStruct.Pin = dimmerArr[i].pin;
//            GPIO_InitStruct.Mode =  GPIO_MODE_OUTPUT_PP;
//            GPIO_InitStruct.Pull = GPIO_NOPULL;
//            GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
//            HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

            //pinMode(dimmer[i].pin, OUTPUT);
        }
		
		if((dimmerArr[i].flag == HEAT) || (dimmerArr[i].flag == SSR)) { dim_mode_only = 0; }
    }
    
    dims = j;
}

void tim_form() //Done
{
    unsigned char j;
	if(dim_val_func_running == 0)
	{
		not_z = 0;
		for(j = 0; (sort_vals[j].val == 0 && j < dims); j++) {not_z++;}

		lim = dims - not_z;
		sort[lim*2-1] = sort_vals[dims-1];

		for(j=0; j<lim-1; j++)
		{
		   sort[lim*2-2-j].val = sort_vals[dims-1-j].val - sort_vals[dims-j-2].val;
		   sort[lim*2-2-j].pin = sort_vals[dims-j-2].pin;
		}

		sort[lim-1].val = 255 - sort_vals[dims-1].val + sort_vals[not_z].val;
		sort[lim-1].pin = sort_vals[dims-1].pin;

		for(j = 0; j<lim-1; j++)
		{
			sort[j] = sort[j+lim];
		}
	}
}

void Dimm_value(unsigned char dim_num, unsigned char power) //Done
{
	dim_val_func_running = 1;
    if(power > 240) {power = 240;}
	 if(dims > 1)
	 {
		dimmerArr[dim_num].flag = DIMM;
		dimmerArr[dim_num].val = power;
		unsigned char j;

		for(unsigned char i = 0; i<dims; i++)
		{
			if(sort_vals[i].pin == dimmerArr[dim_num].pin)
			{
				if(power > sort_vals[i+1].val && i<dims-1)
				{
					for(j = 1; ((power > sort_vals[i+j].val) && (i+j<dims)); j++)
					{
						sort_vals[i+j-1] = sort_vals[i+j];
					}

					sort_vals[i+j-1].val = power;
					sort_vals[i+j-1].pin = dimmerArr[dim_num].pin;
				}
				else if(power < sort_vals[i-1].val && i>0)
				{
					for(j = 1; ((power < sort_vals[i-j].val) && (i-j>=0)); j++)
					{
						sort_vals[i-j+1] = sort_vals[i-j];
					}

					sort_vals[i-j+1].val = power;
					sort_vals[i-j+1].pin = dimmerArr[dim_num].pin;
				 }
				 else {sort_vals[i].val = power;}

				i = dims;
			}
		}
	 }
	 else //single dimmer
	 {
		sort_vals[0].val = power;
		sort_vals[0].pin = dimmerArr[dim_num].pin;
	 }
	dim_val_func_running = 0;
}

void Heater(unsigned char dim_num, unsigned char heat_power) //Done
{
    Dimm_value(dim_num, 0);
    
    dimmerArr[dim_num].flag = HEAT;
    dimmerArr[dim_num].heat_val = heat_power;
    
    //if(heat_power == 0) digitalWrite(dimmer[dim_num].pin, 0);
    if(heat_power == 0) {HAL_GPIO_WritePin(GPIOA, dimmerArr[dim_num].pin, GPIO_PIN_RESET);}
}

void SSR_switch(unsigned char dim_num, unsigned char state) //Done
{
    Dimm_value(dim_num, 0);

    dimmerArr[dim_num].flag = SSR;
    dimmerArr[dim_num].SSR_state = state;

    //if(state == 0) digitalWrite(dimmer[dim_num].pin, 0);
    if(state == 0) {HAL_GPIO_WritePin(GPIOA, dimmerArr[dim_num].pin, GPIO_PIN_RESET);}
}


//======================================================================

void HandleDimmerInterrupt()
{
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	if(dims > 1)
	{
		tim_form();
		cnt=lim*2-1;
		__HAL_TIM_SET_COUNTER(&htim1, (0xffff - (ZERO_VAL - K*sort[cnt].val)));
		//TCNT1 = 0xffff - (ZERO_VAL - K*sort[cnt].val);
	}
	else  //single dimer
	{
		__HAL_TIM_SET_COUNTER(&htim1, (0xffff - (ZERO_VAL - K*sort_vals[0].val)));
		//TCNT1 = 0xffff - (ZERO_VAL - K*sort_vals[0].val);
	}
  
	if(dim_mode_only == 0)
	{
		for(unsigned char i = 0; i<DIMM_MAX; i++)
		{
				if(dimmerArr[i].flag == HEAT)
				{
					//if(dimmer[i].heat_val > heat_cnt) digitalWrite(dimmer[i].pin, 1);
					if(dimmerArr[i].heat_val > heat_cnt) {HAL_GPIO_WritePin(GPIOA, dimmerArr[i].pin, GPIO_PIN_SET);}
					else {HAL_GPIO_WritePin(GPIOA, dimmerArr[i].pin, GPIO_PIN_RESET);}
					//else digitalWrite(dimmer[i].pin, 0);
				}

				if(dimmerArr[i].flag == SSR)
				{
					//if(dimmer[i].SSR_state > 0) digitalWrite(dimmer[i].pin, 1);
					if(dimmerArr[i].SSR_state > 0) {HAL_GPIO_WritePin(GPIOA, dimmerArr[i].pin, GPIO_PIN_SET);}
				}
		}

		if(heat_cnt < MAX_HEAT_DIV-1) {heat_cnt++;}
		else {heat_cnt = 0;}
	}
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}


void HandleTimerInterrupt()
{
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    if(dims > 1)
    {
        //digitalWrite(sort[cnt].pin, 1);
		//delayMicroseconds(TRIAC_PULSE_TIME);
        //digitalWrite(sort[cnt].pin, 0);

//    	HAL_GPIO_WritePin(GPIOE, sort[cnt].pin, GPIO_PIN_SET);
//    	HAL_Delay(TRIAC_PULSE_TIME);
//		HAL_GPIO_WritePin(GPIOE, sort[cnt].pin, GPIO_PIN_RESET);

        HAL_GPIO_TogglePin(GPIOA, sort[cnt].pin);
   
        while(cnt >= 2 && sort[cnt-1].val == 0)
        {
            cnt--;
            //digitalWrite(sort[cnt].pin, 1);
			//delayMicroseconds(TRIAC_PULSE_TIME);
            //digitalWrite(sort[cnt].pin, 0);
//            HAL_GPIO_WritePin(GPIOE, sort[cnt].pin, GPIO_PIN_SET);
//            HAL_Delay(TRIAC_PULSE_TIME);
//            HAL_GPIO_WritePin(GPIOE, sort[cnt].pin, GPIO_PIN_RESET);

            HAL_GPIO_TogglePin(GPIOA, sort[cnt].pin);
        }
    
        cnt--;
        //TCNT1 = 0xffff - K*sort[cnt].val;
        __HAL_TIM_SET_COUNTER(&htim1, (0xffff - K*sort[cnt].val));
    }
	else //single dimmer
	{
		//digitalWrite(sort_vals[0].pin, 1);
        //delayMicroseconds(TRIAC_PULSE_TIME);
        //digitalWrite(sort_vals[0].pin, 0);

//        HAL_GPIO_WritePin(GPIOE, sort_vals[cnt].pin, GPIO_PIN_SET);
//        HAL_Delay(TRIAC_PULSE_TIME);
//        HAL_GPIO_WritePin(GPIOE, sort_vals[cnt].pin, GPIO_PIN_RESET);

        HAL_GPIO_TogglePin(GPIOA, sort_vals[cnt].pin);
		
		//TCNT1 = 0xffff - 20000;
        __HAL_TIM_SET_COUNTER(&htim1, (0xffff - 20000));
	}
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}



 
