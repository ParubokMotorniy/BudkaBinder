//20.02.2019
//by Reptiloid software

//====================================================

#ifndef AC_Dimmer_h
#define AC_Dimmer_h 

//=================================================


    void Dimmer_init_begin(); 
	void Dimmer_pin_assign(unsigned char dim_num, unsigned char dim_pin);
	void Dimmer_init_end();
    void Dimm_value(unsigned char dim_num, unsigned char power);                 
	void SSR_switch(unsigned char dim_num, unsigned char state);
	void Heater(unsigned char dim_num, unsigned char heat_power);
	void HandleTimerInterrupt();
	void HandleDimmerInterrupt();

#endif
