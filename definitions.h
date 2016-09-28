
/*========define the communications you wish to use==========*/
#define I2C //define whether or not to use I2C communication
#define printSerial //define whether or not to use Serial Communication
//#define DEBUG //flag to print debug commands (comment out to exclude)


/*========define the component you wish to use==========*/
//#define pump
//#define peristaltic
//#define device  //define a device and which device is used, do not use with peristaltic pump
//#define injector
//#define coalescer
//#define mixer
//#define valve

/*========define the pitch of the leadscrew on the pump==========*/
#ifdef pump
 //#define pitch0_5 //pitch = 0.5mm/rev
 //#define pitch2_54 //pitch = 2.54mm/rev
 //#define pitch4_76 //pitch = 4.76mm/rev
#endif
 
/*========define which MBARI motor control board to use==========*/
#ifdef I2C 
  #define Address06 //select which address to use.
  /* --------I2C Addresses for boards---------

  pump + cassette                        == 0x03  //define pump, device, and cassette
  pump + coalescer                       == 0x05  //define pump, device, and coalescer
  pump + mixer                           == 0x07  //define pump, device, and mixer
  Injector + PCR injector oil syringe    == 0x06  //define pump, device and injector
  Valve + Droplet Generation oil syringe == 0x08  //define pump and valve
  Peristaltic + Separation oil syringe   == 0x09  //define pump and peristlatic
  Temperature Control Board              == 0x04
  LED_PMT board                          == 0x0A  

  */
  #ifdef Address03
    #define Address 0x03
    #define pump
    #define device
    #define cassette
    #define pitch2_54
    #define pump_484563
    //note set fmap of readsensorvalueA to map range of encoder fmap(1024 is ideal)
    
  #endif
  #ifdef Address05
    #define Address 0x05
    #define pump
    #define device
    #define coalescer
    #define pitch2_54
    #define pump_484563
    #define fmapadjust 16 //correct range of absolute encoder (=1024- max ADC output)
  #endif
  #ifdef Address07
    #define Address 0x02
    #define pump
    #define device
    #define mixer
    #define pitch2_54
    #define pump_484563
  #endif
 
  #ifdef Address06
    #define Address 0x06
    #define pump
    #define device
    #define injector
    #define pitch2_54
    #define pump_484563
  #endif
  #ifdef Address08
    #define Address 0x08
    #define pump
    #define pitch2_54
    #define pump_484563
    #define valve
  #endif
  #ifdef Address09
    #define Address 0x09
    #define pump
    #define peristaltic
    #define pitch2_54
    #define pump_484563
  #endif

#endif
