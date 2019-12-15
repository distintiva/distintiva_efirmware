#include "efirmware.h"

#include <Wire.h>


#define DEVICE_LED_ON            100
#define DEVICE_BUTTON            101


boolean runPackage(int device){
  
  //- override of base firmware command
  if(device == EF_TONE) {
    int hz = ef_read_short(EF_POS_0);
    int tone_time = ef_read_short(EF_POS_0 + 2);
    if(hz>0){
        tone(0, hz,tone_time);
    }else{
        noTone(0); 
    }
    return true;
  }

  //- my device/robot command
  if(device == DEVICE_LED_ON) {
    uint8_t led = ef_read_byte(EF_POS_0 );
    digitalWrite( led, HIGH);
    return true;
  }

     
  return false;

}



boolean getPackage(int device){

  if(device == DEVICE_BUTTON) {
   
    if( digitalRead( 11 )){
      ef_send_byte(1);
    }else{
      ef_send_byte(0);
    }
    return true;
  }

  if(device == MBIT_LIGHT) {
    ef_send_short((double)mbit_light_level() );
    return true;
  }

 

 return false;
}
 


void setup(){

 
 ef_setup();
 ef_callbacks(runPackage, getPackage);
 
 
 //- configure my device/robot pins
 pinMode( 11 , INPUT); //- button
 pinMode( led, OUTPUT);

 tone(0,500,50); 
 delay(50);
 noTone(0);
  
}

void loop(){

   ef_loop();

   // check_some_device_functions()


}
