#include "arduino.h"

//-NOTE:  If compiling other architectures comment these lines
#define EF_USE_ARDUINO_TONE
#define EF_USE_SERVO


#ifdef EF_USE_SERVO
#include <Servo.h>
#endif

#include "efirmware.h"
#include "eblock.h"

String mVersion = "00.00.000";



#ifdef EF_USE_SERVO //-------------------------
Servo servos[8]; 

int servo_pins[8]={0,0,0,0,0,0,0,0};

int searchServoPin(int pin){
    for(int i=0;i<8;i++){
      if(servo_pins[i] == pin){
        return i;
      }
      if(servo_pins[i]==0){
        servo_pins[i] = pin;
        return i;
      }
    }
    return 0;
}
#endif //-----------------------------------

void callOK();

//boolean isAvailable = false;
//char serialRead;
union{
    byte byteVal[4];
    float floatVal;
    long longVal;
}val;

union{
  byte byteVal[8];
  double doubleVal;
}valDouble;

union{
  byte byteVal[2];
  short shortVal;
}valShort;

int len = 52;  // ?¿?
char buffer[52];
byte idx = 0;
byte dataLen;

boolean isStart = false;

boolean isAvailable = false;
char serialRead;

uint8_t command_idx = 0;

double lastTime = 0.0;
double currentTime = 0.0;
unsigned char prevc=0;


typedef boolean (*func_run_t)(int);
func_run_t ef_runModule =NULL;
func_run_t ef_readData =NULL;

void writeHead(){
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd(){
 Serial.println(); 
}
void writeSerial(unsigned char c){
 Serial.write(c);
}

unsigned char ef_read_byte(int idx){
 return buffer[idx]; 
}
void writeBuffer(int idx,unsigned char c){
  buffer[idx]=c;
}

void readSerial(){
  isAvailable = false;
  if(Serial.available()>0){
    isAvailable = true;
    serialRead = Serial.read();
  }
}



void parseData(){
  isStart = false;
  int idx = ef_read_byte(3);  //- NEXTID (not used)
  command_idx = (uint8_t)idx;
  int action = ef_read_byte(4);
  int device = ef_read_byte(5);
  
  //-if t nextid,  set it to device (to notify return the action )
   if(idx==0)idx=device;
  

  switch(action){
    case GET:{

        writeHead();
        writeSerial(idx);
       if( device == EF_TIMER){
         ef_send_float(currentTime);
       }else{
         if( !ef_readData(device) ) { // Firmware overrides basic commands ?
            //----------------------------------------
            //- IF firmware doesn't override basic device commands then execute them here
            int pin = ef_read_byte(6);
            switch(device){
   
                  
              case  EF_DIGITAL:{
                pinMode(pin,INPUT);
                ef_send_float(digitalRead(pin));
              }
              break;
              case  EF_ANALOG:{
                 ef_send_short( e_get_analog(pin) );
              }
              break;
              case  EF_ANALOG_PERC:{
                 ef_send_short( e_get_analog_perc(pin) );
              }
              break;
            }
            //---------------------------------------

         }
       }
       
        writeEnd();
     }
     break;
     case RUN:{
       //runModule(device);
       if(device == VERSION){
            ef_send_string(mVersion);
       }else if( device == EF_TIMER){
         lastTime = millis()/1000.0;
       }else{
         
         if( !ef_runModule(device) ) {
           int pin = ef_read_byte(6);
           switch(device){
                  
              case EF_DIGITAL:{
                e_pin_set(pin, ef_read_byte(EF_POS_0+1)); //0,1 or 2(toggle)

              }
              break;
              case EF_PWM:{
                e_pwm(ef_read_byte(EF_POS_0), ef_read_byte(EF_POS_0+1) );
              }
              break;

              break;
              case EF_TONE:{
              #ifdef EF_USE_ARDUINO_TONE  
                int hz = ef_read_short(EF_POS_0);
                int tone_time = ef_read_short(EF_POS_0 + 2);
                if(hz>0){
                    tone(10, hz,tone_time);
                }else{
                    noTone(10); 
                }
              
              #endif
              }
              break;
             
    
              //----
              #ifdef EF_USE_SERVO
              case  EF_SERVO_PIN:{

                 int v = ef_read_byte(EF_POS_0+1);
                 uint8_t pin = ef_read_byte(EF_POS_0) ;
                 Servo sv = servos[searchServoPin(  pin )]; 
                 if(v >= 0 && v <= 180)
                 {
                   if(!sv.attached())
                   {
                     sv.attach(pin);
                   }
                   sv.write(v);
                 }
               
               }
             break;
              #endif //< USE_SERVO

              //----  


              
            }


         }
        
       }

       callOK();
     }
      break;
      case RESET:{
        //reset
       
        //buzzerOff();
        callOK();
      }
     break;
     case START:{
        //start
        callOK();
      }
     break;
  }
}
void callOK(){
    writeSerial(0xff);
    writeSerial(0x55);
    writeEnd();
}
void ef_send_byte(char c){
  writeSerial(1);
  writeSerial(c);
}
void ef_send_string(String s){
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int i=0;i<l;i++){
    writeSerial(s.charAt(i));
  }
}
//1 byte 2 float 3 short 4 len+string 5 double
void ef_send_float(float value){ 
     writeSerial(2);
     val.floatVal = value;
     writeSerial(val.byteVal[0]);
     writeSerial(val.byteVal[1]);
     writeSerial(val.byteVal[2]);
     writeSerial(val.byteVal[3]);
}
void ef_send_short(double value){
     writeSerial(3);
     valShort.shortVal = value;
     writeSerial(valShort.byteVal[0]);
     writeSerial(valShort.byteVal[1]);
     writeSerial(valShort.byteVal[2]);
     writeSerial(valShort.byteVal[3]);
}
void ef_send_double(double value){
     writeSerial(5);
     valDouble.doubleVal = value;
     writeSerial(valDouble.byteVal[0]);
     writeSerial(valDouble.byteVal[1]);
     writeSerial(valDouble.byteVal[2]);
     writeSerial(valDouble.byteVal[3]);
     writeSerial(valDouble.byteVal[4]);
     writeSerial(valDouble.byteVal[5]);
     writeSerial(valDouble.byteVal[6]);
     writeSerial(valDouble.byteVal[7]);
}

void ef_send_callback(uint8_t id){
      writeHead();
      writeSerial(0x80);
      ef_send_byte(1);  //- de momento value no se usa porque con eventId y el array de callbacks definido en la extensión se hace todo ( pero lo dejo implementado por si hiciera falta)
      ef_send_byte(id); //- eventId ==  A && pressed
      writeEnd();
}

short ef_read_short(int idx){
  valShort.byteVal[0] = ef_read_byte(idx);
  valShort.byteVal[1] = ef_read_byte(idx+1);
  return valShort.shortVal; 
}
float ef_read_float(int idx){
  val.byteVal[0] = ef_read_byte(idx);
  val.byteVal[1] = ef_read_byte(idx+1);
  val.byteVal[2] = ef_read_byte(idx+2);
  val.byteVal[3] = ef_read_byte(idx+3);
  return val.floatVal;
}
char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};
char* ef_read_string(int idx,int len){
  for(int i=0;i<len;i++){
    _receiveStr[i]=ef_read_byte(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}
uint8_t* ef_read_byte_array(int idx,int len){
  for(int i=0;i<len;i++){
    if(i>15){
      break;
    }
    _receiveUint8[i] = ef_read_byte(idx+i);
  }
  return _receiveUint8;
}


void ef_callbacks( boolean (*fRun)(int),  boolean (*fGet)(int) ){
  
   ef_runModule =fRun;
   ef_readData =fGet;

}

void ef_setup(){
   Serial.begin(115200);
}

void ef_loop(){

currentTime = millis()/1000.0-lastTime;
  
  readSerial();
  if(isAvailable){

 
    unsigned char c = serialRead&0xff;
    if(c==0x55&&isStart==false){
     if(prevc==0xff){
      idx=1;
      isStart = true;

     // Serial.print("Ok");
     }
    }else{
      prevc = c;
      if(isStart){
        if(idx==2){
         dataLen = c; 
        }else if(idx>2){
          dataLen--;
        }
        writeBuffer(idx,c);
      }
    }
     idx++;
     if(idx>51){
      idx=0; 
      isStart=false;
     }
     if(isStart&&dataLen==0&&idx>3){ 
        isStart = false;
        parseData(); 
        
        idx=0;
     }
  }

}