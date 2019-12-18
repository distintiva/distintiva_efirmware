

#ifndef efirmnware_h
#define efirmnware_h

/*
ff 55 len idx action device EF_POS_0 slot data a
0  1  2   3   4      5      6         7    8
*/

#define EF_POS_0                   6  //- position of 1st user data inside buffer

#define EF_DIGITAL                30
#define EF_ANALOG                 31
#define EF_PWM                    32
#define EF_SERVO_PIN              33
#define EF_TONE                   34
#define EF_BUTTON_INNER           35
#define EF_ULTRASONIC_ARDUINO     36
#define EF_PULSEIN                37

void writeHead();
void writeEnd();
void writeBuffer(int idx,unsigned char c);
void writeSerial(unsigned char c);


extern void ef_setup();
extern void ef_loop();
extern void ef_callbacks( boolean (*fRun)(int),  boolean (*fGet)(int) );

extern void ef_send_byte(char c);
extern void ef_send_string(String s);
extern void ef_send_float(float value);
extern void ef_send_short(double value);
extern void ef_send_double(double value);
extern void ef_send_callback(uint8_t id);

extern uint8_t ef_read_byte(int idx);
extern short ef_read_short(int idx);
extern float ef_read_float(int idx);
extern char* ef_read_string(int idx,int len);
extern uint8_t* ef_read_byte_array(int idx,int len);






#endif