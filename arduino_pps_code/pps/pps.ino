// example for Timer: robotshop.com/community/forum/t/arduino-101-interrupts/13072
// example for direct Pin control: dreamy.pe.kr/zbxe/CodeClip/3769045
#define PIN_PPS 10
#define PIN_TRIGGER 7

// DDRx : pin status (input or output). 
//  ex) DDRD = B11111110; // pin 1~7: output, pin 0: input (pin 0 and pin 1: TX RX. DO NOT CHANGE THEM!!!)
// PORTB: digital pin 8~13
// PORTC: analog pins
// PORTD: digital pin 0~7
#define B0 B00000001
#define B1 B00000010
#define B2 B00000100
#define B3 B00001000
#define B4 B00010000
#define B5 B00100000
#define B6 B01000000
#define B7 B10000000

// ex) PORTD |= B7 | B5 | B4; // digital pin 7 5 4 HIGH
//     PORTD &= ~(B7 | B5 | B4); // digital pin 7 5 4 LOW
//     PORTD ^= B7 | B5 | B4; // digital pin 7 5 4 toggle

#include <ros.h>
#include <std_msgs/String.h>
#include <camlidar_module/trg_msg.h> // custom message

// node handler
ros::NodeHandle nh;
camlidar_module::trg_msg trgmsg;
ros::Publisher pub_msg("/mcu/trigger", &trgmsg);


volatile unsigned long trigger_time = 0;
volatile unsigned long count = 0;
volatile unsigned long unit_time_counter = 0; 

volatile unsigned long cam_counter = 0;
volatile unsigned long pps_counter = 0;

volatile unsigned long time_sec = 0;
volatile unsigned long time_nsec = 0;

volatile unsigned long time_ss = 0;
volatile unsigned long time_mm = 0;
volatile unsigned long time_hh = 0;
volatile unsigned long time_msmsms = 0;

// volatile String nmea_msg = "$GPRMC,000000.000,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00\r\n"
    
volatile char header_c[6+1] = "GPRMC,";
volatile char hhmmss_msmsms_c[10+1] = "000000.000";
volatile char tail_c[25+1]  = ",A,2,N,1,E,,,230394,0,W,A";
//    volatile char tail_c[53+1]  = ",A,4804.000,N,4436.001,E,022.4,084.4,230394,003.1,W,A";

byte stringChecksum(volatile char* s, int len)
{
  byte c = 0;
  for(int i = 0; i< len; i++){
    c ^= s[i];
  }
  return c;
}

void setup(){
  // pin setting
  
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_PPS,OUTPUT);
  Serial3.begin(9600); // TX3 (D14), RX3 (D15)
  
  cli();// stop interrupts (== noInterrupts() ).
  //set timer 1 interrupt at 2 Hz
  TCCR1A = 0; // set entire TCCR1A register =0
  TCCR1B = 0; // same for B
  TCNT1 = 0;

  OCR1A = 12522-1; // 12500 * 4 us/cnt = 200000 us per compare!// MEGA 12521 -1 : 
  
  // turn on CTC mode
  TCCR1B |= (1 << WGM12); // CTC mode.
  //Set CS12 and CS10 bits for 8 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // % 64 prescaler, 4 us/cnt
  // cs02 cs01 cs00
  // 0 0 0 : no clock source
  // 0 0 1 : prescaler = 1 : 62.5 ns/cnt
  // 0 1 0 : prescaler = 8 : 500 ns/cnt
  // 0 1 1 : prescaler = 64 : 4 us/cnt
  // 1 0 0 : prescaler = 256 : 16 us/cnt
  // 1 0 1 : prescaler = 1024 : 64 us/cnt
  //enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 

  sei(); // allow interrupt

  // ROS initialization
  nh.getHardware()->setBaud(460800);

  nh.initNode();
  nh.advertise(pub_msg);
}

ISR(TIMER1_COMPA_vect){// 50 ms
  ++count;
  
  // log time
  //trigger_time = micros();
  trigger_time = unit_time_counter*49998; // 1.925 us / 50 ms fast 
  
  // send msg
  time_sec  = trigger_time/1000000;
  time_nsec = (trigger_time - time_sec*1000000)*1000; 
  
  
  if(count > 19) {
    for(int i = 0; i < 300; i++){
      PORTH = B4; // PIN_PPS 
      PORTB = B4;
    }
    PORTH &= ~B4;
    PORTB &= ~B4;
    count = 0;
    
    // fill ros msg
    trgmsg.status_trgs = 2;
    trgmsg.seq_cam = ++cam_counter;
    trgmsg.seq_pps = ++pps_counter;

    time_hh = time_sec / 3600;
    time_mm = time_sec / 60;
    time_ss = time_sec % 60; 
    time_msmsms = time_nsec / 1000000;

    if(time_hh < 10){
      hhmmss_msmsms_c[0] = '0';
      volatile char c[2];
      itoa(time_hh, c, 10);
      hhmmss_msmsms_c[1] = c[0];
    }
    else {
      volatile char c[3];
      itoa(time_hh, c ,10);
      hhmmss_msmsms_c[0]= c[0];
      hhmmss_msmsms_c[1] = c[1];
    }
    if(time_mm < 10){
      hhmmss_msmsms_c[2] = '0';
      volatile char c[2];
      itoa(time_mm, c, 10);
      hhmmss_msmsms_c[3] = c[0];
    }
    else {
      volatile char c[3];
      itoa(time_mm, c ,10);
      hhmmss_msmsms_c[2] = c[0];
      hhmmss_msmsms_c[3] = c[1];
    }
    if(time_ss < 10){
      hhmmss_msmsms_c[4] = '0';
      volatile char c[2];
      itoa(time_ss, c, 10);
      hhmmss_msmsms_c[5] = c[0];
    }
    else {
      volatile char c[3];
      itoa(time_ss, c ,10);
      hhmmss_msmsms_c[4] = c[0];
      hhmmss_msmsms_c[5] = c[1];
    }

    if(time_msmsms < 10){
      hhmmss_msmsms_c[7] = '0';
      hhmmss_msmsms_c[8] = '0';
      volatile char c[2];
      itoa(time_msmsms, c, 10);
      hhmmss_msmsms_c[9] = c[0];
    }
    else if(time_msmsms < 100){
      hhmmss_msmsms_c[7] = '0';
      volatile char c[3];
      itoa(time_msmsms, c, 10);
      hhmmss_msmsms_c[8] = c[0];
      hhmmss_msmsms_c[9] = c[1];
    }
    else{
      volatile char c[4];
      itoa(time_msmsms, c, 10);
      hhmmss_msmsms_c[7] = c[0];
      hhmmss_msmsms_c[8] = c[1];
      hhmmss_msmsms_c[9] = c[2];
    }
    
    
    // NMEA message (GPRMC)  
    volatile char buf[41+1];
    strcpy(buf,header_c);
    strcat(buf,hhmmss_msmsms_c);
    strcat(buf,tail_c);

    byte c = stringChecksum((volatile char*)buf,41);
    Serial3.print("$");
    Serial3.print((const char*)buf);
    Serial3.print("*");

    Serial3.println(c,HEX);
    Serial3.flush();
  }
  else {
    for(int i = 0; i < 300; i++){
      PORTH = B4; // PIN_PPS 
    }
    PORTH &= ~B4;
    
    // fill ros msg
    trgmsg.status_trgs = 1;
    trgmsg.seq_cam = ++cam_counter;
  }
  
  trgmsg.stamp.sec  = time_sec;
  trgmsg.stamp.nsec = time_nsec;
  pub_msg.publish(&trgmsg);
  nh.spinOnce(); // ROS message transmission

    
  ++unit_time_counter;
}


void loop(){
}


