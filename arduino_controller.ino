#include <Servo.h>              //Servo library
#include <mcp_can.h>            //CAN Bus Shield Compatibility Library
#include <SPI.h>                //CAN Bus Shield SPI Pin Library
int rpm = 0;                    //variable for RPM
short tempRPM = 0;              //variable for calculating the bytes that needs to be send to the cluster
byte rpmL = 0;                  //
byte rpmH = 0;                  //the bytes that need to be send
const int SPI_CS_PIN = 9;       //Pin on where the CAN Bus Shield has its SPI_CS Pin
MCP_CAN CAN(SPI_CS_PIN);        // Set CS pin
Servo servo;                    //create an objekt of one servo called "servo"
#define PACKET_SYNC 0xFF        //needed to "talk" to the ETS SDK
#define PACKET_VER  2           //needed to "talk" to the ETS SDK
const int handbreak = A0;       //pin where I connected the signal for the handbreak to
const int oilPressureSwitch = A1;//pin where I connected the signal for the oilpressureswitch to
int serial_byte;                 //the serial byte that is send to the arduino
int pack_counter = 0;           //a counter variable for sending commands in a specific delay
bool seat_belt=false;
int temp_seat_belt;
bool battery_warning=false;
int temp_battery_warning;
bool trunklid_open=false;
int temp_trunklid_open;
bool check_lamp;
int temp_check_lamp;
bool door_open=false;
int seatbeltpin=A4;
int doorpin=A3;
int trunklidpin=A2;
bool backgroundlight=false;
void setup()
{
  
  Serial.begin(115200);                 //start a serial communication with the baud-rate:115200
    pinMode(handbreak,OUTPUT);          //set this pin as an output
    pinMode(oilPressureSwitch,OUTPUT);  //set this pin as an output
    pinMode(seatbeltpin,INPUT_PULLUP);  //set this pin as an input with internal pullup-resistor
    pinMode(doorpin,INPUT_PULLUP);      //set this pin as an input with internal pullup-resistor
    pinMode(trunklidpin,INPUT_PULLUP);
    servo.attach(3);                    //attach the servo for the speedometer to pin A3
    delay(10);                          //wait 10 seconds
    servo.writeMicroseconds(835);       //set the servo to the home-position (in my case 835ms pulse width)
    delay(10);                          //wait 10 seconds
if(tempRPM>1000){digitalWrite(oilPressureSwitch,LOW);}else{digitalWrite(oilPressureSwitch,HIGH);}//set the oilpressureswitch corresponding to the RPM
  START_INIT:                           //label where to jump to

    if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
       // Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
                                          // Serial.println("CAN BUS Shield init fail");
                                          // Serial.println("Init CAN BUS Shield again");
        delay(100);                       //wait 100ms
        goto START_INIT;                  //jump to START_INIT:
    }
    
}

void CanSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h)//function to send CAN-commands
{
  unsigned char DataToSend[8] = {a,b,c,d,e,f,g,h};//array storing a small buffer
  CAN.sendMsgBuf(address, 0, 8, DataToSend);      //sending the buffer using the given adress
}

void read_serial_byte_set_servo(int data)         //function to translate the serial data into KM/H values and also sending them
{
  int serial_byt = data;                          //set serial_byt=data
  serial_byt = (serial_byt < 0) ? 0 : ((serial_byt > 180) ? 180 : serial_byt);//when serial_byt is smaller than 0, then set serial_byt=0 else if serial_byt is greater than 180 set serial_byt=180
  serial_byt= (0.9*serial_byt);                   //calculate the value that needs to be send more precise
  if(serial_byt<60){serial_byt=serial_byt-1;}
   if(serial_byt==0){serial_byt=0;}
  if(serial_byt<101)                              //if serial_byt is smaller than 101
  {
    serial_byt=map(serial_byt, 0,100,835,1965);   //map the serial_byt that reaches from 0 to 100 to 835 to 1965
  servo.writeMicroseconds(serial_byt);            //send the mapped values
  }
}

void read_serial_byte_set_servo2(int serial)//function to translate the serial data into RPM values and also sending them
{
   if(serial<0){serial=0;}else{if(serial>180){serial=180;}}   //when serial_byt is smaller than 0, then set serial_byt=0 else if serial_byt is greater than 180 set serial_byt=180
  serial = (serial < 0) ? 0 : ((serial > 180) ? 180 : serial);//when serial_byt is smaller than 0, then set serial_byt=0 else if serial_byt is greater than 180 set serial_byt=180
  serial = map(serial,0,180,0,8000);                          //map the serial_byt that reaches from 0 to 180 to 0 to 8000
  if(serial==0){backgroundlight=false;}else{backgroundlight=true;}
  tempRPM = serial*4;                                         //multiply with 4 
  rpmL = (tempRPM&0xFF);                                      //calculate the LOW-byte for the cluster
  rpmH = tempRPM>>8;                                          //calculate the HIGH-byte for the cluster
  CanSend(0x280, 0x49, 0x0E, rpmL, rpmH, 0x0E, 0x00, 0x1B, 0x0E);//send the calculated values to the cluster
  if(tempRPM>1000){digitalWrite(oilPressureSwitch,LOW);}else{digitalWrite(oilPressureSwitch,HIGH);}//set the oilpressureswitch corresponding to the RPM
}
void skip_serial_byte() //function to skip a serial_byte
{
  (void)Serial.read();
}

void digitalWriteFromBit(int value, int shift)    
{
  bool state= !((value >> shift) & 0x01);
  digitalWrite(handbreak, state);                 //write the handbreak-pin depending on the value given 0=handbreak on 1=handbreak of
}
void lichter2(int data){
if(digitalRead(seatbeltpin)!=HIGH){
  seat_belt = true;
}else{
  seat_belt = false;
}
if(digitalRead(trunklidpin)!=HIGH){
  trunklid_open = true;
}else{
  trunklid_open = false;
}
if(((data >> 2) & 0x01)==HIGH){
  battery_warning=true;
}else{
  battery_warning=false;
}
}
void lichter(int data){
  int blinker=0;
  int beam=0;
   /*Truck lights:

6: Parking light
5: Left indicator
4: Right indicator
3: Low beam
2: High beam
1: Brake
0: Reverse

warning lights:

7: Parking brake
6: Motor brake
5: Air pressure warning
4: Air pressure emergency
3: Fuel warning
2: Battery voltage warning
1: Oil pressure warning
0: Water temperature warning */
if(((data >> 6) & 0x01)==HIGH){
  check_lamp =true;
}else{
  check_lamp = false;
}
if(digitalRead(doorpin)!=HIGH){
  door_open=true;
}else{
  door_open=false;
}

if(((data >> 5) & 0x01)==HIGH&&((data >> 4) & 0x01)==HIGH){
  blinker=3;
}
else{
  if(((data >> 5) & 0x01)==HIGH){blinker=1;}
  if(((data >> 4) & 0x01)==HIGH){blinker=2;}
  if((((data >> 5) & 0x01)!=1)&&(((data >> 4) & 0x01)!=1)){blinker=0;}
}
if(((data >> 3) & 0x01)==HIGH&&((data >> 2) & 0x01)==HIGH){beam=0x16;}
else{
if(((data >> 3) & 0x01)==HIGH){beam=0x13;}
if(((data >> 2) & 0x01)==HIGH){beam=0x04;}
if((((data >> 2) & 0x01)!=1)&&(((data >> 3) & 0x01)!=1)){beam=0;}
}//(data >> shift) & 0x01
CanSend(0x531,beam/*Beam*/,0,blinker/*Blinker*/,0,0,0,0,0);
}
void loop()
{
    //Immobilizer
  CanSend(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0);

  //Airbag
  CanSend(0x050, 0, 0x80, temp_seat_belt, 0, 0, 0, 0, 0);

  //Engine on and ESP enabled
  CanSend(0xDA0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  //ABS
  //CanSend(0x1A0, 0x18, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xff);
  CanSend(0x1A0, 0x18, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0xff);
  if(backgroundlight){
    CanSend(1589,64,0,0,0,0,0,0,0);
    }else{
    CanSend(1589,0,0,0,0,0,0,0,0);
    }
  
  signallamps();
  if (Serial.available() < 16)
    return;
  
  serial_byte = Serial.read();
  if (serial_byte != PACKET_SYNC)
    return;
    
  serial_byte = Serial.read();
  if (serial_byte != PACKET_VER)
    return;
  

  serial_byte = Serial.read();
  read_serial_byte_set_servo(serial_byte) ;
 
  serial_byte = Serial.read();
  read_serial_byte_set_servo2(serial_byte);// RPM
  
  
  skip_serial_byte(); // Brake air pressure
  skip_serial_byte(); // Brake temperature
  skip_serial_byte(); // Fuel ratio
  skip_serial_byte(); // Oil pressure
  skip_serial_byte(); // Oil temperature
  skip_serial_byte(); // Water temperature
  skip_serial_byte(); // Battery voltage
    
 
  // Truck lights byte
  serial_byte = Serial.read();
  lichter(serial_byte);
  
  // Warning lights bytes

  serial_byte = Serial.read();  
  digitalWriteFromBit(serial_byte, 7);
  lichter2(serial_byte);
  // Enabled flags
  serial_byte = Serial.read();
  
  // Text length
  int text_len = Serial.read();
  
  // Followed by text
  if (0 < text_len && text_len < 127)
  {
    
    for (int i = 0; i < text_len; ++i)
    {
      while (Serial.available() == 0) // Wait for data if slow
      {
        //delay(2);
      }
      serial_byte = Serial.read();
      if (serial_byte < 0 && serial_byte > 127)
        return;
      
      
       
//      delay(2);
    }
  }
  //delay(1);
}
void signallamps(){
  //Seat Belt
  if (seat_belt == true) temp_seat_belt = B00000100;
  else temp_seat_belt = B00000000;
  
  //Battery Warning
  if (battery_warning == true) temp_battery_warning = B10000000;
  else temp_battery_warning = B00000000;
  
  //Trunk Lid (Kofferraumklappe)
  if (trunklid_open == true) temp_trunklid_open = B00100000;
  else temp_trunklid_open = B00000000;

  //Check Lamp Signal
  if (check_lamp == true) temp_check_lamp = B00010000;
  else temp_check_lamp = B00000000;
  
  pack_counter++;
  if (pack_counter == 20)
  {
    //Turning Lights 2
    CanSend(0x470, temp_battery_warning, temp_trunklid_open + door_open, 0x00, 0x00, temp_check_lamp, 0x00, 0x00, 0x00);
    pack_counter = 0;
  }
}
