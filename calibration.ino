#include <Wire.h>

float T = 500; //sample time in ms

#define i2c_id 0x68                     //default I2C address   
#define one_byte_read 0x01              //used in a function to read data from the device  
#define two_byte_read 0x02              //used in a function to read data from the device
#define four_byte_read 0x04             //used in a function to read data from the device

byte bus_address = i2c_id;              //holds the I2C address. 
byte bytes_received_from_computer = 0;  //we need to know how many character bytes have been received.
byte serial_event = 0;                  //a flag to signal when data has been received from the serial monitor

char computerdata[20];                  //we make an 20 byte character array to hold incoming data from the serial monitor
char *cmd;                              //char pointer used in string parsing
char *data_byte_1;                      //char pointer used in string parsing
char *data_byte_2;                      //char pointer used in string parsing

uint16_t icr = 0xffff;

union sensor_mem_handler                //declare the use of a union data type
{
  byte i2c_data[4];                   //define a 4 byte array in the union
  long answ;              //define an long in the union
};
union sensor_mem_handler move_data;     //declare that we will refer to the union as �move_data�
volatile byte new_reading = 0;          //a flag to signal when the interrupt pin sees a new reading 
volatile byte continuous_mode = 0;      //use to enable / disable continuous readings 

void setup() 
{                                     //hardware setup
  Serial.begin(9600);                                 //enable serial port at 9600 baud
  setupPWM16();
  Wire.begin();                                   //enable I2C port
  Serial.println("begin");
  active_con(); //turn on temp reader
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  println(reading());
  delay(T);
}

void active_con() 
{
    const byte active_hibernate_register = 0x06;                    //register to read / write
    const byte active_mode = 0x01;                                  //take readings
    const byte hibernate_mode = 0x00;
    
    i2c_write_byte(active_hibernate_register, active_mode);     //write the active mode enable command
    i2c_read(active_hibernate_register, one_byte_read);         //read from the active / hibernate control register to confirm it is set correctly 
    if (move_data.i2c_data[0] == 1)Serial.println("active");
}

float reading() 
{ 
  const byte RTD_register = 0x0E;                 //register to read
  float RTD = 0;                          //used to hold the new RTD value

  i2c_read(RTD_register, four_byte_read);             //I2C_read(OEM register, number of bytes to read)                  
  RTD = move_data.answ;                     //move the 4 bytes read into a float
  RTD /= 1000;                            //divide by 100 to get the decimal point 
  return RTD;
}

void i2c_write_byte(byte reg, byte data) 
{                              //used to write a single byte to a register: i2c_write_byte(register to write to, byte data) 

  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                        //transmit the register that we will start from
  Wire.write(data);                                       //write the byte to be written to the register 
  Wire.endTransmission();                                     //end the I2C data transmission
}

void i2c_read(byte reg, byte number_of_bytes_to_read) 
{                        //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)    

  byte i;                                             //counter
  
  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                        //transmit the register that we will start from
  Wire.endTransmission();                                     //end the I2C data transmission
  Wire.requestFrom(bus_address, (byte)number_of_bytes_to_read);                 //call the device and request to read X bytes
  for (i = number_of_bytes_to_read; i>0; i--) { move_data.i2c_data[i - 1] = Wire.read(); }        //with this code we read multiple bytes in reverse
  Wire.endTransmission();                                     //end the I2C data transmission  
}
