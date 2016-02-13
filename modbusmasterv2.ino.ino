#include <SimpleModbusMaster.h>

/*
   The example will use packet1 to read a register from address 0 (the adc ch0 value)
   from the arduino slave (id=1). It will then use this value to adjust the brightness
   of an led on pin 9 using PWM.
   It will then use packet2 to write a register (its own adc ch0 value) to address 1 
   on the arduino slave (id=1) adjusting the brightness of an led on pin 9 using PWM.
*/

//////////////////// Port information ///////////////////
#define baud 9600
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 2 

#define LED 13

// The total amount of available memory on the master to store data
//#define TOTAL_NO_OF_REGISTERS 1
 unsigned int scratchpad1, scratchpad2;  //used by floatsplit
// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  Cell1HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell1LowByte,  // low byte register for measure. used for float on slave in mv
  Node1outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node1HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node1LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node1HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node1LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
   Cell2HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell2LowByte,  // low byte register for measure. used for float on slave in mv
  Node2outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node2HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node2LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node2HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node2LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell3HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell3LowByte,  // low byte register for measure. used for float on slave in mv
  Node3outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node3HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node3LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node3HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node3LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell4HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell4LowByte,  // low byte register for measure. used for float on slave in mv
  Node4outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node4HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node4LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node4HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node4LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell5HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell5LowByte,  // low byte register for measure. used for float on slave in mv
  Node5outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node5HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node5LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node5HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node5LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell6HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell6LowByte,  // low byte register for measure. used for float on slave in mv
  Node6outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node6HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node6LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node6HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node6LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell7HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell7LowByte,  // low byte register for measure. used for float on slave in mv
  Node7outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node7HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node7LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node7HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node7LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell8HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell8LowByte,  // low byte register for measure. used for float on slave in mv
  Node8outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node8HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node8LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node8HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node8LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell9HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell9LowByte,  // low byte register for measure. used for float on slave in mv
  Node9outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node9HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node9LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node9HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node9LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell10HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell10LowByte,  // low byte register for measure. used for float on slave in mv
  Node10outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node10HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node10LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node10HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node10LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell11HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell11LowByte,  // low byte register for measure. used for float on slave in mv
  Node11outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node11HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node11LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node11HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node11LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
    Cell12HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell12LowByte,  // low byte register for measure. used for float on slave in mv
  Node12outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node12HighByteTopSetpoint, //point that dump reisitor turns on high bytes; used as float on slave in mv
  Node12LowByteTopSetpoint,  //point that dump resistor turns on low bytes
  Node12HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node12LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_PACKETS];

void setup()
{
  // Initialize each packet
  for (int RegisterRef=0; RegisterRef<=(Total_NO_OF_Packets-1); RegisterRef++) {
    int ModbusAddress = RegisterRef%7;
    int NodeId = RegisterRef/7;
  
  modbus_construct(&packets[RegisterRef], NodeId, READ_HOLDING_REGISTERS,ModbusAddress, 2, RegisterRef);
  }

  
//  modbus_construct(&packets[NodeHighByteSetpoint], 1, PRESET_MULTIPLE_REGISTERS, 0, 1, 0);
  // parameters: packet/ID Slave/ function/ Modbus address/Total numbr of registers/position in the  'regs' array.
  // Initialize the Modbus Finite State Machine
  modbus_configure(&Serial, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  
//  pinMode(LED, OUTPUT);

//now send out setpoints to all nodes...
//high setpoint. Uses a float of mv that is then split apart as 16bit integers to send over modbus.
float HighSetpoint;  // need to change this to use eeprom later...
 Floatsplit(HighSetpoint);  // this will leave float in two general 16 bit integer scratchpad.
Node1HighByteTopSetpoint = scratchpad1;
Node1LowByteTopSetpoint = scratchpad2;
}

void Floatsplit(float f) {
  unsigned int * b = (unsigned int *) &f;
 scratchpad1=b[0];
  scratchpad2=b[1];   
  }

void loop()
{
  modbus_update();

 
  long timerstart = millis();
  if (millis()< (timerstart+1000)) {
  regs[0] = 1024; // update data to be written to arduino slave

  modbus_update();}
 if (millis()< (timerstart+2000)) {
  regs[0] =0;
  modbus_update();
 }
 else timerstart=millis();
  
 // analogWrite(LED, regs[0]>>2); // constrain adc value from the arduino slave to 255
}
