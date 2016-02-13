#include <SimpleModbusMaster.h>
#include <string.h>

/*
This code is intended to run on a arudino Mega. It reads a float over two 16 bit transfers and then displays them on a lcd. It also can turn on bits 0 and 1 on each
slave in a 16bit output register. It also uses a two 16 bit registers to transfer a float value in mv that is the upper alarm level. Also another two 16bit registers transfer a float value in mv
that is the lower alarm level. Each node has a output that is set if a cell is in alarm state. There is also the dumpsetpoint that is on two bytes as well.

The master simply uses a hardware serial port to talk using modbus RTU protocol. There is no acutal connection of the txen pin as rs485 isnot implemented in hardware. Rather a mixed 
open collector  optoisolated serial bus is used.

Later additions will include a data connection to the outside world... possible also modbus, or wifi or just plain serial.
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

 float CellVoltages[12]; //array of voltages as floats for general use, display and output
// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  Cell1HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell1LowByte,  // low byte register for measure. used for float on slave in mv
  Node1outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node1HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node1LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node1HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node1LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node1HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node1LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
  Cell2HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell2LowByte,  // low byte register for measure. used for float on slave in mv
  Node2outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node2HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node2LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node2HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node2LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node2HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node2LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell3HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell3LowByte,  // low byte register for measure. used for float on slave in mv
  Node3outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node3HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node3LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node3HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node3LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node3HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node3LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell4HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell4LowByte,  // low byte register for measure. used for float on slave in mv
  Node4outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node4HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node4LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node4HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node4LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node4HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node4LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell5HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell5LowByte,  // low byte register for measure. used for float on slave in mv
  Node5outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node5HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node5LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node5HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node5LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node5HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node5LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell6HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell6LowByte,  // low byte register for measure. used for float on slave in mv
  Node6outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node6HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node6LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node6HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node6LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node6HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node6LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell7HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell7LowByte,  // low byte register for measure. used for float on slave in mv
  Node7outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node7HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node7LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node7HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node7LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node7HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node7LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell8HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell8LowByte,  // low byte register for measure. used for float on slave in mv
  Node8outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node8HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node8LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node8HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node8LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node8HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node8LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell9HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell9LowByte,  // low byte register for measure. used for float on slave in mv
  Node9outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node9HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node9LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node9HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node9LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node9HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node9LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell10HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell10LowByte,  // low byte register for measure. used for float on slave in mv
  Node10outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node10HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node10LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node10HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node10LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node10HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node10LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
      Cell11HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell11LowByte,  // low byte register for measure. used for float on slave in mv
  Node11outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node11HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node11LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node11HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node11LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node11HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node11LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
    Cell12HighByte,  //each register is only 16bit so we need 2 registers for a float high byte measure
  Cell12LowByte,  // low byte register for measure. used for float on slave in mv
  Node12outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node12HighByteTopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node12LowByteTopSetpoint,  //point that alarm turns on low bytes
  Node12HighByteBottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Node12LowByteBottomSetpoint,  //point that low voltage alarm goes off high byte
  Node12HighByteDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
  Node12LowByteDumpSetpoint,    //point that dump resistor turns on low byte. part of float in mv.
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_PACKETS];

void setup()
{
  // Initialize each packet
  for (int RegisterRef=0; RegisterRef<(TOTAL_NO_OF_PACKETS); RegisterRef++) {
    int ModbusAddress = RegisterRef%9;
    int NodeId = RegisterRef/9;
  
 if (ModbusAddress<2)  modbus_construct(&packets[RegisterRef], NodeId, READ_HOLDING_REGISTERS,ModbusAddress, 2, RegisterRef);
 else modbus_construct(&packets[RegisterRef], NodeId, PRESET_MULTIPLE_REGISTERS,ModbusAddress, 7, RegisterRef);
  }

  
//  modbus_construct(&packets[NodeHighByteSetpoint], 1, PRESET_MULTIPLE_REGISTERS, 0, 1, 0);
  // parameters: packet/ID Slave/ function/ Modbus address/Total numbr of registers/position in the  'regs' array.
  // Initialize the Modbus Finite State Machine
  modbus_configure(&Serial1, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);


//now send setpoints to all nodes in array of reg...


for (int RegisterRef=0; RegisterRef<(TOTAL_NO_OF_PACKETS); RegisterRef++){
     int NodePointer = RegisterRef%9;



  float HighSetpoint=4000;  // need to change this to use eeprom later...this the mv value where the alarm is set off
 Floatsplit(HighSetpoint);  // this will leave float in two general 16 bit integer scratchpad.
if ((NodePointer == 4)) regs[RegisterRef] = scratchpad1;
if (NodePointer == 5) regs[RegisterRef] = scratchpad2;
 float LowSetpoint=2500;  // need to change this to use eeprom later...this is the mv value where the alarm is set off (lower)
 Floatsplit(LowSetpoint);  // this will leave float in two general 16 bit integer scratchpad.
if (NodePointer == 6) regs[RegisterRef] = scratchpad1;
if (NodePointer == 7) regs[RegisterRef] = scratchpad2;
if (NodePointer == 3) regs[RegisterRef] = 0;  //set outputs to off.
  float dumpSetpoint=3900;  // need to change this to use eeprom later..this is the mv value where the dump kicks in.
 Floatsplit(LowSetpoint);  // this will leave float in two general 16 bit integer scratchpad.
 if (NodePointer == 8) regs[RegisterRef] = scratchpad1;
if (NodePointer == 9) regs[RegisterRef] = scratchpad2;
}

}

void Floatsplit(float f) {
  unsigned int * b = (unsigned int *) &f;
 scratchpad1=b[0];
  scratchpad2=b[1];   
  }



void loop()
{
  modbus_update();
for (int nodeid=0;nodeid<=12;nodeid++){
  CellVoltages[nodeid]=reform_uint16_2_float32(regs[(nodeid)],regs[(nodeid+1)]);
}

  
 // analogWrite(LED, regs[0]>>2); // constrain adc value from the arduino slave to 255
}

float reform_uint16_2_float32(uint16_t u1, uint16_t u2)
{
    long int num = ((u2 & 0xFFFF) << 16) | (u1 & 0xFFFF);
    float numf;
    memcpy(&numf, &num, 4);
    return numf;
}
