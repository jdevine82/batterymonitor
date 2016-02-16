#include <SimpleModbusMaster.h>
#include <string.h>

/*
This code is intended to run on a arudino Mega. It reads a float over two 16 bit transfers and then displays them on a lcd. It also can turn on bits 0 and 1 on each
slave in a 16bit output register. It also uses a two 16 bit registers to transfer a float value in mv that is the upper alarm level. Also another two 16bit registers transfer a float value in mv
that is the lower alarm level. Each node has a output that is set if a cell is in alarm state. There is also the dumpsetpoint that is on two bytes as well.

The master simply uses a hardware serial port to talk using modbus RTU protocol. There is no acutal connection of the txen pin as rs485 is not implemented in hardware. Rather a mixed 
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

 unsigned long CellVoltages[12]; //array of voltages as floats for general use, display and output
// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  Cell1Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure
  
  Node1outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node1TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
 
  Node1BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node1DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

  Cell2Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure
  
  Node2outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node2TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Node2BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node2DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell3Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure
 
  Node3outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node3TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node3BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node3DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell4Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure
 
  Node4outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node4TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node4BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
 
  Node4DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell5Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node5outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node5TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node5BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node5DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell6Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure
 
  Node6outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node6TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
 
  Node6BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
 
  Node6DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell7Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node7outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node7TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node7BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node7DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell8Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node8outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node8TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node8BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node8DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv
 
      Cell9Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node9outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node9TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node9BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node9HDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell10Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node10outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node10TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node10BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node10DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

      Cell11Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node11outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node11TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node11BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  
  Node11DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

    Cell12Voltage,  //each register is only 16bit so we need 2 registers for a float high byte measure

  Node12outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Node12TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

  Node12BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

  Node12DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv

  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_PACKETS];

void setup()
{
  // Initialize each packet
  for (int RegisterRef=10; RegisterRef<=30; RegisterRef++) {
    int ModbusAddress = RegisterRef%10;
    int NodeId = RegisterRef/10;
  
 if ((ModbusAddress=0) || (ModbusAddress=5))  modbus_construct(&packets[(RegisterRef-10)], (NodeId), READ_HOLDING_REGISTERS,ModbusAddress, 1, (RegisterRef-10));
 else modbus_construct(&packets[(RegisterRef-5)], (NodeId), PRESET_MULTIPLE_REGISTERS,ModbusAddress, 1, (RegisterRef-10));
  }

  
//  modbus_construct(&packets[NodeHighByteSetpoint], 1, PRESET_MULTIPLE_REGISTERS, 0, 1, 0);
  // parameters: packet/ID Slave/ function/ Modbus address/Total numbr of registers/position in the  'regs' array.
  // Initialize the Modbus Finite State Machine
  modbus_configure(&Serial, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);


//now send setpoints to all nodes in array of reg...


for (int RegisterRef=5; RegisterRef<=20; RegisterRef++){
     int NodePointer = RegisterRef%5;




if (NodePointer == 1) regs[(RegisterRef-5)] = 0; //turn off outputs

   word HighSetpoint=4000;  // need to change this to use eeprom later...this the mv value where the alarm is set off
if (NodePointer == 2) regs[(RegisterRef-5)] = HighSetpoint;
 word LowSetpoint=2500;  // need to change this to use eeprom later...this is the mv value where the alarm is set off (lower)
if (NodePointer == 3) regs[(RegisterRef-5)] = LowSetpoint;
word dumpSetpoint=3900;  // need to change this to use eeprom later..this is the mv value where the dump kicks in.
if (NodePointer == 4) regs[(RegisterRef-5)] = dumpSetpoint;  //set outputs to off.

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
for (int nodeid=0;nodeid<=3;nodeid++){
  CellVoltages[nodeid]=regs[(nodeid*5)];//keep in seperate array...

  //do do some display...
  if (CellVoltages[3]>10) digitalWrite(LED,HIGH); else digitalWrite(LED,LOW);
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
