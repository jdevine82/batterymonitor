#include <SimpleModbusSlave.h>
#include <string.h>
/* 
 This code is intended for a arduino nano. It uses the hardware serial port to connect to a master over a propietry hardware bus of open collector optocouplers. It uses the modbus protocol.
 this does create some issues as the modbus protocol only uses 16bit registers so everyting is in mv. 
 2 cells are implemented on each node. 
 reg      description
 0 Cell1Voltage,  //cell voltage in mv 

 1  Node1outputs, //this is a register for the dump loads. bit0 is cell output
 2 Node1TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv

 3 Node1BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv

 4 Node1HDumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv


 The slave code needs to measure the hardware ADC, do some averaging and map into the modbus reg for reading by the master.
 Also the voltages need to be compared to the floats given by the master. If the dump load needs turning on, or the cell volatage is too high or too low turn on the hardware alarm so that 
 the inverter and charger will get turned off in the case if life batteries.

 hardware connections
 0- TX
 1- RX
 2- reserved for tx enable if ever needed....
 ADC0 lower cell  voltage read
 ADC1 upper cell voltage read
 3- dump output for lower cell
 4- dump output for upper cell
 5- alarm output for high/low battery
*/



#define LowerCellDump 3 //hardware pin
#define UpperCellDump 4 //hardware pin
#define AlarmOutput 5 //hardware pin
// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

unsigned int scratchpad1, scratchpad2;

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
 Cell1Voltage,  //voltage in mv
  Cell1outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Cell1TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Cell1BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Cell1DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv    
   Cell2Voltage,  //voltage in mv
  Cell2outputs, //this is a register for the dump loads. bit0 is lower cell, bit1 is upper cell.
  Cell2TopSetpoint, //point that alarm turns on high bytes; used as float on slave in mv
  Cell2BottomSetpoint,  /// point that low voltage alarm goes off low byte; used as float on slave in mv
  Cell2DumpSetpoint,   // point that dump resistor turn on High bytes. part of float in mv 
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

void setup()
{
  /* parameters(HardwareSerial* SerialPort,
                long baudrate, 
		unsigned char byteFormat,
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned int* holding register array)
  */
  
  /* Valid modbus byte formats are:
     SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
     SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
     SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
     
     You can obviously use SERIAL_8N1 but this does not adhere to the
     Modbus specifications. That said, I have tested the SERIAL_8N1 option 
     on various commercial masters and slaves that were suppose to adhere
     to this specification and was always able to communicate... Go figure.
     
     These byte formats are already defined in the Arduino global name space. 
  */
	
  modbus_configure(&Serial, 9600, SERIAL_8N2, 0, 10, HOLDING_REGS_SIZE, holdingRegs);  //slave id will need to change some code to read hardware pins and set this

  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  modbus_update_comms(9600, SERIAL_8N2, 0);   //slave id will need to change some code to read hardware pins and set this
  
  pinMode(LowerCellDump, OUTPUT);
  pinMode(UpperCellDump, OUTPUT);
  pinMode(AlarmOutput, OUTPUT);
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  modbus_update();
  word cellvoltage=GetAveragePinReads(0);  //read and aveage voltage then put into register
  if (cellvoltage>holdingRegs[Cell1TopSetpoint]) digitalWrite(AlarmOutput,HIGH);else digitalWrite(AlarmOutput,LOW); //turn on alarm for external shutdown
   if (cellvoltage<holdingRegs[Cell1BottomSetpoint]) digitalWrite(AlarmOutput,HIGH);else digitalWrite(AlarmOutput,LOW); //turn on alarm for external shutdown
     if (cellvoltage>holdingRegs[Cell1DumpSetpoint])  digitalWrite(LowerCellDump,HIGH); else digitalWrite(LowerCellDump,LOW); //turn on alarm for external shutdown
holdingRegs[Cell1Voltage]=cellvoltage;  //put into register 
 cellvoltage=GetAveragePinReads(1);  //read and aveage voltage then put into register
  if (cellvoltage>holdingRegs[Cell2TopSetpoint]) digitalWrite(AlarmOutput,HIGH);else digitalWrite(AlarmOutput,LOW); //turn on alarm for external shutdown
   if (cellvoltage<holdingRegs[Cell2BottomSetpoint]) digitalWrite(AlarmOutput,HIGH);else digitalWrite(AlarmOutput,LOW); //turn on alarm for external shutdown
     if (cellvoltage>holdingRegs[Cell2DumpSetpoint]) digitalWrite(UpperCellDump,LOW); else digitalWrite(UpperCellDump,HIGH); //turn on alarm for external shutdown
holdingRegs[Cell2Voltage]=cellvoltage;  //put into register 

}

//really need to change this so that it is done on a hardware timer interrupt and continueously running in background...
 word GetAveragePinReads(int Sense){
 int Ctr=0;
unsigned long SumOfReads=0.0;
  // get the prescribed number of samples
while (Ctr < 1000){
  SumOfReads = SumOfReads + analogRead(Sense);
  Ctr++;}
//Calculate and return the average in millivolts
return word((SumOfReads/Ctr)/1.023*5);
}


