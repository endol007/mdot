#ifndef RS485_H
#define RS485_H
 
#include "BufferedSerial.h"
#include "mbed.h"
 
/** RS485 Library
 *  
 * Can send from 1 to 255 bytes from one node to another with:
 *
 * Packet start indicator (STX)
 * Each data byte is doubled and inverted to check validity
 * Packet end indicator (ETX)
 * Packet CRC (checksum)
 *
 * Using MAX485 modules or the MAX485 CSA+
 *
 * Example:
 * @code
 *  #include "mbed.h" 
 *  #include <RS485.h>
 *  Serial pc(USBTX, USBRX); 
 *  RS485 RS485(PC_10,PC_11,PB_3); // Tx, Rx , !RE and DE MAX485 pin
 * 
 *  DigitalOut ho(PB_3); // this pin should be connected to !RE and DE
 *  typedef uint8_t byte;
 * 
 *  byte regvalue[9];
 *  byte data[9] = {0x01,0x04,0x00,0x48,0x00,0x02,0xf1,0xdd};//your data
 *  int main()
 *  {
 *    pc.printf("main\n");
 *    while(1) {
 *       pc.printf("Starting\n");
 *       ho = 1;                  // Enable sending on MAX485
 *       RS485.sendMsg(data,sizeof(data));
 *       wait_ms(600);            // Must wait for all the data to be sent   
 *       ho = 0;                  // Enable receiving on MAX485
 *       pc.printf("Getting data\n");
 *       if(RS485.readable() >0){
 *           memset(regvalue,0,sizeof(regvalue));
 *           wait_ms(200);
 *           RS485.recvMsg(regvalue,sizeof(data),500);
 *           wait_ms(200);
 *           for (int count = 0; count < 9; count++) {
 *               pc.printf("%X - ", regvalue[count]);
 *           }
 *       }else printf("No Data\n");
 *       printf("Done\n");
 *       wait_ms(1000);
 *    }
 *  } 
 * @endcode
 */
 
 
/**
 *  @class RS485
 *  @communicating 
 */
 
    class RS485 : public BufferedSerial
{
private:
    typedef unsigned int word;
    typedef uint8_t byte;
    typedef uint8_t boolean;
    typedef void (*voidFuncPtr)(void);
    
public:
    /** Create a BufferedSerial port, connected to the specified transmit and receive pins
     *  @param tx Transmit pin
     *  @param rx Receive pin
     *  @param dere Enable pin, this pin should be connected to !RE and DE
     *  @note uses BufferedSerial
     */
    RS485(PinName tx, PinName rx, PinName dere);
 
    /** calculate 8-bit CRC
    *  cyclic redundancy check
    *  @param addr byte pointer of information to use (typical an byte array)
    *  @param len length of byte of information were converting
    *  @return the CRC byte
    */
    static byte crc8 (const byte *addr, byte len);
    
    /** sendComplemented byte
    *  send a byte complemented, repeated
    *       only values sent would be (in hex):
    *       0F, 1E, 2D, 3C, 4B, 5A, 69, 78, 87, 96, A5, B4, C3, D2, E1, F0
    *  @what the byte to complement
    */
    void sendComplemented (const byte what);
    
    /** send message
    *  cyclic redundancy check
    *  @param data the data to be sent through RS485
    *  @param length length of the data 
    *  @note puts STX at start, ETX at end, and add CRC
    */   
    void sendMsg (const byte * data, const byte length);
    
    /** receive message
    *  reads serial port and populates data
    *  @param data buffer to receive into
    *  @param length length of the data 
    *  @param timeout clock_mseconds before timing out
    *  @return the number of bytes received
    *  
    */
    byte recvMsg (byte * data, const byte length, unsigned long timeout);
      
 
};
#endif