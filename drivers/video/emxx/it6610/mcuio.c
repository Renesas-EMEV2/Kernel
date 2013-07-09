///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <mcuio.c>
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2010/12/30
//   @fileversion: CAT6611_SAMPLE_1.15
//******************************************/

#ifdef _MCU_
#include "hdmitx.h"

extern BYTE I2CADR ;
extern BYTE I2CDEV ;

void SelectHDMIDev(BYTE dev) ;
BOOL i2c_write_byte( BYTE address, BYTE offset, BYTE byteno, BYTE *p_data, BYTE device );
BOOL i2c_read_byte( BYTE address, BYTE offset, BYTE byteno, BYTE *p_data, BYTE device );


void SelectHDMIDev(BYTE dev)
{
    switch(dev)
    {
#if (TXDEVCOUNT>1)
    case 1:
        I2CDEV = TX1DEV ;
        I2CADR = TX1ADR ;
        break;
#endif
#if (TXDEVCOUNT > 2)
    case 2:
        I2CDEV = TX2DEV ;
        I2CADR = TX2ADR ;
        break;
#endif
#if (TXDEVCOUNT > 3)
    case 3:
        I2CDEV = TX3DEV ;
        I2CADR = TX3ADR ;
        break;
#endif
#if (TXDEVCOUNT > 4)
    case 4:
        I2CDEV = TX4DEV ;
        I2CADR = TX4ADR ;
        break;
#endif
#if (TXDEVCOUNT > 5)
    case 5:
        I2CDEV = TX5DEV ;
        I2CADR = TX5ADR ;
        break;
#endif
#if (TXDEVCOUNT > 6)
    case 6:
        I2CDEV = TX6DEV ;
        I2CADR = TX6ADR ;
        break;
#endif
#if (TXDEVCOUNT > 7)
    case 7:
        I2CDEV = TX7DEV ;
        I2CADR = TX7ADR ;
        break;
#endif
    case 0:
    default:
        I2CDEV = TX0DEV ;
        I2CADR = TX0ADR ;
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Start: I2C for 8051
///////////////////////////////////////////////////////////////////////////////
void set_8051_scl( BOOL bit_value )
{
     SCL_PORT = bit_value;

//	DelayUS(DELAY_TIME);
}

void set_8051_sda( BOOL bit_value, BYTE device )
{
     switch( device ) {
     	case 0:
     	    TX0_SDA_PORT = bit_value;
            break;

#ifdef _2PORT_
     	case 1:
     	    TX1_SDA_PORT = bit_value;
            break;
#endif

#ifdef _3PORT_
     	case 1:
     	    TX1_SDA_PORT = bit_value;
            break;
     	case 2:
     	    TX2_SDA_PORT = bit_value;
            break;
#endif

#ifdef _4PORT_
     	case 1:
     	    TX1_SDA_PORT = bit_value;
            break;
     	case 2:
     	    TX2_SDA_PORT = bit_value;
            break;
     	case 3:
     	    TX3_SDA_PORT = bit_value;
            break;
#endif

#ifdef _8PORT_
     	case 1:
     	    TX1_SDA_PORT = bit_value;
            break;
     	case 2:
     	    TX2_SDA_PORT = bit_value;
            break;
     	case 3:
     	    TX3_SDA_PORT = bit_value;
            break;
     	case 4:
     	    TX4_SDA_PORT = bit_value;
            break;
     	case 5:
     	    TX5_SDA_PORT = bit_value;
            break;
     	case 6:
     	    TX6_SDA_PORT = bit_value;
            break;
     	case 7:
     	    TX7_SDA_PORT = bit_value;
            break;
#endif

     	default:
     	    RX_SDA_PORT = bit_value;
     }
//	DelayUS(DELAY_TIME);
}

BOOL get_8051_sda( BYTE device )
{
     switch( device ) {
     	case 0:
     	    TX0_SDA_PORT = 1;
     	    return TX0_SDA_PORT;
            break;

#ifdef _2PORT_
     	case 1:
     	    TX1_SDA_PORT = 1;
     	    return TX1_SDA_PORT;
            break;
#endif

#ifdef _3PORT_
     	case 1:
     	    TX1_SDA_PORT = 1;
     	    return TX1_SDA_PORT;
            break;
     	case 2:
     	    TX2_SDA_PORT = 1;
     	    return TX2_SDA_PORT;
            break;
#endif

#ifdef _4PORT_
     	case 1:
     	    TX1_SDA_PORT = 1;
     	    return TX1_SDA_PORT;
            break;
     	case 2:
     	    TX2_SDA_PORT = 1;
     	    return TX2_SDA_PORT;
            break;
     	case 3:
     	    TX3_SDA_PORT = 1;
     	    return TX3_SDA_PORT;
            break;
#endif

#ifdef _8PORT_
     	case 1:
     	    TX1_SDA_PORT = 1;
     	    return TX1_SDA_PORT;
            break;
     	case 2:
     	    TX2_SDA_PORT = 1;
     	    return TX2_SDA_PORT;
            break;
     	case 3:
     	    TX3_SDA_PORT = 1;
     	    return TX3_SDA_PORT;
            break;
     	case 4:
     	    TX4_SDA_PORT = 1;
     	    return TX4_SDA_PORT;
            break;
     	case 5:
     	    TX5_SDA_PORT = 1;
     	    return TX5_SDA_PORT;
            break;
     	case 6:
     	    TX6_SDA_PORT = 1;
     	    return TX6_SDA_PORT;
            break;
     	case 7:
     	    TX7_SDA_PORT = 1;
     	    return TX7_SDA_PORT;
            break;
#endif

     	default:
     	    RX_SDA_PORT = 1;
     	    return RX_SDA_PORT;
     }
}

void i2c_8051_start( BYTE device )
{
    set_8051_sda( HIGH, device );
    set_8051_scl( HIGH );
    set_8051_sda( LOW, device );
    set_8051_scl( LOW );
}

void i2c_8051_write( BYTE byte_data, BYTE device )
{
 BYTE data bit_cnt, tmp;
 BOOL data bit_value;

     for(bit_cnt=0; bit_cnt<8; bit_cnt++) {
         tmp = (byte_data << bit_cnt) & 0x80;
		 bit_value = tmp && 0x80;

         set_8051_sda( bit_value, device );
         set_8051_scl( HIGH );
         set_8051_scl( LOW );
     }
}

BOOL i2c_8051_wait_ack( BYTE device )
{
 BOOL data ack_bit_value;

    set_8051_sda( HIGH, device );
    set_8051_scl( HIGH );
    ack_bit_value = get_8051_sda( device );
    set_8051_scl( LOW );

    return ack_bit_value;
}

BYTE i2c_8051_read( BYTE device )
{
 BYTE data bit_cnt, byte_data;
 BOOL data bit_value;

     byte_data = 0;
     for(bit_cnt=0; bit_cnt<8; bit_cnt++) {
         set_8051_scl( HIGH );

	     bit_value = get_8051_sda( device );

	     byte_data = (byte_data << 1) | bit_value;

	     set_8051_scl( LOW );
     }

     return byte_data;
}

void i2c_8051_send_ack( BOOL bit_value, BYTE device )
{
     set_8051_sda( bit_value, device );
     set_8051_scl( HIGH );
     set_8051_scl( LOW );
     set_8051_sda( HIGH, device );
}

void i2c_8051_end( BYTE device )
{
     set_8051_sda( LOW, device );
     set_8051_scl( HIGH );
     set_8051_sda( HIGH, device );
}

BOOL i2c_write_byte( BYTE address, BYTE offset, BYTE byteno, BYTE *p_data, BYTE device )
{
 BYTE data i;

     i2c_8051_start(device);				// S

     i2c_8051_write(address&0xFE, device);		// slave address (W)
     if( i2c_8051_wait_ack(device)==1 )	{		// As
         i2c_8051_end(device);
	 return 0;
      }

     i2c_8051_write(offset, device);			// offset
     if( i2c_8051_wait_ack(device)==1 )	{		// As
         i2c_8051_end(device);
	 return 0;
     }

     for(i=0; i<byteno-1; i++) {
     	 i2c_8051_write(*p_data, device);		// write d
     	 if( i2c_8051_wait_ack(device)==1 ) {		// As
     	     i2c_8051_end(device);
	     return 0;
         }
         p_data++;
     }

     i2c_8051_write(*p_data, device);			// write last d
     if( i2c_8051_wait_ack(device)==1 )	{		// As
     	 i2c_8051_end(device);
	 return 0;
     }
     else {
     	 i2c_8051_end(device);
	 return 1;
     }
}

BOOL i2c_read_byte( BYTE address, BYTE offset, BYTE byteno, BYTE *p_data, BYTE device )
{
 BYTE data i;

     i2c_8051_start(device);				// S

     i2c_8051_write(address&0xFE, device);		// slave address (W)
     if( i2c_8051_wait_ack(device)==1 ) {		// As
         i2c_8051_end(device);
         return 0;
     }

     i2c_8051_write(offset, device);			// offset
     if( i2c_8051_wait_ack(device)==1 ) {		// As
         i2c_8051_end(device);
         return 0;
     }

     i2c_8051_start(device);

     i2c_8051_write(address|0x01, device);		// slave address (R)
     if( i2c_8051_wait_ack(device)==1 ) {		// As
         i2c_8051_end(device);
         return 0;
     }

     for(i=0; i<byteno-1; i++) {
         *p_data = i2c_8051_read(device);		// read d
         i2c_8051_send_ack(LOW, device);		// Am

         p_data++;
     }

     *p_data = i2c_8051_read(device);			// read last d
     i2c_8051_send_ack(HIGH, device);			// NAm
     i2c_8051_end(device);

    return 1;
}


///////////////////////////////////////////////////////////////////////////////
// I2C for original function call
///////////////////////////////////////////////////////////////////////////////
BYTE HDMITX_ReadI2C_Byte(BYTE RegAddr)
{
 BYTE data p_data;

 i2c_read_byte(I2CADR, RegAddr, 1, &p_data, I2CDEV);

 return p_data;
}

SYS_STATUS HDMITX_WriteI2C_Byte(BYTE RegAddr, BYTE d)
{
 BOOL data flag;

 flag = i2c_write_byte(I2CADR, RegAddr, 1, &d, I2CDEV);

 return !flag;
}

SYS_STATUS HDMITX_ReadI2C_ByteN(BYTE RegAddr, BYTE *pData, int N)
{
 BOOL data flag;

 flag = i2c_read_byte(I2CADR, RegAddr, N, pData, I2CDEV);

 return !flag;
}

SYS_STATUS HDMITX_WriteI2C_ByteN(SHORT RegAddr, BYTE *pData, int N)
{
 BOOL data flag;

 flag = i2c_write_byte(I2CADR, RegAddr, N, pData, I2CDEV);

 return !flag;
}

///////////////////////////////////////////////////////////
// Function Body
///////////////////////////////////////////////////////////
void
DelayMS(USHORT ms)
{
	USHORT i;
    BYTE j;

    for(i=0; i<ms; i++)
       for( j=0; j<122; j++ );

}

#if 0
void
delay1ms(UINT count)
{
UINT i;
BYTE j;

    for(i=0; i<count; i++)
       for( j=0; j<122; j++ );
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Start: I2C for 8051
///////////////////////////////////////////////////////////////////////////////

#endif // _MCU_
