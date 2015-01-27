#ifndef atmel_spektrumSerial_h
#define atmel_spectrumSerial_h

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Spektrum Serial /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>

#define SSO_FRAME_SIZE 16
#define SSO_MAX_VAL 0x3FF // 1023

volatile uint8_t sso_frame[SSO_FRAME_SIZE];
volatile uint8_t frame_counter;
volatile uint8_t frame_send;

void spektrumSerial_init(void)
{
    int i;

    /* frame loss count */
    sso_frame[0] = 0x00; // Let's pretend we are not loosing any frames
    /* Receiver identifier */
    sso_frame[1] = 0x01; // Let's pretend we are DX6i or similar

    // Set all channel data to 0
    for (i=0; i<7; i++)
    {
        //sso_frame[2+(i*2)] = (sso_channel_map[i] << 2);
        sso_frame[2+(i*2)] = ((i&7) << 2);
        sso_frame[2+(i*2)+1] = 0x00;
    }

    /* Enable USART subsystem */
    UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);

    /* Setup baud rate */
    //#define UART_BAUD_RATE 115200 // 16.0MHz
    //#define UART_BAUD_RATE 112390 // 16.4MHz
    //#define UART_BAUD_RATE 110371 // 16.7MHz
    //#define UART_BAUD_RATE 108423 // 17.0MHz
    /*
    Clock_send=16.4-16.7 measured by Oszi
    Clock_kkbo=19.2-20.0 both very temperaturestable!
    Working Range: 16.6-18.4 MHz, Test in 0.1Steps --> Middle 17.5
    */
#define UART_BAUD_RATE 105324 // 17.5MHz

#define UART_BAUD_SELECT ((F_CPU/UART_BAUD_RATE/16)-1)
    UBRR0L=((unsigned char)UART_BAUD_SELECT);

    UDR0 = 0xAA;

    /* configure second pin to be output for serial out */
    DDRD |= (1 << 1); // TX

    frame_counter = 0;
    frame_send = 0;
    asm("sei");
}

void spektrumSerial_send(unsigned int *channel_data)
{
    int i;
    unsigned int temp_val;

    for (i=0; i<6; i++)
    {
        if (channel_data[i] <= 0)
        {
            temp_val = 0;
        }
        else if (channel_data[i] >= SSO_MAX_VAL)
        {
            temp_val = SSO_MAX_VAL;
        }
        else
        {
            temp_val = channel_data[i];//&SSO_MAX_VAL;
        }
        //sso_frame[2+(i*2)] = (sso_channel_map[i] << 2) | (temp_val>>8);
        sso_frame[2+(i*2)] = ((i&7) << 2) | (temp_val>>8);
        sso_frame[2+(i*2)+1] = temp_val & 0xFF;
    }

    /* Just enable the serial interrupt the rest will be taken care of */
    frame_send = 0;
    UCSR0B|=(1<<TXCIE0);
}

ISR(USART_TX_vect)
{
    asm("sei");
    UDR0 = sso_frame[frame_counter];
    frame_counter++;

    if (frame_counter >= SSO_FRAME_SIZE)   /* Frame is over, let's disable ourselves */
    {

        /* reset */
        frame_counter = 0;

        /* disable */
        UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);

        frame_send = 1;
    }
}

#endif
