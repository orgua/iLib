#ifndef atmel_spektrumSerial_h
#define atmel_spectrumSerial_h

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Spektrum Serial /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
// forked from: https://github.com/esden/ppm_to_spektrum_encoder


#include <avr/io.h>
#include <avr/interrupt.h>

// Choose only one of the two systems
#define USE_SPEKTRUM2048 // otherwise SPEKTRUM1024 is used
// TODO: define a workaround to compensate if you just want more channels, but no higher resolutiono

#define SPEKTRUM_FRAME_SIZE 	16


#ifdef USE_SPEKTRUM2048 // 11 bit frames
const uint16_t  SPEKTRUM_VAL_MASK       = 0x7FF; // 2047
const uint8_t   SPEKTRUM_CHAN_COUNT     = 12;
const uint8_t   SPEKTRUM_CHAN_SHIFT     = 3;
const uint8_t   SPEKTRUM_CHAN_MASK      = 0x0F;
const uint8_t   SPEKTRUM_CHAN_STATIC    = 4; // first # Channels get transmitted every time, the others on change
#else                   // 10 bit frames
const uint16_t  SPEKTRUM_VAL_MASK       = 0x3FF; // 1023
const uint8_t   SPEKTRUM_CHAN_COUNT     = 7;
const uint8_t   SPEKTRUM_CHAN_SHIFT     = 2;
const uint8_t   SPEKTRUM_CHAN_MASK      = 0x07;
#endif // USE_SPEKTRUM2048


volatile uint8_t spektrum_frame[SPEKTRUM_FRAME_SIZE];
volatile uint8_t frame_counter;
volatile uint8_t frame_send;


void spektrumSerial_init(void)
{
    /* frame loss count */
    spektrum_frame[0] = 0x00; // Let's pretend we are not loosing any frames
    /* Receiver identifier */
    spektrum_frame[1] = 0x01; // Let's pretend we are DX6i or similar

    // Set all initial channel data to 0
    for (uint8_t i=0; i<7; i++)
    {
        spektrum_frame[2+(i*2)]   = ((i&SPEKTRUM_CHAN_MASK) << SPEKTRUM_CHAN_SHIFT);
        spektrum_frame[2+(i*2)+1] = 0x00;
    }

    /* Enable USART subsystem */
    UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);

    /* Setup baud rate */
    //#define UART_BAUD_RATE 115200 // 16.0MHz
    //#define UART_BAUD_RATE 112390 // 16.4MHz
    //#define UART_BAUD_RATE 110371 // 16.7MHz
    //#define UART_BAUD_RATE 108423 // 17.0MHz
    //#define UART_BAUD_RATE 105324 // 17.5MHz
    const uint32_t UART_BAUD_RATE   = 108200;
    const uint32_t UART_BAUD_SELECT = ((F_CPU/UART_BAUD_RATE/16)-1);

    UBRR0L=((unsigned char)UART_BAUD_SELECT);

    UDR0 = 0xAA;

    /* configure second pin to be output for serial out */
    DDRD |= (1 << 1); // TX

    frame_counter = 0;
    frame_send = 0;
    asm("sei");
}



void spektrumSerial_send(uint16_t *channel_data)
{
    uint16_t    temp_val;
    uint8_t     temp_i;

#ifdef USE_SPEKTRUM2048
    static uint8_t  position_newVal  = {SPEKTRUM_CHAN_STATIC};
    static uint8_t  position_sameVal = {SPEKTRUM_CHAN_STATIC};
    uint8_t         position_start;
    static uint16_t old_val[SPEKTRUM_CHAN_COUNT-SPEKTRUM_CHAN_STATIC];
    uint16_t        bitmap_selected  = {0};

    // make the position_newVal run through the Channels and until position_start is
    if (position_newVal>SPEKTRUM_CHAN_STATIC) position_start = position_newVal - 1;
    else                                      position_start = SPEKTRUM_CHAN_COUNT - 1;
#endif // USE_SPEKTRUM2048


    for (uint8_t i=0; i<7; i++)
    {
#ifdef USE_SPEKTRUM2048
        if (i < SPEKTRUM_CHAN_STATIC)      // handle the channels that get transmitted every time
        {
            temp_i = i;
        }
        else
        {
            temp_i = 0xFF;
            while(temp_i == 0xFF)
            {
                if (position_start != position_newVal) // run through the remaining channels and find changed ones
                {
                    if (old_val[position_newVal-SPEKTRUM_CHAN_STATIC] != (channel_data[position_newVal] & SPEKTRUM_VAL_MASK))
                    {
                        old_val[position_newVal-SPEKTRUM_CHAN_STATIC] = (channel_data[position_newVal] & SPEKTRUM_VAL_MASK);
                        temp_i = position_newVal;
                        bitSet(bitmap_selected, position_newVal);
                    }
                    if (++position_newVal >= SPEKTRUM_CHAN_COUNT)           position_newVal = SPEKTRUM_CHAN_STATIC;
                }
                else    // fill the remaining places with unchanged channels
                {
                    if (bitRead(bitmap_selected,position_sameVal) == 0)     temp_i = position_sameVal;
                    if (++position_sameVal >= SPEKTRUM_CHAN_COUNT)          position_sameVal = SPEKTRUM_CHAN_STATIC;
                }
            }
        }
#else
        temp_i = i;
#endif // USE_SPEKTRUM2048

        if 		(channel_data[temp_i] <= 0) 				temp_val = 0;
        else if (channel_data[temp_i] >= SPEKTRUM_VAL_MASK)	temp_val = SPEKTRUM_VAL_MASK;
        else										        temp_val = channel_data[temp_i];

        spektrum_frame[2+(i*2)] = ((temp_i&SPEKTRUM_CHAN_MASK) << SPEKTRUM_CHAN_SHIFT) | (temp_val>>8);
        spektrum_frame[2+(i*2)+1] = temp_val & 0xFF;

    }



    /* Just enable the serial interrupt the rest will be taken care of */
    frame_send = 0;
    UCSR0B|=(1<<TXCIE0);
}

ISR(USART_TX_vect)
{
    asm("sei");
    UDR0 = spektrum_frame[frame_counter];
    frame_counter++;

    if (frame_counter >= SPEKTRUM_FRAME_SIZE)   /* Frame is over, let's disable ourselves */
    {
        /* reset */
        frame_counter = 0;
        /* disable */
        UCSR0B=(0<<RXCIE0)|(0<<TXCIE0)|(0<<RXEN0)|(1<<TXEN0);
        frame_send = 1;
    }
}

#endif
