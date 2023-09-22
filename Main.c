/*
 * File:   Gas.c
 * Author: Gabri
 *
 * Created on 15 de agosto de 2023, 08:19 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#include <xc.h>
#include "ADC.h"
#include "I2C.h"

#define _tmr0_n 255 //TMR0 load value

//Variables definidas
//int ADC1, ADC2;
uint8_t z;
uint8_t flama;
uint8_t TMR0_count;
uint8_t pulse_width = 255; //Manual PWM pulse width 

void setup(void);

void __interrupt() isr(void){
    //if(T0IF == 1){  //TMR0
    //    TMR0 = _tmr0_n;
    //    TMR0_count = TMR0_count+45;
    //   T0IF = 0;
    //}
       
    if(PIR1bits.SSPIF == 1)
    {
        SSPCONbits.CKP = 0;
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }
        
        //Lectura del Esclavo y Escritura del maestro
        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW)
        {
            //__delay_us(7);
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            //__delay_us(2);
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupci?n recepci?n/transmisi?n SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepcion se complete
        //    mode = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepci?n
            __delay_us(250);
        }
        
        //Escritura del Esclavo y Lectura del Maestro
        if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW)
        {
            z = SSPBUF;
            BF = 0;
            SSPBUF = flama;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);

        }        
        PIR1bits.SSPIF = 0;
    }
}

void main(void) {
    setup();
    while(1){
        
        if (PORTBbits.RB0 == 1)
        {
            __delay_ms(20);
            if (PORTBbits.RB0 == 1){
                while(RB0 == 1){
                    flama = 1;
                    PORTA = 0b00000111;
                    __delay_ms(20);
                    PORTA = 0b00000100;
                    __delay_ms(20);     
                }

            }
        }else if(PORTBbits.RB0 == 0){
            
            __delay_ms(20);
            if (PORTBbits.RB0 == 0){
                while(RB0 == 0){
                    flama = 0;
                    PORTA = 0b00000000;
                    
                }

            }
        }
    }
    return;
}


void setup(void)
{
    ANSEL = 0;
    ANSELH = 0;
    
    TRISA = 0;
    TRISB = 0b00000001;
    TRISD = 0;
    TRISCbits.TRISC5 = 0; //RC5 Output
    
    PORTA = 0;
    PORTD = 0;
    PORTB = 0;
    PORTC = 0;
    
    // Oscilador 
    OSCCONbits.IRCF =0b111; 
    OSCCONbits.SCS = 1; 
    
    //Configuration of Timer
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz 
    OSCCONbits.SCS = 1;
    
    //configuración del TMR0 
    OPTION_REGbits.PS = 0b100;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.T0CS = 0;    
    TMR0 = _tmr0_n;
    
    //INTCONbits.GIE = 1;         // Habilitamos interrupciones
    //INTCONbits.PEIE = 1;        // Habilitamos interrupciones PEIE
    I2C_Slave_Init(0x60);   
    return;
}
