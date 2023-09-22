/*
 * File:   IOCB.c
 * Author: Gabri
 *
 * Created on 22 de agosto de 2023, 06:19 AM
 */


#include "IOCB.h"

void iocb_init(uint8_t pinesB)
{
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;             //Global Interrupt Enable
    INTCONbits.T0IE = 0;
    INTCONbits.INTE = 1;
    INTCONbits.RBIE = 1;            //PORTB Interrupt on Change
    INTCONbits.T0IF = 0;
    INTCONbits.INTF = 0;
    INTCONbits.RBIF = 0;   
    
    TRISB |= pinesB;    //RBx as Input    
    nRBPU = 0;          //Enable PORTB pull-up's
    WPUB |= pinesB;     //Set RBx pull-up's 
    IOCB |= pinesB;     //RBx Interrupt Enable         
}
