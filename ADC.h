/* 
 * File:   ADC.h
 * Author: Gabriel Cipriano
 *
 * Created on 21 de julio de 2023
 */

#ifndef ADC_H
#define	ADC_H

#include <xc.h>
// Configuracion del ADC
void adc_init(int channel);
int adc_read ();
void adc_change_channel(int channel);
int adc_get_channel();

#endif	/* AD
