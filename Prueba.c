/*
 * File:   PostLab9.c
 * Author: Andrea Rodriguez Zea
 *
 * Created on April 29, 2022, 10:46 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 4000000
#define _tmr0_value 250                                     // Delay

uint8_t counter;                                            // Se declara la variable para contador
uint8_t pot3;                                               // Variable para uso de potenciometro

void setup(void);

void __interrupt() isr (void)                               // Interrupciones
{
    if (PIR1bits.ADIF)                                      // Interrupción en ADC
    {
         if (ADCON0bits.CHS == 0){
            CCPR1L = (ADRESH>>1)+30;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = ADRESL >> 7;
         }
           
        else if (ADCON0bits.CHS == 1){
            CCPR2L = (ADRESH>>1)+45;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = ADRESL >> 7;
        }
        else
            pot3 = ADRESH;
        PIR1bits.ADIF = 0;
    }

    if(T0IF == 1){                                          // TMR0 interrupcion
        INTCONbits.T0IF = 0;
        counter ++;                                         // Si existe un aumento en timer volver el nuevo
        TMR0 = _tmr0_value;
        
        if (counter < pot3)                                 // Relación de valores para el encendido del LED
            PORTDbits.RD0 = 1;
        else
            PORTDbits.RD0 = 0;                              // Si la relación no es verdad, se deja apagado el LED
   
        INTCONbits.T0IF = 0;
    }    
}

void main(void) {
    setup();
    while(1){
    
        if (ADCON0bits.GO == 0) {                            // Loop general
            if (ADCON0bits.CHS == 0)
                ADCON0bits.CHS = 1;
            else if (ADCON0bits.CHS == 1)
                ADCON0bits.CHS = 2;
            else
                ADCON0bits.CHS = 0;

            __delay_us(1000);
            ADCON0bits.GO = 1;    
        }      
    }        
}

void setup (void){
  
    ANSEL = 0b00000111;                                     //AN0 como entradas analógicas
    ANSELH = 0;
    
    TRISA = 0b00000111;                                     // PORTA como salida, RA0 & RA1 como entradas 
    PORTA = 0;                                              // Limpiamos PORTA 
    
    TRISD = 0;
    PORTD = 0;
    
    // OSC CONFIG
    OSCCONbits.IRCF = 0b0110;                               // IRCF <2:0> -> 111 4 MHz
    OSCCONbits.SCS = 1;                                     // Oscilador interno

    // TMR0 CONFIG
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.T0SE = 0;
    OPTION_REGbits.PSA = 0;                                 //Un TMR0 con un Prescaler 1:256
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 0;     
    TMR0 = _tmr0_value;                                     // Reiniciamos el TMR0 a 217 para quedarnos con el retardo
        
    INTCONbits.T0IE = 1;                                    // Habilitamos interrupciones del TMR0
    INTCONbits.T0IF = 0;                                    // Limpiamos bandera de interrupción del TMR0
        
    // ADC CONFIG
    ADCON0bits.ADCS = 0b01;                                 // ADCS <1:0> -> 10 FOSC/32
    ADCON0bits.CHS = 0;                                     // CHS  <3:0> -> 0000 AN0
    ADCON1bits.ADFM = 0;                                    // Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;                                   // Referencia en VDD
    ADCON1bits.VCFG1 = 0;                                   // Referencia en VSS
    
    
    // PWM CONFIG
    
    TRISCbits.TRISC2 = 1;                                   // RC2 -> CCP1 como entrada
    TRISCbits.TRISC1 = 1;                                   // RC1 -> CCP2 como entrada
    PR2 = 255;                                              // Periodo del TIMER2
    CCP1CONbits.P1M = 0;                                    // Salida simple
    CCP1CONbits.CCP1M = 0b1100;                             // asignación del modo a PWM1
    CCP2CONbits.CCP2M = 0b1100;                             // asignación del modo a PWM2
            
    CCPR1L = 0x0F;                                          // Valor inicial del duty cycle
    CCP1CONbits.DC1B = 0;                                   // CONFIG bits menos significativos
    
    CCPR2L = 0x0F;                                          // Valor inicial del duty cycle
    CCP2CONbits.DC2B0 = 0;                                  // CONFIG bits menos significativos
    
    // TIMER2 CONFIG
    PIR1bits.TMR2IF = 0;                                    // Flag del TIMER2 en 0    
    T2CONbits.T2CKPS = 0b11;                                // Prescaler 1:16
    T2CONbits.TMR2ON = 1;                                   // Encendemos TIMER2
    
    while (PIR1bits.TMR2IF == 0);                           // Esperamos una interrupción del TIMER2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;                                   // RC2 -> CCP1 como salida del PWM
    TRISCbits.TRISC1 = 0;                                   // RC1 -> CCP2 como salida
    
    // INT CONFIG
    PIE1bits.ADIE = 1;                                      // Habilitamos interrupciones del ADC  
    PIR1bits.ADIF = 0;                                      // Flag del ADC en 0
    INTCONbits.PEIE = 1;                                    // Habilitamos interrupciones de los puertos
    INTCONbits.GIE = 1;                                     // Habilitamos interrupciones globales
    
    __delay_us(50);
    ADCON0bits.ADON = 1;                                    // Encender ADC
    
    return;
   
}