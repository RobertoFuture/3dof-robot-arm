/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.6
        Device            :  PIC18F47K42
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include <math.h>
#define pi 3.141592

uint16_t phi, mappedphi;        //base
uint16_t theta, mappedtheta;    //shoulder
uint16_t psi, mappedpsi;        //elbow
uint16_t gamma, mappedgamma;    //tweezers

//medidas del brazo
double d = 36; //(mm) altura desde la base al hombro
double h = 18.5; //1/2 de la altura del objeto: 37mm
double a = 119; //(mm) antebrazo
double b = 110; // (mm) brazo 
double c = 90; // (mm) pinza

uint16_t map(int value, int inputMin, int inMax, int outMin, int outMax);
void servo1();
//void servo2(); aea mongol
//void servo3();
//void servo4();
void Stab();
void ChargeDutyValue();
void move();
void Initial();
void Contraction();
void PickUp();
void sendNotice();

void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();
    
    Initial();
    ChargeDutyValue();
    __delay_ms(1500);
    
    while (1)
    {   
        move();
        if(getch() == 'j'){  
            PickUp();
            __delay_ms(3000);
            sendNotice();
        }
        if(getch() == 't'){  
            Initial();
            ChargeDutyValue();
            __delay_ms(1500);
            sendNotice();
        }
    }
}

uint16_t map(int value, int inputMin, int inMax, int outMin, int outMax){
    return (value - inputMin) * (outMax - outMin) / (inMax - inputMin) + outMin;
} 

void sendNotice(){
    putch('s');
}

void servo1(){ //BASE SERVO
    if(getch() == 'b'){     //clockwise
        phi++;
        mappedphi = map(phi,-70,70,77,30);
        PWM1_LoadDutyValue(mappedphi);
        __delay_ms(50);
        sendNotice();
    }
    if(getch() == 'a'){     //conterclockwise
        phi--;
        mappedphi = map(phi,-70,70,77,30);
        PWM1_LoadDutyValue(mappedphi);
        __delay_ms(50);
        sendNotice();
    }
}
/*
void servo2(){ //SHOULDER SERVO
    if(getch() == 'e'){
        theta++;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
    }
    if(getch() == 'c'){
        theta--;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
    }
}

void servo3(){ //ELBOW SERVO
    if(getch() == 'f'){
        psi++;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);
    }
    if(getch() == 'd'){
        psi--;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);  
    }
}

void servo4(){  //TWEEZERS SERVO
    if(getch() == 'g'){
        gamma++;
        mappedgamma = map(gamma,-70,70,77,30);
        PWM4_LoadDutyValue(mappedgamma);
        __delay_ms(50);
    }
    if(getch() == 'h'){
        gamma--;
        mappedgamma = map(gamma,-70,70,77,30);
        PWM4_LoadDutyValue(mappedgamma);
        __delay_ms(50);
    }
}
*/
void Stab(){
    if(getch() == 'c'){     //backwards
        theta--;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
        psi--;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);
        sendNotice();
    }
    if(getch() == 'd'){     //fordwards
        theta++;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
        psi++;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);
        sendNotice();
    }
}

void ChargeDutyValue(){
    mappedphi = map(phi,-70,70,77,30);
    PWM1_LoadDutyValue(mappedphi);
    mappedtheta = map(theta,-70,70,77,30);
    PWM2_LoadDutyValue(mappedtheta);
    mappedpsi = map(psi,-70,70,77,30);
    PWM3_LoadDutyValue(mappedpsi);
    mappedgamma = map(gamma,-70,70,77,30);
    PWM4_LoadDutyValue(mappedgamma);
}

void move(){          
    servo1();
    Stab();
}

void Initial(){
    phi = 0;        
    theta = -52;      
    psi = -7;        
    gamma = 45; 
}

void Contraction(){
    phi = 0;        
    theta = -70;      
    psi = -43;        
    gamma = -10; 
}

void Down(){
    int alpha_0;
    int alpha;
    int mu;
    int lambda;
    double radalpha_0;
    double radalpha;
    double radmu;
    double radlambda;
    double A = a;
    double B = b + c;
    double D = d - h;
    double C;
    double longTerm;

    alpha_0 = 45 - theta;
    radalpha_0 = alpha_0*(pi/180);
    C = b + a*cos(radalpha_0);
    longTerm = -pow(A,4)+2*pow(A*B,2)+2*pow(A*C,2)+2*pow(A*D,2)-pow(B,4);
    longTerm += 2*pow(B*C,2)+2*pow(B*D,2)-pow(C,4)-2*pow(C*D,2)-pow(D,4);
    radalpha = 2*(atan((-sqrt(longTerm)-2*A*D)/(pow(A,2)-pow(B,2)+2*A*C+pow(C,2)+pow(D,2))));
    radmu = 2*(atan((sqrt(longTerm)-2*B*D)/(pow(A,2)-pow(B,2)-2*B*C-pow(C,2)-pow(D,2))));
    radlambda = asin((D+C-b)/B);
    alpha = (int)(radalpha*(180/pi));
    mu = (int)(radmu*(180/pi));
    lambda = (int)(radlambda*(180/pi));
    
    if (cos(radalpha_0) >= cos(35*(pi/180))) {      // Lejos
        psi = 90 - lambda - alpha_0;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);
        theta = 45 - alpha_0;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
    }
    else {     // Cerca
        psi = 90 - mu - alpha;
        mappedpsi = map(psi,-70,70,77,30);
        PWM3_LoadDutyValue(mappedpsi);
        __delay_ms(50);
        theta = 45 - alpha;
        mappedtheta = map(theta,-70,70,77,30);
        PWM2_LoadDutyValue(mappedtheta);
        __delay_ms(50);
    }
}

void PickUp(){
    gamma=-10;
    ChargeDutyValue();
    __delay_ms(1500);
    Down();
    __delay_ms(1500);
    gamma=45;
    ChargeDutyValue();
    __delay_ms(1500);
    phi=-55; 
    theta=-45;
    psi=0;
    ChargeDutyValue();
    __delay_ms(1500);
    gamma=-10;
    ChargeDutyValue();
    __delay_ms(1500);
    Contraction();
    ChargeDutyValue();
    __delay_ms(2000);
    Initial();
    ChargeDutyValue();
}