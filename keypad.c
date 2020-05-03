/* 
 * File:   newmain.c
 * Author: user
 *
 * Created on March 22, 2020, 11:33 AM
 */

/* this program use PIC16f877a that connect by PORTD and PORTC  4 stepper-motors 28BYJ-48 */
/*we you need 5V/2A charger   to turn on the motors for turn on the motors  */


/*https://en.wikiversity.org/wiki/Aircraft_piloting/Attitude_flying*/



//#include <stdio.h>
#include <stdlib.h>


#include <xc.h>
#define _XTAL_FREQ 4000000
// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG



 const int delay = 1500; // using for motor step 
 
 //char c = 0 ;  // this will contorol the angle of the motor (64 for 45 degree)
 //char last_S_H='i',last_S_V = 'i' ; // this save the last sensor move the H:horizontal V:vertical motor
 //signed char last_Di_H = 1 ,last_Di_V=1; // this save the last Direction the H:horizontal V:vertical motor move 
// char motors = 'i'; 
 /* if i: initialize 
                         a: all motor
                    *    v: vertical  
                         h: horizentail 
                    */     



char last_angle_vertical = 0 , last_direction_vertical = 0 ;
char last_angle_horizontal = 0 , last_direction_horizontal = 0 ; 

//const unsigned char TABLE[] = {0x3f, 0x6, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x7, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
 

// a is the array  to move motor one step clockwise
const char a [] = {0xEE,0xCC,0xDD,0x99,0xBB,0x33,0x77,0x66};
// ac is the array  to move motor one step counter-clockwise
const char ac [] = {0x66,0x77,0x33,0xBB,0x99,0xDD,0xCC,0xEE};
const char array_size = 8 ; // the size of the a[] 
/*
 * 
 */


void step(signed char direction,char VH);
void move_shaft(signed char start_angle_vertical,signed char end_angle_vertical,signed char start_angle_horizontal,signed char end_angle_horizontal,signed char direction_vertical,signed char direction_horizontal);
char calcuate_step_per_angle(signed char start_angle,signed char end_angle);



int main(int argc, char** argv) {
   
    PORTB = 0x00;
    PORTC=  00;
    TRISD =  0x00 ; 
    TRISC = 0x00;
    TRISB = 0x0F ;
  /*  TRISA=0x00;
    //TRISE = 0x00 ; 
    ADCON0=0x00;
    ADCON1= 0x07;//*/
    
  //  char b = 0  ;
   char result ; 
    while(1){
       //  b = PORTC ^ 0x01;
        
        /*      
         7 8 9 
         4 5 6
         1 2 3
               RB7      
             
        RB4    RB5   RB6
         
               RB3        
         
         */
        //result = scan();
        //display(result);
        //result=map(result);
      /* // vertical
        if(result == 0x04) {}*/
        
        
       if( RB0 == 1){ 
         //  start(last_angle_horizontal,-45,1 , 'h');
          move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,45,
                      -1,1);//  button 3 , move up-right*/ 
          // start(last_angle_vertical,-45,1 , 'v');
          //step(-1,'v');
          __delay_ms(100);
        
        }
       else if( RB1 == 1 ){
           //start(last_angle_horizontal,45,-1 , 'h');
           move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,0,
                      -1,-last_direction_horizontal);// button 2 will move to up */
                          
           //start(last_angle_vertical,45,-1 , 'v');
          //step(-1,'v');
          __delay_ms(100);
        
        }
       else if( RB2 == 1 ){
           //start(last_angle_horizontal,45,-1 , 'h');
           move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,-45,
                      -1,-1);//  1
                          
           //start(last_angle_vertical,45,-1 , 'v');
          //step(-1,'v');
          __delay_ms(100);
        
        }
       else if( RB3 == 1 ){
           //start(last_angle,0,-last_direction , 'a');
             //move_shaft_to_zero(last_angle_vertical,last_angle_horizontal,-last_direction_vertical,-last_direction_horizontal);
            move_shaft(last_angle_vertical,0,
                        last_angle_horizontal,0,
                       -last_direction_vertical,-last_direction_horizontal);// button 5 set all to zero angle */
             
             //start(last_angle_horizontal,0,-last_direction_horizontal , 'h');
            // start(last_angle_vertical,0,-last_direction_vertical , 'v');
          //step(-1,'v');
          __delay_ms(100);
        }
    
        
      
    }
        
    return (EXIT_SUCCESS);
}

/*this function take three parameters : 
 start_angle : to control the angle of the motor to make the motor go it should be (between 0 - steps)
 direction : define the direction of the motor
 current : define which current sensor call this function and it will be save in last_S_H
 *  *  */

char calcuate_step_per_angle(signed char start_angle,signed char end_angle){

    char steps, angle =(char)abs(end_angle - start_angle);
    
    //if step = 1 the motor will rotate (360/512) degree = 0.703 degree 
    if (angle == 45 ){
       steps = 64 ;   // this will move the motor/s to (+/-) 45 degree the direction decide where to move 
    }
    else if(angle == 90){
        steps= 128 ;  // this will move the motor/s to (+/-) 90 degree the direction decide where to move 
    }
    else{
        steps = 0 ; // if step = 0 then the motor will not move to any direction 
    }

  return steps ;
}




/*this function  will control 4 motors connected to PORTD and PORTC 
 we assume that to motor will be connected to which PORT _one PORT to
 * control vertical movement and the other for horizontal_
 
 * the Parameter meaning : 
 1-  start_angle_vertical: the current angle of the two motors in PORTD
 2-  end_angle_vertical: the requiring  angle of the two motors in PORTD
 3-  start_angle_horizontal: the current angle of the two motors in PORTC
 4-  end_angle_horizontal: the requiring  angle of the two motors in PORTC
 5-  direction_vertical
 6-  direction_horizontal
 */
void move_shaft(signed char start_angle_vertical,signed char end_angle_vertical,
                signed char start_angle_horizontal,signed char end_angle_horizontal,
                signed char direction_vertical,signed char direction_horizontal)
{

    char steps_v,steps_h;
    steps_v=  calcuate_step_per_angle(start_angle_vertical,end_angle_vertical);
    steps_h = calcuate_step_per_angle(start_angle_horizontal,end_angle_horizontal);
    
    char i =  0,j= 0 ; 
    if(steps_v == steps_h){
     while(i<steps_v){
             step (direction_vertical,'v');
             step (direction_horizontal,'h');
        
        i++;
    }
   }
    else{
        while(i<steps_v){
             step (direction_vertical,'v');
           //  step (direction_horizontal,'h');
             if(j<steps_h){
                step (direction_horizontal,'h'); 
                j++;
             }
        
        i++;
       }
        while(j<steps_h){
         //    step (direction_vertical,'v');
             step (direction_horizontal,'h');
             if(i<steps_v){
                step (direction_vertical,'v'); 
                i++;
             }
        
        j++;
       }
   } 
    
    
    
    last_angle_vertical = end_angle_vertical ;  // save  where  the last angle set/point for vertical motors
    last_direction_vertical = direction_vertical ; //save the direction we move 
    last_angle_horizontal = end_angle_horizontal ;  // save  where  the last angle set/point for horizontal motors
    last_direction_horizontal = direction_horizontal ; //save the direction we move 
    
    
}



/*this function make the motor to rotate 
 if direction positive rotate counter clockwise
 if direction negative rotate clockwise
 VH:                                                                                     
     v for vertical motor
     h for horizontal motor */
void step (signed char direction,char VH){
    int i ;
    /*id direction >=0 rotate clockwise*/
    if(direction >= 0 ){
        for (i=0;i<(int)array_size;i++){
            if( VH=='v'){
               PORTD =  (ac[i]<< 4) | (a[i]>>4);
               __delay_us(delay);
            }
            else if( VH=='h'){
               PORTC =  (ac[i]<< 4) | (a[i]>>4);
               __delay_us(delay);
            }
           
        }
    }
    
    /*if direction negative rotate counter clockwise */
    else{
      for (i=(int)array_size-1;i>=0;i--){
            if( VH=='v'){
               PORTD = (ac[i]<< 4) | (a[i]>>4);
               __delay_us(delay);
            }
            else if( VH=='h'){
               PORTC = (ac[i]<< 4) | (a[i]>>4);
               __delay_us(delay);
            }
        }
    }
}



          /* move_shaft(last_angle_vertical,45,
                      last_angle_horizontal,0,
                      1,-last_direction_horizontal);// button 8 will move to down*/

          /* move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,0,
                      -1,-last_direction_horizontal);// button 2 will move to up */

     
            /* move_shaft(last_angle_vertical,0,
                        last_angle_horizontal,0,
                       -last_direction_vertical,-last_direction_horizontal);// button 5 set all to zero angle */
       
           
           /* move_shaft(last_angle_vertical,0,
                      last_angle_horizontal,45,
                      -last_direction_vertical,1);//  button 6 , move right*/
       
         /*   move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,45,
                      -1,1);//  button 1 , move up-right*/                      
       
          
          /*   move_shaft(last_angle_vertical,0,
                      last_angle_horizontal,-45,
                      -last_direction_vertical,1);//  button 4 , move right*/
         
         /*   move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,-45,
                      -1,-1);//  button 1 , move up-left*/
                          
         /*   move_shaft(last_angle_vertical,-45,
                      last_angle_horizontal,45,
                      -1,1);//  button 3 , move up-right*/  
                      
         /*   move_shaft(last_angle_vertical,45,
                      last_angle_horizontal,45,
                      1,1);//  button 9 , move down-left*/
                          
         /*   move_shaft(last_angle_vertical,45,
                      last_angle_horizontal,-45,
                      1,-1);//  button 7 , move down-right*/                         
                                                    
         
        
