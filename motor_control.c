/*
 *APSC 1299: Introduction to Microcontrollers
 *Final Robot Project
 *April 6th, 2020
 * 
 * This program is designed to work with the PIC18F4525 microcontroller and 
 * will guide a line following robot through a variety of different track 
 * challenges including gaps, turns, dead ends, and crossroads.
 * 
 * Bobsy Narayan, Chris Wen, Tongyu Li
 */

#include "sumovore.h"
#include "motor_control.h"
#include <xc.h>

void motor_control(void) {
    int position = 0b00100u;
    int crossover_pass = 0;
    int end = 0;
    int nearby = 0;
    //very simple motor control
    switch (SeeLine.B) {
        case 0b10000u:
        case 0b01000u:
        case 0b00100u:
        case 0b00010u:
        case 0b00001u:
            //no breaks all above readings end up here
            //tight curves
            follow_simple_curves();
            break;

        case 0b01100u:
        case 0b11100u:
        case 0b11110u:
            spin_left();
            break;

        case 0b00110u:
        case 0b00111u: //90 degree angle turns
        case 0b01111u:
            spin_right();
            break;

        case 0b01110u:
            straight_fwd();
            break;

        case 0b00000u: //gap or dead end
            OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_16BIT & T0_PS_1_256);
            TMR0IF = 0;
            WriteTimer0(40000); //for gaps
            while (TMR0IF != 1) {
                check_sensors();
                set_leds();
                if (SeeLine.b.Left == 1 || SeeLine.b.CntLeft == 1 || SeeLine.b.Center == 1 || SeeLine.b.CntRight == 1 || SeeLine.b.Right == 1)
                    return;
            }
            while (SeeLine.B == 0b00000u) { //dead end
                reverse();
                check_sensors();
                set_leds();
            }
            while (SeeLine.B != 0b10000u) { //continue spinning until in the correct position
                check_sensors();
                set_leds;
                spin_left();
            }
        case 0b11111u://finish line & right angled crossover
            OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_16BIT & T0_PS_1_256);
            WriteTimer0(45000);
            TMR0IF = 0;
            while (TMR0IF != 1) {
                straight_fwd();
                check_sensors();
                set_leds();
                if (SeeLine.B != 0b11111u) {
                    return; //crossover, keep going back to follow_simple_curve
                }
                    
            }
            while (1) no_move(); //hit final end and stop

        case 0b01001u:
        case 0b10010u:
        case 0b01010u:
        case 0b10100u:
        case 0b00101u: // crossover + small angle turns + nearby trails
            straight_fwd();
            while (((SeeLine.B & 0b00100u) >> 2) == 1) {
                position = SeeLine.B - 0b00100u; //update position for use of small angle turn
                check_sensors();
                set_leds();
                while (SeeLine.B != 0b00100u) { //nearby trail+small angle turn
                    check_sensors();
                    set_leds();
                    straight_fwd();

                    if (SeeLine.B == 0b00100u) { //small angle turn
                        if ((position < 0b01000u) && (position >= 0b00010u)) { //signal was on its right side, small angle turn
                            spin_right();
                            for (int i = 0; i < 500; i++) {
                                _delay(4800ul); //spin 90 degree
                            }
                            position = 0b00100u;
                            return;
                        }

                        else if (position >= 0b01000) { //signal was on its left side
                            spin_left();
                            for (int j = 0; j < 500; j++) {
                                _delay(4800ul); //spin 90 degree
                            }
                            position = 0b00100u;
                            return;
                        }

                    }
                    nearby = 1; //reach this line if it's a nearby trail
                }

                if (nearby = 1) { //end of the nearby trail
                    nearby = 0;
                    return;
                }

                while ((SeeLine.B == 0b00100u) && (crossover_pass != 1)) { //crossover, keep straight
                    check_sensors();
                    set_leds();
                }

                crossover_pass = 1;
                if (SeeLine.B == 0b00100) {
                    crossover_pass = 0; //end of the cross over
                    return;
                }
            }
            break;
    }
}

void follow_simple_curves(void) {
    if (SeeLine.b.Center) straight_fwd();
    else if (SeeLine.b.Left) spin_left();
    else if (SeeLine.b.CntLeft) turn_left();
    else if (SeeLine.b.CntRight) turn_right();
    else if (SeeLine.b.Right) spin_right();
}

void spin_left(void) {
    set_motor_speed(left, rev_fast, 0);
    set_motor_speed(right, fast, 0);
}

void turn_left(void) {
    set_motor_speed(left, stop, 0);
    set_motor_speed(right, fast, 0);
}

void straight_fwd(void) {
    set_motor_speed(left, fast, 0);
    set_motor_speed(right, slow, 86);
}

void spin_right(void) {
    set_motor_speed(left, fast, 0);
    set_motor_speed(right, rev_fast, 0);
}

void turn_right(void) {
    set_motor_speed(left, fast, 0);
    set_motor_speed(right, stop, 0);
}

void reverse(void) {
    set_motor_speed(left, rev_fast, 0);
    set_motor_speed(right, rev_medium, 60000);
}

void no_move(void) {
    set_motor_speed(left, stop, 0);
    set_motor_speed(right, stop, 0);
}