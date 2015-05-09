//
// Photoresistor Traffic Light
//
// December 2014
//
// Meng (Julius) Lee
//
// Gabriel Ojeda
//
// Description:
//
// The purpose of this project was to simulate cars going through a two way intersection 
// using photoresistors, LEDs, and an ATMEGA-1284 microcontroller. The intersection 
// consists of two traffic lights, two car paths, and an indicator LED to show that the 
// car passed through the intersection. Each traffic light has one red LED, one yellow 
// LED, and one green LED.  Each car path consists of eight LEDs to show the car 
// approaching the traffic light. One traffic light has eight blue LEDs and the other has 
// eight white LEDs. The indicator light is a single green LED. 
//

#include "timer.h"
#include <stdio.h>

void ADC7_init();                       //Function to get the ADC value from potentiometer
uint16_t read_adc(uint8_t channel);     //Variable to determine which ADC channel is receiving an input

//--------Find GCD function --------------------------------------------------
unsigned long int findGCD(unsigned long int a, unsigned long int b) {
	unsigned long int c;
	while(1) {
		c = a % b;
		if(c == 0){
			return b;
		}
		a = b;
		b = c;
	}
	return 0;
} 
//--------End find GCD function ----------------------------------------------

//--------Task scheduler data structure---------------------------------------
typedef struct _task {
	signed char state;                    //Task's current state
	unsigned long int period;             //Task period
	unsigned long int elapsedTime;        //Time elapsed since last task tick
	int (*TickFct)(int);                  //Task tick function
} task;
//--------End Task scheduler data structure-----------------------------------

//--------Global Variables----------------------------------------------------
unsigned char car_waiting = 0;          //Global variable for SM1 to signal SM2 if car led moves to the last place
unsigned char car_waiting1 = 0;         //Global variable for SM3 to signal SM4 if car led moves to the last place
unsigned char good_to_go = 0;           //Global variable for SM2 signaling SM1 that the car is good to go
unsigned char gtg = 0;                  //Global variable for SM4 signaling SM3 that the car is good to go
unsigned char cnt = 0;                  //Global counter for first state machine
unsigned char cnt_SM2 = 0;              //Global counter for second state machine
unsigned char cnt_SM3 = 0;              //Global counter for third state machine
unsigned char cnt_SM4 = 0;              //Global counter for fourth state machine
uint16_t adc0_value = 0;
uint16_t adc6_value = 0;
//--------End Global Variables------------------------------------------------

//--------User defined FSMs---------------------------------------------------
//Enumeration of states.
enum SM1_States { SM1_init, SM1_wait, SM1_sensed_car, SM1_moving_car, SM1_moving_car_8, SM1_waited};    //State machine to control moving car in lane 1
enum SM2_States { SM2_init, SM2_wait, SM2_green, SM2_stillgreen, SM2_greener, SM2_yellow};              //State machine to control traffic light in lane 1
enum SM3_States { SM3_init, SM3_wait, SM3_sensed_car, SM3_moving_car, SM3_moving_car_8, SM3_waited };   //State machine to control moving car in lane 2 
enum SM4_States { SM4_init, SM4_wait, SM4_green, SM4_stillgreen, SM4_greener, SM4_yellow };             //State machine to control traffic light in lane 2

int SMTick1(int state) {

	ADC7_init();
	adc0_value = read_adc(0);
	
//--------Local Variables----------------------------------------------------
	unsigned short s = adc0_value;                      //ADC value read from potentiometer
	static const unsigned short max_val = 0x0067;       //Max value read from potentiometer
	static const unsigned short min_val = 0x0030;       //Min value read from potentiometer
	unsigned short almost = min_val + 0x05;             //Almost value to ensure reading from potentiometer
//--------End Local Variables------------------------------------------------
	
	switch (state) { //Transitions
		case SM1_init:
			state = SM1_wait;                               //Transition from SM1_init to SM1_wait state
		break;
		case SM1_wait:                                    //Wait state
			if(cnt > 10)                                    //Prolong the "LED off" so that the green light when car passes is on longer
			{
				PORTD = (PORTD &0xBF);                        //PORTD controlling green light when car passes is off
			}
			cnt++;
			if (s < almost)                                 //Almost is calibrated so that if adc value - s - gets low enough when we block the light, condition is true
			{
				cnt = 0;                                      //Reset cnt to 0 because it was used in wait
				state = SM1_sensed_car;                       //State goes to sensed car if s<almost
			}
			else if (s >= almost)                           //If s is greater than or the value almost, state stays in wait state
			{
				state = SM1_wait;
			} 
			break;
		case SM1_sensed_car:                              //State goes directly into moving car
			state = SM1_moving_car;
		break;
		case SM1_moving_car:                              //Detects if the car is in last position
			if(PORTB == 0x80)                               //If PORTB = 0x80, state goes to moving_car_8
			{
				cnt = 0;
				state = SM1_moving_car_8;
			}
			else
			{
				state = SM1_moving_car;	
			}
			break;
		case SM1_moving_car_8:                            //Car waits in front of the stop light
			if(cnt < 10)
			{
				state = SM1_moving_car_8;
			}
			else                                            //Done waiting state goes to SM1_waited
			{
				state = SM1_waited;
			}
			cnt++;
		break;
		case SM1_waited:
			if (good_to_go == 1)                            //Wait for light SM2 to signal good to go = 1
			{
				PORTB = 0;                                    //Clear the car led
				PORTD = (PORTD & 0xBF)|0x40;                  //Set green light when car passes to 1
				car_waiting = 0;                              //Clear car_waiting for the next car
				cnt = 0;                                      //Reset cnt
				state = SM1_wait;                             //Goes back to SM1_wait
			}
			else 
			{
				state = SM1_waited;                           //If good to go == 0 it stays in SM1_wait
			}
		break;
		default:
			state = SM1_init;                               //Default: Initial state
		break;
	} //Transitions

	switch(state) { //Actions
        case SM1_wait:
        break;
        case SM1_sensed_car:
            PORTB = 0x01;                             //The position of the first car
        break;
        case SM1_moving_car:
            PORTB = PORTB << 1;                       //Shift the position of the car by 1
            break;
        case SM1_moving_car_8:
        break;
        case SM1_waited:
            car_waiting = 1;                          //Signal SM2 that the car is done waiting
        break;
        default:
            state = SM1_wait;                         //Default: Initial state
        break;
	} //Actions

	return state;
} //SMTick1

int SMTick2(int state) {

	switch (state) { //Transitions
		case SM2_init:
			PORTD = 0x24;                                   //Turn on red light for both lanes.
			state = SM2_wait;
		break;
		case SM2_wait:                                    //Waiting for SM1 to signal that car is done waiting
			if(car_waiting == 0)
			{
				state = SM2_wait;
			}
			else if ((car_waiting == 1) && cnt_SM2 < 2)     //Delay for car waiting
			{
				cnt_SM2++;
				state = SM2_wait;
			}
			else if((car_waiting == 1) && cnt_SM2 >= 2)     //State goes to green after the short count down
			{
				state = SM2_green;
			}
		break;
		case SM2_green:
			state = SM2_stillgreen;                         //State goes right after to state still green
		break;
		case SM2_stillgreen:
			PORTD = PORTD & 0xBF;                           //Reset the light
			cnt_SM2 = 0;
			state = SM2_greener;                            //State goes right after state greener
		break;
		case SM2_greener:
			if (cnt_SM2 < 4)                                //Wait for 4 cycles
			{
				state = SM2_greener;
			}
			else if (cnt_SM2 >= 4)                          //Waiting for G1 countdown to be done
			{
				PORTD = (PORTD & 0xF8)| 0x02;                 //Turn off G1 turn on Y1
				cnt_SM2 = 0;                                  //Reset cnt_SM2
				state = SM2_yellow;							              //State goes to yellow
			}
			cnt_SM2++;
		break;
		case SM2_yellow:									                //Count down for the yellow light
			if (cnt_SM2 < 4)
			{
				state = SM2_yellow;
			}
			else if (cnt_SM2 >= 4)
			{
				PORTD = (PORTD & 0xF8)| 0x04;                 //Turn off Y1 turn on R1
				cnt_SM2 = 0;				
				good_to_go = 0;                               //Signal SM1 that car cannot pass anymore
				state = SM2_wait;
			}
			cnt_SM2++;
		break;
		default:
			state = SM2_init;		
		break;
	} //Transitions

	switch(state) { //Actions
		case SM2_init:	
		break;
		case SM2_wait:
		break;
		case SM2_green:
			PORTD = (PORTD & 0xF8)| 0x01;					          //Turn on G1
		break;
		case SM2_stillgreen:
			good_to_go = 1;									                //Signal SM1 that a car is good to go
		break;
		case SM2_greener:
		break;
		case SM2_yellow:
		break;
		default:		
		break;
	} //Actions

	return state;
	
} //SMTick2

int SMTick3(int state) {
	
	ADC7_init();											                  //Initialize the adc
	adc6_value = read_adc(6);								            //Read in adc value from PA6
	
//--------Local Variables----------------------------------------------------	
	unsigned short s = adc6_value;
	static const unsigned short max_val = 0x0067;										
	static const unsigned short min_val = 0x0030;											
	unsigned short almost = min_val + 0x05;
//--------End Local Variables------------------------------------------------
	
	switch (state) { //Transitions
		case SM3_init:
		state = SM3_wait;
		break;
		case SM3_wait:
			if(cnt_SM3 > 10)								                //Delay to turn off C2 - light when car passes
			{
				PORTD = (PORTD & 0x7F);                       //Change C2 to 0
			}
			cnt_SM3++;
			if (s < almost)									                //When s=adcvalue<almost, the state goes to sensed car
			{
				cnt_SM3 = 0;
				state = SM3_sensed_car;
			}
			else if (s >= almost)							              //If condition is false, it stays in wait state
			{
				state = SM3_wait;
			}
		break;
		case SM3_sensed_car:								              //Car sensed, state goes into moving car
			cnt_SM3 = 0;	
			state = SM3_moving_car;
		break;
		case SM3_moving_car:
			if(PORTC == 0x80)								                //When car reaches the last spot, PORTC = 0x80
			{
				cnt_SM3 = 0;
				state = SM3_moving_car_8;
			}
			else
			{
				state = SM3_moving_car;
			}
		break;
		case SM3_moving_car_8:								            //Delay for car to wait in front of the stop light
			if(cnt_SM3 < 10)
			{
				state = SM3_moving_car_8;
			}
			else											                      //Countdown for car to wait is done
			{
				state = SM3_waited;
			}
			cnt_SM3++;
		break;
		case SM3_waited:									                //Waiting for signal from SM4 good to go (gtg) turns on
			if (gtg == 1)
			{
				PORTC = 0;									                  //Clear the car led
				PORTD = (PORTD & 0x7F)|0x80;			            //Turn on C2
				car_waiting1 = 0;							                //Reset car waiting1 to 0 for the next passing car
				cnt_SM3 = 0;
				state = SM3_wait;							                //State goes to wait
			}
			else
			{
				state = SM3_waited;							              //If gtg == 0, stays waiting
			}
		break;
		default:
			state = SM3_init; 								              //Default: Initial state
		break;
	} //Transitions

	switch(state) { //Actions
		case SM3_wait:
		break;
		case SM3_sensed_car:
			PORTC = 0x01;									                 //First position for the car led to be lit
		break;
		case SM3_moving_car:
			PORTC= PORTC << 1;								             //Shift car position left by 1 until it is PORTC = 0x80
		break;
		case SM3_moving_car_8:
		break;
		case SM3_waited:
		car_waiting1 = 1;									               //Signal SM4 that the car is done waiting.
		break;
		default:
			state = SM3_wait; 								             //Default: Initial state
		break;
	} //Actions

	return state;
}

int SMTick4(int state) {
	
	switch (state) { //Transitions
		case SM4_init:
		state = SM4_wait;
		break;
		case SM4_wait:										               //If the carwaiting is not on it stays to wait
			if(car_waiting1 == 0)
			{
				state = SM4_wait;
			}
			else if ((car_waiting1 == 1) && cnt_SM4 < 2)	 //After the car waiting is 1, delays for 2 cycles
			{
				cnt_SM4++;
				state = SM4_wait;
			}
			else if((car_waiting1 == 1) && cnt_SM4 >= 2)	 //After the countdown the state goes to green
			{
				state = SM4_green;
			}
			break;
        case SM4_green:                              //State goes straight into stillgreen after action
				state = SM4_stillgreen;
			break;
        case SM4_stillgreen:                         //Reset light 2
				PORTD = PORTD & 0x7F;
				cnt_SM4 = 0;
				state = SM4_greener;									
			break;
		case SM4_greener:
			if (cnt_SM4 < 4)
			{
				state = SM4_greener;
			}
			else if (cnt_SM4 >= 4)							           //Turn off G2 turn on Y2
			{
				PORTD = (PORTD & 0xC7)| 0x10; 
				cnt_SM4 = 0;
				state = SM4_yellow;
			}
			cnt_SM4++;
		break;
		case SM4_yellow:                                 //Yellow light is turned off, and turn on red light.
			if (cnt_SM4 < 4)
			{
				state = SM4_yellow;
			}
			else if (cnt_SM4 >= 4)
			{
				PORTD = (PORTD & 0xC7)| 0x20;                //Red light on
				cnt_SM4 = 0;                                 //Reset cnt
				gtg = 0;                                     //Signal SM3 that no car should pass anymore
				state = SM4_wait;                            //State goes back to wait
			}
			cnt_SM4++;
		break;
		default:
			state = SM4_init;
		break;
	} //Transitions

	switch(state) { //Actions
		case SM4_init:		
		break;
		case SM4_wait:
		break;
		case SM4_green:
			PORTD = (PORTD & 0xC7)| 0x08;					         //Turn on G1
		break;
		case SM4_stillgreen:
			gtg = 1;										                   //Signal SM3 that the car may pass
		break;
		case SM4_greener:
		break;
		case SM4_yellow:
		break;
		default:
		break;
	} //Actions

	return state;
} //SMTick4
// --------END User defined FSMs-----------------------------------------------

int main(){

	DDRA = 0x00;		//PORTA set to input
	PORTA = 0xFF;		//PORTA outputs 0xFF
	DDRB = 0xFF;		//PORTB set to output
	PORTB = 0x00;		//PORTB outputs 0x00
	DDRC = 0xFF;		//PORTC set to output
	PORTC = 0x00;		//PORTC outputs 0x00
	DDRD = 0xFF;		//PORTD set to output
	PORTD = 0x00;		//PORTD outputs 0x00

	// Period for the tasks
	unsigned long int SMTick1_calc = 50;
	unsigned long int SMTick2_calc = 500;
	unsigned long int SMTick3_calc = 50;
	unsigned long int SMTick4_calc = 500;

	//Calculating GCD
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(SMTick1_calc, SMTick2_calc);
	tmpGCD = findGCD(tmpGCD, SMTick3_calc);
	tmpGCD = findGCD(tmpGCD, SMTick4_calc);

	//Greatest common divisor for all tasks or smallest time unit for tasks.
	unsigned long int GCD = tmpGCD;

	//Recalculate GCD periods for scheduler
	unsigned long int SMTick1_period = SMTick1_calc/GCD;
	unsigned long int SMTick2_period = SMTick2_calc/GCD;
	unsigned long int SMTick3_period = SMTick3_calc/GCD;
	unsigned long int SMTick4_period = SMTick4_calc/GCD;

	//Declare an array of tasks 
	static task task1, task2, task3, task4;
	task *tasks[] = { &task1, &task2, &task3, &task4 };
	const unsigned short numTasks = sizeof(tasks)/sizeof(task*);

	// Task 1
	task1.state = SM1_init;									          //Task initial state.
	task1.period = SMTick1_period;							      //Task Period.
	task1.elapsedTime = SMTick1_period;						    //Task current elapsed time.
	task1.TickFct = &SMTick1;								          //Function pointer for the tick.

	// Task 2
	task2.state = SM2_init;									          //Task initial state.
	task2.period = SMTick2_period;							      //Task Period.
	task2.elapsedTime = SMTick2_period;						    //Task current elapsed time.
	task2.TickFct = &SMTick2;								          //Function pointer for the tick.

	// Task 3
	task3.state = SM3_init;									          //Task initial state.
	task3.period = SMTick3_period;							      //Task Period.
	task3.elapsedTime = SMTick3_period; 					    //Task current elasped time.
	task3.TickFct = &SMTick3; 								        //Function pointer for the tick.

	// Task 4
	task4.state = SM4_init;									          //Task initial state.
	task4.period = SMTick4_period;							      //Task Period.
	task4.elapsedTime = SMTick4_period; 					    //Task current elasped time.
	task4.TickFct = &SMTick4; 								        //Function pointer for the tick.

	// Set the timer and turn it on
	TimerSet(GCD);
	TimerOn();
	
	unsigned short i; 										            //Scheduler for-loop iterator
	
	while(1) {
	
		// Scheduler code
		for(i = 0; i < numTasks; i++) {
			// Task is ready to tick
			if(tasks[i]->elapsedTime == tasks[i]->period) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
	}
	
	while(!TimerFlag);
	TimerFlag = 0;
	}

	return 0;
}

void ADC7_init()
{	
	ADCSRA |= ((1<<ADPS2)|1<<ADPS0);						      //ADC reference clock
	ADMUX |= 1 << REFS0;									            //Voltage reference
	ADCSRA |= 1 << ADEN;									            //Turn on ADC
	ADCSRA |= 1<< ADSC;										            //Initial conversion
}

uint16_t read_adc(uint8_t channel)
{
	ADMUX &= 0xF0;											              //Clear the older channel that was read
	ADMUX |= channel;										              //Defines the new ADC channel to be read
	ADCSRA |= 1<<ADSC;										            //Starts a new conversion
	while(ADCSRA & (1<<ADSC));								        //Wait until the conversion is done
	return ADCW;											                //Returns the ADC value of the chosen channel
}
