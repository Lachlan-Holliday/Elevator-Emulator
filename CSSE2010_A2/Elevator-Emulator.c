/*
 * Elevator-Emulator.c
 *
 * Main file
 *
 * Authors: Peter Sutton, Ahmed Baig
 * Modified by Lachlan Holliday
 */ 

/* Definitions */

#define F_CPU 8000000L
#define BUZZER_DDR DDRD
#define BUZZER_PORT PORTD
#define BUZZER_PIN PC7

#define SEG_A (1<<PA0)
#define SEG_B (1<<PA1)
#define SEG_C (1<<PA2)
#define SEG_D (1<<PA3)
#define SEG_E (1<<PA4)
#define SEG_F (1<<PA5)
#define SEG_G (1<<PA6)
#define SEG_MASK (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G)

#define SSD_CC (1<<PD2)
#define SSD_DP (1<<PD3)

#define LED_L0 (1<<PC4)
#define LED_L1 (1<<PC5)
#define LED_L2 (1<<PC6)
#define LED_L3 (1<<PC7)
#define LED_MASK (LED_L0|LED_L1|LED_L2|LED_L3)



/* External Library Includes */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h>

/* Internal Library Includes */

#include "display.h"
#include "ledmatrix.h"
#include "buttons.h"
#include "serialio.h"
#include "terminalio.h"
#include "timer0.h"

/* Data Structures */

typedef enum {UNDEF_FLOOR = -1, FLOOR_0=0, FLOOR_1=4, FLOOR_2=8, FLOOR_3=12} ElevatorFloor;

uint8_t floor_seg[4] = {
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,    // “0”
	SEG_B|SEG_C,                            // “1”
	SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,          // “2”
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_G           // “3”
};

/* Global Variables */
uint32_t time_since_move;
ElevatorFloor current_position;
ElevatorFloor destination;
ElevatorFloor current_floor;
ElevatorFloor traveller_dest;
ElevatorFloor last_traveller_floor = UNDEF_FLOOR;
const char *direction;
bool moved = false;
bool traveller_present = false;
bool traveller_onboard = false;
ElevatorFloor traveller_floor;
uint16_t speed;
uint8_t last_direction = SEG_G;
uint32_t floors_with_traveller    = 0;
uint32_t floors_without_traveller = 0;



#define TRAVELLER_COLUMN 4

bool led_animating = false;
uint32_t led_anim_start = 0;

void start_led_animation(void) {
	led_animating = true;
	led_anim_start = get_current_time();
}

void service_led_animation(void) {
	if (!led_animating) return;

	uint32_t dt = get_current_time() - led_anim_start;
	
	PORTC &= ~LED_MASK;


	if (dt < 400) { //door closed
        PORTC |= (LED_L1|LED_L2);
	}
	else if (dt < 800) { // door open
        PORTC |= (LED_L0|LED_L3);
	}
	else if (dt < 1200) { //door closing
        PORTC |= (LED_L1|LED_L2);
	}
	else {
		led_animating = false; //door close
        PORTC |= (LED_L1|LED_L2);
	}
}



/* Internal Function Declarations */

void initialise_hardware(void);
void start_screen(void);
void start_elevator_emulator(void);
void handle_inputs(void);
void draw_elevator(void);
void draw_floors(void);
void draw_traveller(void);
uint16_t get_speed(void);


/* Main */

int main(void) {
	// Setup hardware and call backs. This will turn on 
	// interrupts.
	initialise_hardware();
	
	// Show the splash screen message. Returns when display is complete
	start_screen();
	
	// Start elevator controller software
	start_elevator_emulator();
}

/* Internal Function Definitions */


static void beep(uint16_t freq, uint16_t dur_ms)
{
	OCR1A   = F_CPU/(16UL*freq) - 1;
	TCNT1   = 0;
	TCCR1A  = 0;
	TCCR1B  = (1<<WGM12) | (1<<CS11);

	uint32_t count = (uint32_t)freq * dur_ms * 2UL / 1000UL;
	while (count--) {
		while (!(TIFR1 & (1<<OCF1A)));
		TIFR1        = (1<<OCF1A);
		BUZZER_PORT ^= (1<<BUZZER_PIN);
	}
	TCCR1B = 0;
	BUZZER_PORT &= ~(1<<BUZZER_PIN);
}


/**
 * @brief All hardware initialisation occurs here
 * @arg none
 * @retval none
*/
void initialise_hardware(void) {
	
	ledmatrix_setup();
	init_button_interrupts();
	// Setup serial port for 19200 baud communication with no echo
	// of incoming characters
	init_serial_stdio(19200,0);
	
	init_timer0();
	
	// Turn on global interrupts
	sei();
	
	/* segments A–G on PORTA */
	DDRA |= SEG_MASK;
	/* turn all segments off to start */
	PORTA &= ~SEG_MASK;

	/* CC and DP on PORTC */
	DDRD |= SSD_CC | SSD_DP;
	PORTD |=  SSD_CC;
	PORTD &= ~SSD_DP;
	BUZZER_DDR |= (1<<BUZZER_PIN);
	
	DDRC |= LED_MASK;
	PORTC &= ~LED_MASK;

}

/**
 * @brief Displays the "EC" start screen with elevator symbol
 * @arg none
 * @retval none
*/
void start_screen(void) {
	// Clear terminal screen and output a message
	clear_terminal();
	move_terminal_cursor(10,10);
	printf_P(PSTR("Elevator Controller"));
	move_terminal_cursor(10,12);
	printf_P(PSTR("CSSE2010 project by Lachlan Holliday"));
	move_terminal_cursor(10,14);
	printf_P(PSTR("Student Number: 48840468"));
	
	// Show start screen
	start_display();
	
	// Animation variables
	uint32_t doors_frame_time = 0;
	uint32_t interval_delay = 150;
	uint8_t frame = 0;
	uint8_t doors_opening_closing = 1; // 1 => opening, 0 => closing
	
	// Wait until a button is pressed, or 's' is pressed on the terminal
	while(1) {

		
		// Don't worry about this if/else tree. Its purely for animating
		// the elevator doors on the start screen
		if (get_current_time() - doors_frame_time  > interval_delay) {
			start_display_animation(frame);
			doors_frame_time   = get_current_time(); // Reset delay until next movement update
			if (doors_opening_closing) {
				interval_delay = 150;
				frame++;
				if (frame == 1) interval_delay = 2000;
				if (frame == 3) doors_opening_closing = 0;
			} else {
				interval_delay = 150;
				frame--;
				if (frame == 2) interval_delay = 500;
				if (frame == 0) doors_opening_closing = 1;
			}
		}
	
		// First check for if a 's' is pressed
		// There are two steps to this
		// 1) collect any serial input (if available)
		// 2) check if the input is equal to the character 's'
		char serial_input = -1;
		if (serial_input_available()) {
			serial_input = fgetc(stdin);
		}
		// If the serial input is 's', then exit the start screen
		if (serial_input == 's' || serial_input == 'S') {
			break;
		}
		// Next check for any button presses
		int8_t btn = button_pushed();
		if (btn != NO_BUTTON_PUSHED) {
			break;
		}
	}
}

static void multiplex_ssd(void) {
	static uint32_t last_t = 0;
	static bool show_floor = false;
	uint32_t now = get_current_time();
	if (now == last_t) return;
	last_t = now;
	show_floor = !show_floor;

	if (show_floor) {
		PORTD |= SSD_CC;
		} else {
		PORTD &= ~SSD_CC;
	}

	PORTA &= ~SEG_MASK;
	PORTD &= ~SSD_DP;

	if (show_floor) {
		uint8_t f = current_floor / 4;
		PORTA |= floor_seg[f];
		if (current_position % 4 != 0) {
			PORTD |= SSD_DP;
		}
		//right
		PORTD &= ~SSD_CC;
		} else {
		PORTA |= last_direction;  
		//left
		PORTD |= SSD_CC;
	}
}



/**
 * @brief Initialises LED matrix and then starts infinite loop handling elevator
 * @arg none
 * @retval none
*/
void start_elevator_emulator(void) {
	
	// Clear the serial terminal
	clear_terminal();
	
	// Initialise Display
	initialise_display();
	
	// Clear a button push or serial input if any are waiting
	// (The cast to void means the return value is ignored.)
	(void)button_pushed();
	clear_serial_input_buffer();

	time_since_move = get_current_time();
	
	current_position = FLOOR_0;
	destination      = FLOOR_0;
	current_floor    = FLOOR_0;
	direction        = "Stationary";
	moved            = true;
	traveller_dest = UNDEF_FLOOR;
	last_traveller_floor = UNDEF_FLOOR;

	
	
	// Draw the floors and elevator
	draw_elevator();
	draw_floors();
	
	current_position = FLOOR_0;
	destination = FLOOR_0;
	
	while(true) {
        multiplex_ssd();
		service_led_animation();


		speed = get_speed(); 
		
		// Only update the elevator every 200 ms
		if (get_current_time() - time_since_move > speed) {	
			uint8_t next_seg = SEG_G;


			if (destination > current_position) { // Move up
				current_position++;
				moved     = true;
				direction = "Up";
				next_seg  = SEG_A;
				if (current_position % 4 == 0) {
					current_floor = current_position;
					if (traveller_onboard) {
						floors_with_traveller++;
						} else {
						floors_without_traveller++;
					}
				}
			}
			else if (destination < current_position) { // Move down
				current_position--;
				moved     = true;
				direction = "Down";
				next_seg  = SEG_D;
				if (current_position % 4 == 0) {
					current_floor = current_position;
					if (traveller_onboard) {
						floors_with_traveller++;
						} else {
						floors_without_traveller++;
					}
				}
			}
			
			if (traveller_present && current_position == traveller_floor) {
				traveller_present = false;
				traveller_onboard = true;  
				destination = traveller_dest;
				traveller_dest = UNDEF_FLOOR;
				draw_traveller();
				beep(500, 100);
				
				start_led_animation(); 

			}
			
			if (traveller_onboard && current_position == destination) {
				traveller_onboard = false;
				beep(500, 100);
				
				start_led_animation();

			}

			
			if (next_seg != last_direction) {
				last_direction = next_seg;
				// update left digit
				PORTA = (PORTA & ~SEG_MASK) | last_direction;
			}
			
			draw_elevator();
			
			time_since_move = get_current_time(); // Reset delay until next movement update
		}
		if (moved) {
			clear_terminal();
			uint8_t floor_num = current_floor / 4;

			move_terminal_cursor(10,10);
			printf("Current Level: %d", floor_num);

			move_terminal_cursor(10,12);
			printf("Direction: %s", direction);

			move_terminal_cursor(10,14);
			printf("Floors with traveller: %lu", floors_with_traveller);

			move_terminal_cursor(10,16);
			printf("Floors without traveller: %lu", floors_without_traveller);

			moved = false;
		}
	
		handle_inputs();
	}
}

/**
 * @brief Draws 4 lines of "FLOOR" coloured pixels
 * @arg none
 * @retval none
*/
void draw_floors(void) {
	for (uint8_t i = 0; i < WIDTH; i++) {
		update_square_colour(i, FLOOR_0, FLOOR);
		update_square_colour(i, FLOOR_1, FLOOR);
		update_square_colour(i, FLOOR_2, FLOOR);
		update_square_colour(i, FLOOR_3, FLOOR);
	}
}

void draw_traveller(void) {
	if (last_traveller_floor != UNDEF_FLOOR) {
		int prev_row = last_traveller_floor + 1;
		update_square_colour(TRAVELLER_COLUMN, prev_row, EMPTY_SQUARE);
	}
	if (traveller_present) {
		int row = traveller_floor + 1;
		uint8_t obj;
		switch (traveller_dest) {
			case FLOOR_0: obj = TRAVELLER_TO_0; break;
			case FLOOR_1: obj = TRAVELLER_TO_1; break;
			case FLOOR_2: obj = TRAVELLER_TO_2; break;
			case FLOOR_3: obj = TRAVELLER_TO_3; break;
			default:      obj = TRAVELLER_TO_0; break;
		}
		update_square_colour(TRAVELLER_COLUMN, row, obj);
		last_traveller_floor = traveller_floor;
		} else {
		last_traveller_floor = UNDEF_FLOOR;
	}
}


/**
 * @brief Draws the elevator at the current_position
 * @arg none
 * @retval none
*/
void draw_elevator(void) {
	
	// Store where it used to be with old_position
	static uint8_t old_position; // static variables maintain their value, every time the function is called
	
	int8_t y = 0; // Height position to draw elevator (i.e. y axis)
	
	// Clear where the elevator was
	if (old_position > current_position) { // Elevator going down - clear above
		y = old_position + 3;
		} else if (old_position < current_position) { // Elevator going up - clear below
		y = old_position + 1;
	}
	if (y % 4 != 0) { // Do not draw over the floor's LEDs
		update_square_colour(1, y, EMPTY_SQUARE);
		update_square_colour(2, y, EMPTY_SQUARE);
	}
	old_position = current_position;
	
	// Draw a 2x3 block representing the elevator
	for (uint8_t i = 1; i <= 3; i++) { // 3 is the height of the elevator sprite on the LED matrix
		y = current_position + i; // Adds current floor position to i=1->3 to draw elevator as 3-high block
		if (y % 4 != 0) { // Do not draw on the floor
			update_square_colour(1, y, ELEVATOR);
			update_square_colour(2, y, ELEVATOR); // Elevator is 2 LEDs wide so draw twice
		}
	}
}

/**
 * @brief Reads btn values and serial input and adds a traveller as appropriate
 * @arg none
 * @retval none
*/
void handle_inputs(void) {
	
	
	/* ******** START HERE ********
	
	 The following code handles moving the elevator using the buttons on the
	 IO Board. Add code to handle BUTTON2_PUSHED and BUTTON3_PUSHED
	 
	 Here is how the following code works:
	 1. Get btn presses (if any have occurred). Remember that this is
		all handled in the buttons.c/h library.
	 2. Use an if/else tree based on which of the buttons has been
		pressed.
	 3. Set the destination of the elevator to the FLOOR_X corresponding
		with the particular button that was pressed.
	
	*/
	
	// We need to check if any button has been pushed
	uint8_t btn = button_pushed();
	char serial_input = -1;
	if (serial_input_available()) {
		serial_input = fgetc(stdin);}
		
	if (traveller_present || (current_floor != destination)) {
		return;
	}
	
	
	uint8_t switch_bits = (PIND >> 5) & 0b11 ;
	ElevatorFloor dest = UNDEF_FLOOR;
	switch (switch_bits) {
		case 0: dest = FLOOR_0; break;
		case 1: dest = FLOOR_1; break;
		case 2: dest = FLOOR_2; break;
		case 3: dest = FLOOR_3; break;
	}
	
	if (btn == BUTTON0_PUSHED || serial_input == '0') {
		if (dest == FLOOR_0) return;            
		traveller_dest = dest;
		traveller_floor = FLOOR_0;
		traveller_present= true;
		destination = FLOOR_0;      
		draw_traveller();
		beep(3000, 50);
	}
	else if (btn == BUTTON1_PUSHED || serial_input == '1') {
		if (dest == FLOOR_1) return;
		traveller_dest = dest;
		traveller_floor = FLOOR_1;
		traveller_present= true;
		destination = FLOOR_1;
		draw_traveller();
		beep(3000, 50);
	}
	else if (btn == BUTTON2_PUSHED || serial_input == '2') {
		if (dest == FLOOR_2) return;
		traveller_dest = dest;
		traveller_floor = FLOOR_2;
		traveller_present = true;
		destination = FLOOR_2;
		draw_traveller();
		beep(3000, 50);
	}
	else if (btn == BUTTON3_PUSHED || serial_input == '3') {
		if (dest == FLOOR_3) return;
		traveller_dest = dest;
		traveller_floor = FLOOR_3;
		traveller_present = true;
		destination = FLOOR_3;
		draw_traveller();
		beep(3000, 50);
	}
}

uint16_t get_speed(void) {
	if (PIND & (1 << 4)) {
		return 250;
		} else {
		return 100;
	}
}