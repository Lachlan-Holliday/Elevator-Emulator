/*
 * Elevator-Emulator.c
 *
 * Main file
 *
 * Authors: Peter Sutton, Ahmed Baig
 * Modified by <YOUR NAME HERE>
 */ 

/* Definitions */

#define F_CPU 8000000L

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

/* Global Variables */
uint32_t time_since_move;
ElevatorFloor current_position;
ElevatorFloor destination;
ElevatorFloor current_floor;
const char *direction;
bool moved = false;

/* Internal Function Declarations */

void initialise_hardware(void);
void start_screen(void);
void start_elevator_emulator(void);
void handle_inputs(void);
void draw_elevator(void);
void draw_floors(void);

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
	
	
	// Draw the floors and elevator
	draw_elevator();
	draw_floors();
	
	current_position = FLOOR_0;
	destination = FLOOR_0;
	
	while(true) {
		
		// Only update the elevator every 200 ms
		if (get_current_time() - time_since_move > 200) {	
			
			
			

			// Adjust the elevator based on where it needs to go
			if (destination - current_position > 0) { // Move up
				current_position++;
				moved = true;
				direction = "Up";
			} else if (destination - current_position < 0) { // Move down
				current_position--;
				moved = true;

				direction = "Down";
			} else {
				direction = "Stationary";
			}
			
			 if (current_position % 4 == 0) {
				 current_floor = current_position;
			 }
			
			// As we have potentially changed the elevator position, lets redraw it
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
			moved = false;
		}
		
		// Handle any button or key inputs
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
	
	if (btn == BUTTON0_PUSHED) {
		// Move to Floor 0
		destination = FLOOR_0;
	} else if (btn == BUTTON1_PUSHED) {
		// Move to Floor 1
		destination = FLOOR_1;
	} else if (btn == BUTTON2_PUSHED) {
		// Move to Floor 2
		destination = FLOOR_2;
	
	} else if (btn == BUTTON3_PUSHED) {
		// Move to Floor 3
		destination = FLOOR_3;
	}
	
	// Check for if a '0, 1, 2, 3' is pressed
	// There are two steps to this
	// 1) collect any serial input (if available)
	// 2) check if the input is equal to the character 's'
	char serial_input = -1;
	if (serial_input_available()) {
		serial_input = fgetc(stdin);
	}
	// If the serial input is 's', then exit the start screen
	if (serial_input == '0') {
		destination = FLOOR_0;
	} else if (serial_input == '1') {
		destination = FLOOR_1;
	} else if (serial_input == '2') {
		destination = FLOOR_2;
	} else if (serial_input == '3') {
		destination = FLOOR_3;
	}

	}
	
char current_floor_tostring(ElevatorFloor floor) {
	switch (floor){
		case FLOOR_0: return "0";
		case FLOOR_1: return "1";
		case FLOOR_2: return "2";
		case FLOOR_3: return "3";
	}	
	return "Unknown";
}

char get_direction(ElevatorFloor current, ElevatorFloor destination) {
	
}