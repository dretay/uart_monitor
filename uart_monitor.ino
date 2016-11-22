#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Timer.h>
#include <Button.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 7);
#define ROW_COUNT(array) (sizeof(array) / sizeof(*array))
#define LONG_PRESS 1000

const byte SERIAL_IN_PIN = 2;
struct in_bytes {
	byte value;	
	bool set;	
};

// vars
in_bytes rx_bytes[78];
word rx_byte_idx;
volatile word isr_times_buffer[60];
byte isr_idx_buffer;
word max_step, min_step;
byte max_idx, min_idx;
unsigned long isr_micros;
bool manual_baud = false;
Timer t;
Button clearButton(8, false, false, 20);
Button manualBaudButton(9, false, false, 20);
unsigned short baud_rate_idx;
const long baud_rates[] = { 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200 };
const int total_baud_rates = sizeof(baud_rates) / sizeof(long);

// measure the delta b/w change interrupts on the serial pin
void isr_serial_change() {
	// update slots array
	if (isr_idx_buffer > 59) {
		isr_idx_buffer = 0;
	}
	isr_times_buffer[isr_idx_buffer++] = micros() - isr_micros;
	isr_micros = micros();
}

// calculate the baud rate base on interrupt deltas
void update_time_buffer(void) {
	if (!manual_baud) {
		// search minimal/maximal time step
		max_step = 0;
		min_step = 0xFFFF;
		noInterrupts();
		for (byte i = 0; i < 60; i++) {
			if ((isr_times_buffer[i] != 0) && (isr_times_buffer[i] != 0xFFFF)) {
				if (isr_times_buffer[i] > max_step) {
					max_step = isr_times_buffer[i];
					max_idx = i;
				}
				if (isr_times_buffer[i] < min_step) {
					min_step = isr_times_buffer[i];
					min_idx = i;
				}
			}
		}
		interrupts();
		// compute auto baudrate
		int idx = norm_baud_idx(1e6 / min_step);

		if (idx >= 0 && baud_rates[idx] != baud_rates[baud_rate_idx]) {
			baud_rate_idx = idx;
			flash_baud();
			Serial.begin(baud_rates[baud_rate_idx]);
		}
	}
}

void printBuffer(void) {
	display.clearDisplay();
	display.setCursor(0, 0);	
	
	word i = rx_byte_idx;
	word idx = 0;	
	while ((idx < ROW_COUNT(rx_bytes))) {
		if (rx_bytes[i].set) {
			// no printable ascii -> set to 0x00
			char ascii = ((rx_bytes[i].value >= 0x20) && (rx_bytes[i].value <= 0x7e)) ? rx_bytes[i].value : ' ';
			display.print(ascii);	
		}
		idx++;
		
		if (++i >= ROW_COUNT(rx_bytes)) {
			i = 0;
		}

	}
	display.display();
	
	
}

// return true if std_baud baudrate and measured baudrate are within 10%
bool is_baud(long std_baud, long measure) {
	return (measure > (std_baud - std_baud * 0.1)) && (measure < (std_baud + std_baud *0.1));
}

// account for clock drift by picking the closest reasonable baud value
unsigned short norm_baud_idx(unsigned long baud) {
	unsigned short i = 0;
	for (; i < total_baud_rates; i++) {
		if (is_baud(baud_rates[i], baud)) {
			return i;
		}
	}
	return -1;
}


void serialEvent() {
	while (Serial.available()) {		
		rx_bytes[rx_byte_idx].value = (char)Serial.read();
		rx_bytes[rx_byte_idx].set = true;
		if (++rx_byte_idx >= ROW_COUNT(rx_bytes))
			rx_byte_idx = 0;
	}
}

void setup() {
	rx_byte_idx = 0;
	isr_idx_buffer = 0;
	baud_rate_idx = 0;

	
	Serial.begin(baud_rates[baud_rate_idx]);
	int updateJob = t.every(500, update_time_buffer);
	int printJob = t.every(1000, printBuffer);

	display.begin();	
	display.setContrast(60);
	display.clearDisplay();	
	display.setTextSize(1);
	display.setTextColor(BLACK);
	display.display();


	pinMode(SERIAL_IN_PIN, INPUT);
	digitalWrite(SERIAL_IN_PIN, HIGH); //pullup
	attachInterrupt(0, isr_serial_change, CHANGE);

}

void flash_baud() {
	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE, BLACK);
	if (manual_baud) {
		display.println("MANUAL");
	}
	else {
		display.println("AUTO");
	}	
	display.println(baud_rates[baud_rate_idx]);
	display.display();
	delay(500);
	
	display.setTextSize(1);
	display.setTextColor(BLACK);

}

void loop() {
	t.update();
	clearButton.read();
	if (clearButton.wasReleased()) {
		int i = 0;
		for (; i < ROW_COUNT(rx_bytes); i++) {
			rx_bytes[i].set = false;
		}
	}

	manualBaudButton.read();
	if (manualBaudButton.wasReleased()) {		
		if (++baud_rate_idx == total_baud_rates) {
			manual_baud = false;
			baud_rate_idx = 0;
			flash_baud();
		}
		else {
			manual_baud = true;
			flash_baud();
			Serial.begin(baud_rates[baud_rate_idx]);
		}
	}
	
}