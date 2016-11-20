#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Timer.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 7);
#define ROW_COUNT(array) (sizeof(array) / sizeof(*array))

const byte SERIAL_IN_PIN = 2;
struct in_bytes {
	byte value;	
	bool set;	
};

// vars
in_bytes rx_bytes[64];
word rx_byte_index;
volatile word isr_times_buffer[60];
byte isr_index_buffer;
word max_step, min_step;
byte max_index, min_index;
unsigned long auto_baud;
unsigned long uart_baud;
unsigned long isr_micros;
Timer t;

// measure the delta b/w change interrupts on the serial pin
void isr_serial_change() {
	// update slots array
	if (isr_index_buffer > 59)
		isr_index_buffer = 0;
	isr_times_buffer[isr_index_buffer++] = micros() - isr_micros;
	isr_micros = micros();
}

// calculate the baud rate base on interrupt deltas
void update_time_buffer(void) {
	// search minimal/maximal time step
	max_step = 0;
	min_step = 0xFFFF;
	noInterrupts();
	for (byte i = 0; i < 60; i++) {
		if ((isr_times_buffer[i] != 0) && (isr_times_buffer[i] != 0xFFFF)) {
			if (isr_times_buffer[i] > max_step) {
				max_step = isr_times_buffer[i];
				max_index = i;
			}			
			if (isr_times_buffer[i] < min_step) {
				min_step = isr_times_buffer[i];
				min_index = i;
			}
		}
	}
	interrupts();
	// compute auto baudrate
	auto_baud = norm_baud(1e6 / min_step);

	if (auto_baud > 0 && auto_baud != uart_baud) {
		uart_baud = auto_baud;
		Serial.begin(norm_baud(auto_baud));				
	}	
}

void printBuffer(void) {
	display.clearDisplay();
	display.setCursor(0, 0);
	
	
	word i = rx_byte_index;
	word index = 0;
	word line = 0;
	while ((index < ROW_COUNT(rx_bytes))) {
		if (rx_bytes[i].set) {
			// no printable ascii -> set to 0x00
			char ascii = ((rx_bytes[i].value >= 0x20) && (rx_bytes[i].value <= 0x7e)) ? rx_bytes[i].value : ' ';
			display.print(ascii);	
		}
		index++;
		
		if (++i >= ROW_COUNT(rx_bytes))
			i = 0;

	}
	display.display();
	
	
}

// return true if std_baud baudrate and measured baudrate are within 10%
bool is_baud(long std_baud, long measure) {
	return (measure > (std_baud - std_baud * 0.1)) && (measure < (std_baud + std_baud *0.1));
}

// account for clock drift by picking the closest reasonable baud value
unsigned long norm_baud(unsigned long baud) {
	if (is_baud(300, baud))
		return 300;
	else if (is_baud(600, baud))
		return 600;
	else if (is_baud(1200, baud))
		return 1200;
	else if (is_baud(2400, baud))
		return 2400;
	else if (is_baud(4800, baud))
		return 4800;
	else if (is_baud(9600, baud))
		return 9600;
	else if (is_baud(14400, baud))
		return 14400;
	else if (is_baud(19200, baud))
		return 19200;
	else if (is_baud(28800, baud))
		return 28800;
	else if (is_baud(38400, baud))
		return 38400;
	else if (is_baud(57600, baud))
		return 57600;
	else if (is_baud(115200, baud))
		return 115200;
	else
		return 0;
}


void serialEvent() {
	while (Serial.available()) {		
		rx_bytes[rx_byte_index].value = (char)Serial.read();
		rx_bytes[rx_byte_index].set = true;
		if (++rx_byte_index >= ROW_COUNT(rx_bytes))
			rx_byte_index = 0;
	}
}

void setup() {
	rx_byte_index = 0;
	isr_index_buffer = 0;

	
	uart_baud = 9600;
	Serial.begin(uart_baud);
	int updateJob = t.every(500, update_time_buffer);
	int printJob = t.every(1000, printBuffer);

	display.begin();	
	display.setContrast(50);
	display.clearDisplay();	
	display.setTextSize(1);
	display.setTextColor(BLACK);
	display.display();


	pinMode(SERIAL_IN_PIN, INPUT);
	digitalWrite(SERIAL_IN_PIN, HIGH); //pullup
	attachInterrupt(0, isr_serial_change, CHANGE);


}


void loop() {
	t.update();
	
}