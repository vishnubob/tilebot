#include "NewSoftSerial.h"
//#include "pololu/orangutan.h"
//#include "SoftwareSerial.h"

#define rx_barcode_pin 12
#define tx_barcode_pin 11

#define rx_pololu_pin 9
#define tx_pololu_pin 8

// set up a new serial port
NewSoftSerial bcode_port(rx_barcode_pin, tx_barcode_pin);
NewSoftSerial robot_port(rx_pololu_pin, tx_pololu_pin);

#define BUTTON_C        (1 << 5)
#define BUTTON_B        (1 << 4)
#define BUTTON_A        (1 << 1)
#define BUTTON_ALL      (BUTTON_A | BUTTON_B | BUTTON_C)

char barcode_buf[32];
char robot_buf[32];
volatile bool new_barcode;

void play_tune(char *tune)
{
    int tune_len = strlen(tune);
    if (tune_len > (sizeof(robot_buf) - 2))
    { 
        return; 
    }
    robot_buf[0] = '\xB3';
    robot_buf[1] = (char)tune_len;
    bcopy(tune, robot_buf + 2, tune_len);
    send_robot(robot_buf, tune_len + 2);
}

char get_buttons()
{
    send_robot("\xC7",1);
    recv_robot(robot_buf, 1, 5);
    return robot_buf[0];
}

bool button_press(char button)
{
    return get_buttons() & button;
}
    

void send_robot(char *buf, int buflen)
{
    while(buflen)
    {
        robot_port.print(*buf);
        buf++;
        buflen--;
    }
}

void recv_robot(char *buf, int expect, int timeout)
{
    while(timeout && expect)
    {
        if (robot_port.available())
        {
            *buf = robot_port.read();
            buf++;
            expect--;
        } else
        {
            timeout--;
            delay(100);
        }
    }
}

// set the motor speeds
void slave_set_motors(int speed1, int speed2)
{
	char message[4] = {0xC1, speed1, 0xC5, speed2};
	if(speed1 < 0)
	{
		message[0] = 0xC2; // m1 backward
		message[1] = -speed1;
	}
	if(speed2 < 0)
	{
		message[2] = 0xC6; // m2 backward
		message[3] = -speed2;
	}
	send_robot(message,4);
}

// do calibration
void slave_calibrate()
{
	send_robot("\xB4",1);
	int tmp_buffer[5];

	// read 10 characters (but we won't use them)
	//serial_receive_blocking((char *)tmp_buffer, 10, 100);
}

// reset calibration
void slave_reset_calibration()
{
	send_robot("\xB5",1);
}

// calibrate (waits for a 1-byte response to indicate completion)
void slave_auto_calibrate()
{
	int tmp_buffer[1];
	send_robot("\xBA",1);
	//serial_receive_blocking((char *)tmp_buffer, 1, 10000);
}

// sets up the pid constants on the 3pi for line following
void slave_set_pid(char max_speed, char p_num, char p_den, char d_num, char d_den)
{
	char string[6] = "\xBB";
	string[1] = max_speed;
	string[2] = p_num;
	string[3] = p_den;
	string[4] = d_num;
	string[5] = d_den;
	send_robot(string,6);
}

// stops the pid line following
void slave_stop_pid()
{
	send_robot("\xBC", 1);
}

void read_barcode()
{
    new_barcode = true;
}

void read_barcode_actual()
{
    detachInterrupt(0);
    int timeout = 10;
    char *ptr = barcode_buf;
    while(timeout)
    {
        if (bcode_port.available())
        {
            char ch = bcode_port.read();
            if (ch == '\r')
            {
                break;
            }
            *ptr = ch;
            ptr++;
        } else
        {
            delay(100);
            timeout--;
        }
    }
    *ptr = 0;
    attachInterrupt(0, read_barcode, LOW);
}

void configure_barcode_scanner()  
{
    bcode_port.begin(9600);
    Serial.println("barcode config");
    attachInterrupt(0, read_barcode, LOW);
    new_barcode = false;
}

void configure_robot()
{
	robot_port.begin(9600);
    send_robot("\x81",1);
    recv_robot(robot_buf, 6, 5);
    Serial.println(robot_buf);
    
    // announce we're ready!
    play_tune("l16o6gab>c");
}

void setup()
{
    // set the data rate for the SoftwareSerial port
    Serial.begin(9600);

    configure_barcode_scanner();
    configure_robot();
}

void loop()
{
    char button = get_buttons();
    if (button)
    {
        if (button & BUTTON_A)
            Serial.print("A");
        if (button & BUTTON_B)
            Serial.print("B");
        if (button & BUTTON_C)
            Serial.print("C");
        Serial.println("");
    }
    
    if(new_barcode)
    {
        read_barcode_actual();
        Serial.print("barcode: ");
        Serial.println(barcode_buf);
        char tune[] = "\xB3 l16o6gab>c";
        tune[1] = sizeof(tune)-3;
        send_robot(tune,sizeof(tune)-1);
        new_barcode = false;
    }
}

