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

#define START_CODE          1
#define STOP_CODE           2
#define FORWARD_CODE        3
#define REVERSE_CODE        4
#define TURN_LEFT_CODE      5
#define TURN_RIGHT_CODE     6

#define MOVEMENT_DELAY      1000

char barcode_buf[32];
char robot_buf[32];
unsigned char command_que[32];
volatile bool new_barcode;
unsigned char command_cnt;

void play_tune(char *tune)
{
    int tune_len = strlen(tune);
    if (tune_len > (sizeof(robot_buf) - 2))
    { 
        return; 
    }
    robot_buf[0] = '\xB3';
    robot_buf[1] = (char)tune_len;
    memcpy(robot_buf + 2, tune, tune_len);
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
        robot_port.print(*buf++);
        buflen--;
    }
}

void recv_robot(char *buf, int expect, int timeout)
{
    while(timeout && expect)
    {
        if (robot_port.available())
        {
            *buf++ = robot_port.read();
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
    recv_robot(robot_buf, 1, 10000);
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
            *ptr++ = ch;
        } else
        {
            delay(100);
            timeout--;
        }
    }
    bcode_port.flush();
    *ptr = 0;
    new_barcode = false;
    //attachInterrupt(0, read_barcode, LOW);
}

void configure_barcode_scanner()  
{
    bcode_port.begin(9600);
    Serial.println("barcode config");
    //attachInterrupt(0, read_barcode, LOW);
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

bool collect_codes()
{
    Serial.println("CC");
    play_tune("l16o6gaga");
    slave_auto_calibrate();
    unsigned char *head = command_que;
    command_cnt = 0;
    unsigned char code;
    new_barcode = false;
    slave_set_pid(20, 1, 20, 3, 2);

    unsigned long ts_stop = millis() + 60UL * 1000UL;
    unsigned long ts_barcode = 0;

    Serial.println(ts_stop);

    while (1)
    {
        unsigned long ts_now = millis();
        if (ts_now > ts_stop)
        {
            break;
        }
        if((ts_now > ts_barcode))
        {
            attachInterrupt(0, read_barcode, LOW);
        }
        if(new_barcode)
        {
            ts_barcode = ts_now + 500UL;
            read_barcode_actual();
            char *ptr = barcode_buf;
            while(*ptr)
            {
                code = (*ptr++) - '0';
                if ((code >= 1) && (code <= 6))
                {
                    break;
                }
            }
            if ((code <= 0) || (code > 6))
            {
                slave_stop_pid();
                Serial.println("badcode");
                play_tune("l4o4caca");
                return false;
            }
            if (code == STOP_CODE)
            {
                slave_stop_pid();
                play_tune("l16o6gg");
                return true;
            }
            play_tune("l16o6gg");
            Serial.print("code: ");
            Serial.println(code + '0');
            *head++ = code;
            ++command_cnt;
        }
    }
    slave_stop_pid();
    play_tune("l4o6gg");
    return false;
}

void run_codes()
{
    Serial.println("RC");
    unsigned char *head = command_que;
    for (unsigned char idx = 0; idx < command_cnt; ++idx)
    {
        unsigned char code = command_que[idx];
        switch (code)
        {
            case START_CODE:
                /* technically a NOP */
                break;
            case STOP_CODE:
                return;
                break;
            case FORWARD_CODE:
                slave_set_motors(20, 20);
                delay(MOVEMENT_DELAY);
                slave_set_motors(0, 0);
                break;
            case REVERSE_CODE:
                slave_set_motors(-20, -20);
                delay(MOVEMENT_DELAY);
                slave_set_motors(0, 0);
                break;
            case TURN_LEFT_CODE:
                slave_set_motors(-20, 20);
                delay(MOVEMENT_DELAY);
                slave_set_motors(0, 0);
                break;
            case TURN_RIGHT_CODE:
                slave_set_motors(20, -20);
                delay(MOVEMENT_DELAY);
                slave_set_motors(0, 0);
                break;
        }
    }
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
    /* wait for button push */
    while (1)
    {
        for(int x = 0; x < 20; ++x)
        {
            button_press(BUTTON_A);
            robot_port.flush();
        }

        if (button_press(BUTTON_A))
        {
            Serial.println("inner loop");
            if (collect_codes())
            {
                break;
            }
        }
    }

    for(int x = 0; x < 20; ++x)
    {
        button_press(BUTTON_A);
        robot_port.flush();
    }

    while (1)
    {
        if (button_press(BUTTON_A))
        {
            break;
        }
    }
    run_codes();
}

