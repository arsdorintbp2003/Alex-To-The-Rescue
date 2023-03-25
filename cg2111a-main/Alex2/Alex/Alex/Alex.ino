#include <CircularBuffer.h>
#include <serialize.h>
#include <stdarg.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

// declaring Circular Buffer for serial communications
CircularBuffer<char, 200> recvBuffer; // Buffer storing data from Pi
CircularBuffer<char, 200> xmitBuffer; // Buffer storing data to sent to Pi

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      180

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.10619298

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

// Alex Pin declarations.
#define PIN7 (1 << 7)
#define PIN6 (1 << 6)
#define PIN5 (1 << 5)
#define PIN4 (1 << 4)
#define PIN2 (1 << 2)
#define PIN1 (1 << 1)
#define PIN0 1

// Ultrasonic pins
// Right Sensor
#define TRIG_PIN1 12 // PB4 
#define ECHO_PIN1 11 // PB3

// Left Sensor
#define TRIG_PIN2 8 // PB0 
#define ECHO_PIN2 7 // PD7


#define ALEX_LENGTH 17
#define ALEX_BREADTH 15

#define PI 3.141592654

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

// To use for ultrasonic calculations.
float SPEED_OF_SOUND = 0.0345;

// Ultrasonic distance variables;
float UltraLDist = 0;
float UltraRDist = 0;

// Ultrasonic Bypass command
bool master_command = false;

// Configuration constant for Left motor to turning movements
#define LConT 0.8
// Configuration constant for Left motor to front/back movements
#define LConFB 0.96

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Store the ticks from Alex's left and
// right encoders for turning.
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = UltraLDist;
  statusPacket.params[11] = UltraRDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  PIND |= 0b00001100;
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD || dir == BACKWARD) {
    if (dir == FORWARD) {
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    }
    else if (dir == BACKWARD) {
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    }
  }
  else if (dir == LEFT || dir == RIGHT) {
    if (dir == LEFT) {
      leftReverseTicksTurns++;
    }
    else if (dir == RIGHT) {
      leftForwardTicksTurns++;
    }
  }
  
  //dbprintf("LEFT: %d\n", leftForwardTicks);
}

void rightISR()
{
  if (dir == FORWARD || dir == BACKWARD) {
    if (dir == FORWARD) {
      rightForwardTicks++;
    }
    else if (dir == BACKWARD) {
      rightReverseTicks++;
    }
  }
  else if (dir == LEFT || dir == RIGHT) {
    if (dir == LEFT) {
      rightForwardTicksTurns++;
    }
    else if (dir == RIGHT) {
      rightReverseTicksTurns++;
    }
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  DDRD &= 0b11110011;
  EICRA |= 0b0001010; 
  EIMSK |= 0b00000011;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */

// Set up the serial connection
// with bare-metal code.
void setupSerial()
{
  // Setting Baud rate of 9600
  UBRR0L = 103;
  UBRR0H = 0;
  // Bits 7 & 6 set to 00 - Asynchronous mode
  // Bits 5 & 4 set to 00 - No parity checks
  // Bit 3 set to 0 - setting 1 stop bit
  // Bits 2 & 1 (UCSZ01/UCSZ00) set to 11 - Setting 8 bits 
  // data transmission.
  // Bit 0 (UCPOL0) always set to 0.
  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Since we are using only RXC interrupt while disabling UDIRE0 for now, we set RXCIE0 (bit 7)and UDIRE0 (bit 5) to 0.
  // TXCIE0 (bit 6) is disabled.
  // Bits 4 and 3 are set to 1 to enable receiver and transmitter respectively.
  // Bit 2 (UCSZ02) must be 0, to form the 0b011 we need to choose 8-bit data size.
  // Lastly, RXB0 and TXB0 are set to 0 as 9-bit data sizes are not used.
  UCSR0B = 0b10011000;
}

int readSerial(char *buffer)
{
  int count = 0;
  while (!recvBuffer.isEmpty()) 
  {
    // Reading from recvbuffer.
    buffer[count] = recvBuffer.pop();
    count++;
  }
  return count;
}

void writeSerial(const unsigned char *buffer, int len)
{
  TResult result = PACKET_OK;
  int i;
  for (i = 1; i < len && result == PACKET_OK; i++)
  {
    // Writing to transmit buffer.
    if (xmitBuffer.unshift(buffer[i]))
    {
      result = PACKET_OK;
    } else {
      result = PACKET_COMPLETE;
    }
  }
  // first byte is written to UDR0 to start the proverbial ball rolling.
  UDR0 = buffer[0];
  // UDRE interrupt (bit 5) enabled to start ISR.
  UCSR0B |= 0b00100000;
}


// Receive interrupt ISR.
ISR(USART_RX_vect)
{
  unsigned char data = UDR0;
  recvBuffer.unshift(data);
}

// Transmit interrupt ISR.
ISR(USART_UDRE_vect)
{
  unsigned char data;
  if (!xmitBuffer.isEmpty())
  {
    data = xmitBuffer.pop();
    UDR0 = data;
  } else {
      UCSR0B &= 0b11011111;
  }
}


/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
    DDRD |= (PIN5 | PIN6);
    DDRB |= (PIN1 | PIN2);
    TCNT0 = 0;
    TCNT1 = 0;
}

// Start the PWM for Alex's motors.
void startMotors()
{
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;
  if(speed > 100.0)
    speed = 100.0;
  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist =  + deltaDist;
  int val = pwmVal(speed);

/* Our motor set up is:  
   *    LR - Pin 5, PD5, OC0B, OCR0B - set PWM
   *    LF - Pin 6, PD6, OC0A, OCR0A - set PWM
   *    RF - Pin 10, PB2, OC1B, OCR1B - set PWM
   *    RR - Pin 9, PB1, OC1A, OCR1A - set PWM
*/

  // enabling compare matches for LF and RF only
  TCCR0A = 0b10000001;
  TCCR1A = 0b00100001;
  OCR0A = val*LConFB;
  OCR1B = val;
  OCR0B = 0;
  OCR1A = 0;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

  dir = BACKWARD;
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = reverseDist + deltaDist;
  int val = pwmVal(speed);

/*      Our motor set up is:  
   *    LR - Pin 5, PD5, OC0B, OCR0B - set PWM
   *    LF - Pin 6, PD6, OC0A, OCR0A - set PWM
   *    RF - Pin 10, PB2, OC1B, OCR1B - set PWM
   *    RR - Pin 9, PB1, OC1A, OCR1A - set PWM
*/
  
  // enabling compare matches for LR and RR only
  TCCR0A = 0b00100001;
  TCCR1A = 0b10000001;  
  OCR0B = val*LConFB;
  OCR1A = val;
  OCR0A = 0;
  OCR1B = 0;
}

// Function to get an estimate of the number of wheel ticks needed
// to turn a specific angle.
unsigned long computeDeltaTicks(float ang) 
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{  
  dir = LEFT; 
  int val = pwmVal(speed);
  if (ang == 0) 
    deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  /* Our motor set up is:  
   *    LR - Pin 5, PD5, OC0B, OCR0B - set PWM
   *    LF - Pin 6, PD6, OC0A, OCR0A - set PWM
   *    RF - Pin 10, PB2, OC1B, OCR1B - set PWM
   *    RR - Pin 9, PB1, OC1A, OCR1A - set PWM
   */
   
  // enabling compare matches for LR and RF only
  TCCR0A = 0b00100001;
  TCCR1A = 0b00100001;
  OCR0B = val*LConT;
  OCR1B = val;
  OCR0A = 0;
  OCR1A = 0;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT; 
  int val = pwmVal(speed);
  if (ang == 0) deltaTicks = 9999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
/*     Our motor set up is:  
  *    LR - Pin 5, PD5, OC0B, OCR0B - set PWM
  *    LF - Pin 6, PD6, OC0A, OCR0A - set PWM
  *    RF - Pin 10, PB2, OC1B, OCR1B - set PWM
  *    RR - Pin 9, PB1, OC1A, OCR1A - set PWM
*/
  // enabling compare matches for LF and RR only
  TCCR0A = 0b10000001;
  TCCR1A = 0b10000001;
  OCR0A = val;
  OCR1A = val;
  OCR0B = 0;
  OCR1B = 0;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  // Disabling all compare matches
  TCCR0A = 0b00000001;
  TCCR1A = 0b00000001;
  master_command = true;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns =0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
  switch(which)
  {
    case 0:
      leftForwardTicks = 0;
      break;

    case 1:
      rightForwardTicks=0;
      break;

    case 2:
      leftReverseTicks=0;
      break;

    case 3:
      rightReverseTicks = 0;
      break;

    case 4:
      leftForwardTicksTurns = 0;
      break;

    case 5:
      rightForwardTicksTurns = 0;
      break;

    case 6:
      leftReverseTicksTurns=0;
      break;

    case 7:
      rightReverseTicksTurns=0;
      break;

    case 8:
      forwardDist=0;
      break;

    case 9:
      reverseDist=0;
      break;
  }
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  
  if(command->command == COMMAND_GET_STATS)
  {
    UltraRDist = Ultrasonic_R_dist();
    UltraLDist = Ultrasonic_L_dist();
    sendStatus();
    return;
  }
  if(command->command == COMMAND_STOP){
    sendOK();
    stop();
    // By passing the ultrasonic distance measurement for master_command.
    UltraRDist = 10;
    UltraLDist = 10;
    return;
  }
  if(UltraRDist <= 5.5){
    sendMessage("R too close! s to reset");
    return;
  }
  if(UltraLDist <= 5.5){
    sendMessage("L too close! s to reset");
    return;
  }
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
        
    default:
      sendBadCommand();
  }
}

// Function not used. 
void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {     
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setupUltrasonic()
{
  // Setting the right side ultrasonic sensor
  // trigger and echo pins as output and input respectively
  DDRB |= PIN4; // Trigger Pin output
  DDRB &= ~PIN3; // Echo Pin input

  // Setting the left side ultrasonic sensor
  // trigger and echo pins as output and input respectively
  DDRB |= PIN0; // Trigger Pin output
  DDRD &= ~PIN7; // Echo Pin input
}

float Ultrasonic_R_dist()
{
  PORTB |= PIN4; // Trigger Pin set to high.
  delayMicroseconds(10);
  PORTB &= ~PIN4; // Trigger Pin set to low.
  float microsecs = pulseIn(ECHO_PIN1, HIGH);
  float dist_cms = microsecs*SPEED_OF_SOUND/2;
  return dist_cms;
}

float Ultrasonic_L_dist()
{
  PORTB |= PIN0; // Trigger Pin set to high.
  delayMicroseconds(10);
  PORTB &= ~PIN0; // Trigger Pin set to low.
  float microsecs = pulseIn(ECHO_PIN2, HIGH);
  float dist_cms = microsecs*SPEED_OF_SOUND/2;
  return dist_cms;
}

void setup() {
  // put your setup code here, to run once:
  AlexDiagonal =sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = AlexDiagonal * PI;
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  setupUltrasonic();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;
    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    } 
    else 
      if(result == PACKET_CHECKSUM_BAD) 
      {
        sendBadChecksum();
      } 
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
        master_command = false;
      }
    } else if (dir == BACKWARD) {
        if (reverseDist > newDist)
        {
          deltaDist = 0;
          newDist = 0;
          stop();
          master_command = false;
        }
      } else if (dir == STOP) {
            deltaDist = 0;
            newDist = 0;
            stop();
            master_command = false;
        }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        master_command = false;
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        master_command = false;
      }
    } else if (dir == STOP) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        master_command = false;
      }
  }

  if (dir != STOP && !master_command) 
  {
    UltraRDist = Ultrasonic_R_dist();
    UltraLDist = Ultrasonic_L_dist();
    if (UltraRDist <= 5.5 | UltraLDist <= 5.5) 
    {
      deltaTicks = 0;
      deltaDist = 0;
      stop();
    }
  }
}