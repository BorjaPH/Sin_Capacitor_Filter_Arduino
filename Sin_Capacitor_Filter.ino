#include <Arduino.h>
#include <stdint.h>
#include <avr/interrupt.h>

/*
 * TEST_INPUT_CAPTURE: this macro constant allows to test the frequency_counter
 * on the same Arduino UNO board. In this case, it uses TIMER1_OVERFLOW_PIN to 
 * generate a digital periodic signal. This pin happens to be pin 13, where is
 * the Arduino's led, so you will see a blinking led at a specific period. Keep
 * this constant commented if you do not want to test this program on the same 
 * Arduino board.
 */
#define TEST_INPUT_CAPTURE

#ifdef TEST_INPUT_CAPTURE

const int TIMER1_OVERFLOW_PIN=13;
#define TIMER1_OVERFLOW_PIN_PORT PORTB // Arduino's pin 13
const int TIMER1_OVERFLOW_PIN_BIT=5;

/*
 * Constant TIMER1_OVERFLOW_COUNTS determines the period of the digital
 * test signal. It is a multiple of the TIMER1 clock prescaler period the 
 * TOP value set for TIMER1.
 */
const int TIMER1_OVERFLOW_COUNTS=100; // aproximately 409.6 msecs (100*4.096 msecs.)

volatile bool timer1_overflow_pin_status=false;
volatile int timer1_overflow_counter=0;

#endif // TEST_INPUT_CAPTURE

/*
 * Constant INPUT_CAPTURE_PIN specifies the input capture pin, which in Arduino
 * UNO board is pin 2. In this pin it is where the input signal to measure should 
 * be set.
 */
#define OC2A_PIN 11
const int INPUT_CAPTURE_PIN=2;
#define INPUT_CAPTURE_PIN_PORT PORTD // Arduino's pin 2
const int INPUT_CAPTURE_PIN_BIT=2;
volatile bool input_capture_pin_status=false;

#define TIMER1_OVERFLOW_VALUE ((uint64_t)(1<<10))
#define TIMER1_OVERFLOW_HALF_VALUE (1<<9)

volatile uint64_t timer1_overflows=0;

/*
 * Macro constant TIMESTAMPS_LENGTH is the length of an internal buffer of capture
 * measurements. The application fills this buffer with TIMESTAMPS_LENGTH captures,
 * and them shows the times collected between captures (which should be the period
 * of the signal) through the Serial interface
 */
#define TIMESTAMPS_LENGTH 40+1 // should be less than 256

#define SECS_PER_TIMER1_TICK (double(4e-6))
#define SECS_FRACTIONAL_DIGITS 6
#define HZ_FRACTIONAL_DIGITS 6

volatile uint64_t timestamps[TIMESTAMPS_LENGTH];
volatile uint8_t current_timestamp;
volatile uint32_t period;
volatile float t;


// interrupt vector for timer1 overflow event
ISR(TIMER1_OVF_vect)
{
  timer1_overflows+=TIMER1_OVERFLOW_VALUE;
  
#ifdef TEST_INPUT_CAPTURE  
  timer1_overflow_counter++;
  if(timer1_overflow_counter==TIMER1_OVERFLOW_COUNTS)
  {
    timer1_overflow_counter=0;
    
    // toggle timer1 overflow pin
    timer1_overflow_pin_status=!timer1_overflow_pin_status;
    if(timer1_overflow_pin_status) bitSet(TIMER1_OVERFLOW_PIN_PORT,TIMER1_OVERFLOW_PIN_BIT);
    else bitClear(TIMER1_OVERFLOW_PIN_PORT,TIMER1_OVERFLOW_PIN_BIT);
  }
#endif // TEST_INPUT_CAPTURE
}

// interrupt vector for Timer1 Input Capture event
ISR(TIMER1_CAPT_vect)
{
  uint16_t icr1_register=ICR1;
  TCCR1B ^= (1<<(ICES1));
  TIFR1 |= (1<<(ICF1));
  
  if(current_timestamp<TIMESTAMPS_LENGTH)
  {
    timestamps[current_timestamp]=timer1_overflows+icr1_register;
    
    if(
     (TIFR1 & (1<<TOV1)) && // is timer1 overflow flag set? and
     (icr1_register<TIMER1_OVERFLOW_HALF_VALUE) // did the capture take place after the overflow?
    ) timestamps[current_timestamp]+=TIMER1_OVERFLOW_VALUE;

    current_timestamp++;
  }
  
  // toggle timer1 input capture pin
  input_capture_pin_status=!input_capture_pin_status;
  if(input_capture_pin_status) bitSet(INPUT_CAPTURE_PIN_PORT,INPUT_CAPTURE_PIN_BIT);
  else bitClear(INPUT_CAPTURE_PIN_PORT,INPUT_CAPTURE_PIN_BIT);
}

ISR(TIMER2_OVF_vect)
{
  t += 1.024;
  uint64_t sen_signal = 2500 + (2500 * sin((2 * M_PI * t) / period));
  sen_signal = map(sen_signal, 0, 5000, 0, 255);
  OCR2A = (int)sen_signal;
}

// function prototypes
void print_uint64_t(const uint64_t* value);
 
// the setup routine runs once when you press reset:
void setup() {       
  
#ifdef TEST_INPUT_CAPTURE  
  // initialize timer1 overflow pin as output
  pinMode(TIMER1_OVERFLOW_PIN,OUTPUT);
  bitClear(TIMER1_OVERFLOW_PIN_PORT,TIMER1_OVERFLOW_PIN_BIT); 
#endif // TEST_INPUT_CAPTURE
  
  // initialize timer1 input capture pin
  pinMode(INPUT_CAPTURE_PIN,OUTPUT);
  bitClear(INPUT_CAPTURE_PIN_PORT,INPUT_CAPTURE_PIN_BIT); 
  
  //initialize the serial port  
  Serial.begin(9600);
  pinMode(OC2A_PIN,OUTPUT);
  current_timestamp=0;
  period=0;
  t=0;

  while (!printSetOutput()) {}
  SetTimer1();
  SetTimer2();
  
  Serial.println("----> capturing signal ...");
  
  interrupts();
}

// the loop routine runs over and over again forever:
void loop() 
{
  if(current_timestamp==TIMESTAMPS_LENGTH)
  {
    Serial.print("----> "); Serial.print(TIMESTAMPS_LENGTH); Serial.println(" timestamps taken");
    
    uint64_t previous=*timestamps, current;
    for(int i=1; i<TIMESTAMPS_LENGTH; i++)
    {
      current=timestamps[i];
      uint64_t difference=current-previous;
      double difference_in_seconds= double(difference)*SECS_PER_TIMER1_TICK;
      double frequency=1/difference_in_seconds;

      Serial.print("  "); Serial.print(i); Serial.print(": "); 
      print_uint64_t(current); Serial.print('-');
      print_uint64_t(previous); Serial.print('=');
      print_uint64_t(difference); Serial.print("(");
      Serial.print(difference_in_seconds,SECS_FRACTIONAL_DIGITS); 
      Serial.print(" s. <-> ");
      Serial.print(frequency,HZ_FRACTIONAL_DIGITS);
      Serial.print(" Hz)");
      if (i%2 == 0) {
        Serial.println("[Not duty time]");
      }else {
        Serial.println("[Duty time]");  
      }
      previous=current;
    }
    Serial.println("----> continue? ");
    
    byte key=0;
    while( (key!='y') && (key!='Y') )
    {
      if(Serial.available()>0) key=Serial.read();
    }
    
    while (!printSetOutput()) {}
    
    noInterrupts();
    SetTimer1();
    SetTimer2();
    interrupts();
    
    current_timestamp=0;
    
    Serial.println("----> capturing signal ...");
  }
}

boolean printSetOutput() {
  Serial.println("Please, enter the output to generate --> Period [value]");
  Serial.println("Please, remeber that the output value must be between 0-1000 ms");  
  while (!Serial.available()) {}
  String command = Serial.readStringUntil('\n');
  if (command.startsWith("Period ")) {
    period = command.substring(7).toInt();
    Serial.print("Value = "); Serial.println(period);
    if (period >= 0 && period <= 1000 ) {
      return true; 
    }else {
      Serial.println("Bad Number. Please try again");
      }
  }
  return false;
}

uint8_t SelectPrescaler (uint16_t value) {
    switch(value) {
      case 0:
        return 1;
      case 8:
        return 2;
      case 32:
        return 3;
      case 64:
        return 4;
      case 128:
        return 5;
      case 256:
        return 6;
      case 1024:
        return 7;
      default:
        return 8;
      }  
}
  
void print_uint64_t(const uint64_t& value)
{
  constexpr static size_t uint64_t_max_length=64;
  static char uint64_t_str__[uint64_t_max_length];

  uint64_t q=value; size_t digit=0;
  char* ptr=&(uint64_t_str__[uint64_t_max_length-1]); 
  *(ptr--)=0;
  while(
    (digit<uint64_t_max_length-1) && (q>0)
  )
  { *(ptr--)='0'+q%10; q=q/10; digit++; }

  Serial.print(ptr+1);   
}

void SetTimer1() {
  Serial.println("Setting Timer 1 registers...");
  // Timer1 Input Capture settings:
  //   - activating the noise canceller (implies a 4-oscillator-cycle delay)
  //   - setting falling edge for input capture signal
  //   - setting fast 10-bit pwm mode
  //     (supposing a prescaler of 64 -> T=4 usecs)
  TCCR1A=0;
  TCCR1B=0;
  TCCR1A |= (1<<(WGM11)) | (1<<(WGM10));
  TCCR1B |= (1<<(ICNC1)) | (1<<(WGM12)) | (1<<(CS11)) | (1<<(CS10)) | (1<<(ICES1));
  TCCR1B &= ~(1<<(WGM13)) & ~(1<<(CS12));
  TIMSK1 |= (1<<(ICIE1)) | (1<<(TOIE1));
}

void SetTimer2() {
  Serial.println("Setting Timer 2 registers...");
  // Timer2settings:
  //   - fast pwm with max on 0xff
  //   - prescaler of 64
  //   - OC2A pwm not inverted output
  //   - OC2B pwm not inverted output
  TCCR2A=0;
  TCCR2B=0;
  TCCR2A |= (1<<(WGM21)) | (1<<(WGM20));
  TCCR2A |= (1<<(COM2A1)) | (1<<(COM2B1));
  TCCR2B = 4; 
  TCCR2B &= ~(1<<(WGM22));
  TIMSK2 |= (1<<(TOIE2));
  
  // stablishing a duty cycle of 50%
  OCR2A=127; // this is Arduino UNO pin 11 
}
