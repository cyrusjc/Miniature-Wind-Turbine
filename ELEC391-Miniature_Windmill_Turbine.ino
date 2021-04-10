#include <Automaton.h>
#include <AccelStepper.h>
#include <QueueArray.h>

//////////////////// [ INIT GLOBAL CLASSES ] ///////////////////////////////

// Automaton Analog  Interrupt Machine
Atm_analog voltage_meter;
Atm_analog current_meter;
Atm_analog boost_converter;

// Stepper control step,  1, 3,  2, 4
AccelStepper stepper1(8, 11, 7, 10, 8);


//Dynamic Datastructures
QueueArray <int> queue_1;
QueueArray <int> queue_2;

//Automaton timer machines
Atm_timer timer_1;
Atm_timer timer_2;

//////////////////// [ Constant Variables ] /////////////////////////////////

// Wind position controller settings
const int DEFAULT_STEPS    = 90;

// ADC readings scale factors
const float CURRENT_SCALE_FACTOR       = 0.0;
const float BOOST_VOLTAGE_SCALE_FACTOR = 0.0;
const float VOLTAGE_SCALE_FACTOR       = 0.0;

// External Interrupts
const byte INTERRUPT_PIN_1 = 2;
const byte INTERRUPT_PIN_2 = 3;

/////////////////// [ Global variables ] ///////////////////////////////////
/*
 * [ STATES ]
 * 
 * INIT -> 0
 * RUN  -> 1
 * 
 */

float current_voltage       = 0.0;
float current_current       = 0.0;
float current_boost_voltage = 0.0;
float stored_power          = 0.0;
float stored_voltage        = 0.0;
float duty                  = 0.9;
int   interrupt_1_time      = 0;
int   interrupt_2_time      = 0;



void setup() {
  
  //  [ DEBUG ]
  Serial.begin(9600);

  //  [ INIT ]
  // NOTE: Order of initialization important due to dependency injection of machines
  
  //Init External Interrupts
  init_external_interrupts();

  //Init timer interrupts
  init_timer_interrupts();

  // Init voltage meter
  init_voltage_meter();

  // Init current meter
  init_current_meter();

  // Init Voltage Meter Boost Converter
  init_voltage_meter_boost();

  //Init timers
  init_timer_machines();

  // Init stepper
  init_stepper();
}

void loop() {
  // Run automaton machines
  automaton.run();
  // Run stepper
  stepper1.run();
  // Debugging
  //print_readings();
}

/////////////////////// [ EXTERNAL INTERRUPTS ] ///////////////////////////////

/*
 * init_external_interrupts: Initializes external interrupts to read signals from rotor encoder
 *                           to adjust based on wind detector
 *                           -> Uses pin 2,3 (Arduino designated pins for external interrupts)
 */
void init_external_interrupts() {
  Serial.println("External Interrupts Initialized");
  pinMode(INTERRUPT_PIN_1, INPUT_PULLUP);                                                // Set pins to be active on pull-up signal
  pinMode(INTERRUPT_PIN_2, INPUT_PULLUP);                                                // Set pins to be active on pull-up signal
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interrupt_callback_1, RISING); // Set external interrupt on pin_1. Attach callback <vector>.
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), interrupt_callback_2, RISING); // Set external interrupt on pin_2. Attach callback <vector>.
}

/*
 * interrupt_callback_1: Handles interrupts trigger on interrupt_pin_1.
 *                       -> Gets current time elpsed since progam started in ms
 *                       -> Dicards garbage values and noise by comparing current time elapsed (T) 
 *                          with the last recorded time elapsed (T-1)
 *                       -> Pushes time elapsed to queue_1 and records current elapsed time
 */

void interrupt_callback_1(){
  // Get current time with respecto to counter
  int _time = millis();
  if (_time <= interrupt_1_time + 200) { return; }
  // Push value to queue 
  queue_1.enqueue(_time);
  // Record value
  interrupt_1_time = _time;
  Serial.print("Callback1 ");
  Serial.println(_time);
}

/*
 * interrupt_callback_2: Handles interrupts trigger on interrupt_pin_2.
 *                       -> Gets current time elpsed since progam started in ms
 *                       -> Dicards garbage values and noise by comparing current time elapsed (T) 
 *                          with the last recorded time elapsed (T-1)
 *                       -> Pushes time elapsed to queue_2 and records current elapsed time
 */
void interrupt_callback_2(){
  //Get current time with respecto to counter
  int _time = millis();
  if (_time <= interrupt_2_time + 200) { return; }
  // Push value to queue 
  queue_2.enqueue(_time);
  // Record value
  interrupt_2_time = _time;
  Serial.print("Callback2 ");
  Serial.println(_time);
}

////////////////////////////////// [ TIMER INTERRUPTS ] //////////////////

/*
 * init_timer_interrupts: Timer 1 interrupt set to work at FAST PWM Mode
 *                        -> Frequency and Duty cycle set by wrtting on:
 *                           -> Input Capture Register (Frequency)
 *                           -> Ouput Compare Register (Duty Cycle)
 *                        -> As outlined on Atmega328 datasheet, FAST PWM is set by:
 *                           -> Enabling the COM1A1(R/W), and WGM11(R/W) bits of TCCR1A register
 *                           -> Enabling the CS10(R/W), WGM12(R/W), and WGM13(R/W) bits of TCCR1B register
 */
void init_timer_interrupts() {
  Serial.println("Timer Interrupts Initialized");
  // Set pin to output pwm (Pin 9 is controlled by Arduino's Timer 1)
  pinMode(9, OUTPUT);
  cli();                                        //Stop interrupts (Function located on system's global header file)
  TCCR1A = 0;                                   // Clear TCCR1A
  TCCR1B = 0;                                   // Clear TCCR1B
  TCNT1  = 0;                                   // Initialize counter value to 0
  //Set register bits to enable 16-bit resolution Fast PWM Mode
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(CS10) | _BV(WGM12) | _BV(WGM13);

  ICR1 = 1000;                                  // Input Capture Register set to 1000. Frequency approx 15,500,000/1000 = 15.5kHz
  OCR1A = duty*1000;                            // Output Compare Register set to a ratio of pwm signal length (0.00<->1.00)*ICR1 = Duty Cycle
  sei();                                        //Allow interrupts (Function located on system's global header file)
}

/////////////////////// [ VOLTAGE METER ] //////////////////////

/*
 * init_voltage_meter: Initializes Automaton analog state machine for voltage readings
 */
 void init_voltage_meter() {
      Serial.println("Voltage Meter Initialized");
      voltage_meter.begin(A1, 50) //For now analog read is polled every 50 ms
      .onChange(voltage_callback, 3);
      //TODO: Calibrate poll rate to optimal rate after integarting all hardware components
 }

 /*
 * voltage_callback: Callback triggered every time wind_detector sense changes in its analog read.
 * 
 * params: idx - Index of callback being triggered
 *         v   - Updated value of wind_detector read
 *         up  - Flag indicating if updated values is greater [ 1 ] or less than old [ 0 ] value
 * TODO: Calculate conversion factor for ADC readings when integrated with hardware
 */
void voltage_callback( int idx, int v, int up ) {
    current_voltage = (float) v;
}
 
/////////////////////// [ CURRENT METER ] ///////////////////////////////

/*
 * init_current_meter
 */
void init_current_meter() {
  Serial.println("Current Meter Initialized");
  current_meter.begin(A2, 50)
  .onChange(current_callback, 3);
}

/*
 * current_callback: Callback triggered every time wind_detector sense changes in its analog read.
 * 
 * params: idx - Index of callback being triggered
 *         v   - Updated value of wind_detector read
 *         up  - Flag indicating if updated values is greater [ 1 ] or less than old [ 0 ] value
 * TODO: Calculate conversion factor for ADC readings when integrated with hardware
 */
void current_callback( int idx, int v, int up ) {
  current_current = (float) v;
}

/////////////////////// [ BOOST VOLTAGE METER ] //////////////////////

/*
 * init_voltage_meter_boost: Initializes Automaton analog state machine for voltage readings after boost converter
 */
 void init_voltage_meter_boost() {
      Serial.println("Voltage Meter (Boost) Initialized");
      boost_converter.begin(A3, 50)
      .onChange(boost_callback, 3);
 }

 /*
 * boost_callback: Callback triggered every time wind_detector sense changes in its analog read.
 * 
 * params: idx - Index of callback being triggered
 *         v   - Updated value of wind_detector read
 *         up  - Flag indicating if updated values is greater [ 1 ] or less than old [ 0 ] value
 * TODO: Calculate conversion factor for ADC readings when integrated with hardware
 */
void boost_callback( int idx, int v, int up ) {
    current_boost_voltage = (float) v;
}

/////////////////////// [ TIMER MACHINES ] //////////////////////

/*
 * init_timer_machines: Initializes Automaton timer machines
 */

void init_timer_machines() {
    timer_1.begin( 0.5 )          // Set timer interval to 0.5 miliseconds
    .repeat( -1 )                 // Set timer to repeat indefinitively
    .onTimer( timer_1_callback )  // Attach timer trigger to callback
    .start();
    timer_2.begin( 1 )            // Set timer interval to to 1 milisecond
    .repeat( -1 )                 // Set timer to repeat indefinitively
    .onTimer( timer_2_callback )  // Attach timer trigger to callback
    .start();
    Serial.println("Timer Machines Initialized");
}

/*
 * timer_1_callback: Handles timer triggers. Continously check for the size of the
 *                   interrupt queues and adjust stepper position accordingly
 *                   -> Checks if the queues containing external interrupts elapsed 
 *                      time are empty or unequal. If so < return >
 *                   -> Checks if stepper motor is currently being adjusted if so < return >
 *                   -> Pops the oldest values of both queues (FIFO)
 *                   -> Compares times elapsed to adjust stepper motor on proper direction
 */
void timer_1_callback( int idx, int v, int up ) {

  if(queue_1.isEmpty() || queue_2.isEmpty()) { return; } // Check if empty
  if(queue_1.count() != queue_2.count()) { return; }     // Check equeal lengths 
  if(stepper1.distanceToGo() != 0) { return; }           // Checks if stepper is being used
 
  int val_1 = queue_1.dequeue();                         // Pop data out of queue
  int val_2 = queue_2.dequeue();                         // Pop data out of queue
  if(val_1 < val_2) {
    //Move stepper 15 degrees right
      turn_right();
  }
  else if(val_1 > val_2) {
    //Move stepper 15 degress left
      turn_left();
  }
  Serial.print("Val1:");
  Serial.println(val_1);
  Serial.print("Val2:");
  Serial.println(val_2);
}

/*
 * timer_2_callback: Handles timer tiggers. Calls MPPT algorithm when called.
 */
void timer_2_callback( int idx, int v, int up ) {
  track_max_power();
  return;
}

////////////////////////////////// [ STEPPER ] //////////////////////////

/*
 * init_stepper: Initializes stepper class
 */
void init_stepper() {
  // Set stepper max speed
  stepper1.setMaxSpeed(1000.0);
  // Set stepper accelaration (1000, is fastest)
  stepper1.setAcceleration(1000.0);
  // Set stepper speed (For some reason there is no much difference between values)
  // TODO: Investigate MaxSpeed, Acceleration and Speed for stepper motor used on final demo
  stepper1.setSpeed(1000);
}


/*
 * turn_right_by: Turns stepper right by 'N' steps
 * 
 * params: steps - Number of steps to move right
 */
void turn_right_by(int steps) {
  Serial.print("Move right by: ");
  Serial.println(steps);
  stepper1.moveTo(stepper1.currentPosition() + steps);
}

/*
 * turn_left_by: Turns stepper left by 'N' steps
 * 
 * params: steps - Number of steps to move left
 */
void turn_left_by(int steps) {
  Serial.print("Move left by: ");
  Serial.println(steps);
  stepper1.moveTo(stepper1.currentPosition() - steps);
}

/*
 * turn_right: Turns stepper right by default steps
 */
void turn_right() {
  Serial.print("Move right by: ");
  Serial.println(DEFAULT_STEPS);
  stepper1.moveTo(stepper1.currentPosition() + DEFAULT_STEPS);
}

/*
 * turn_left: Turns stepper left by default steps
 */
void turn_left() {
  Serial.print("Move left by: ");
  Serial.println(DEFAULT_STEPS);
  stepper1.moveTo(stepper1.currentPosition() - DEFAULT_STEPS);
}

////////////////////////////////// [ MPPT ] /////////////////////////////

/*
 * track_max_power: Entry point to MPPT Algorithm
 */

void track_max_power() {
  float voltage = current_voltage;             // Get current Voltage Reading
  float current = current_current;             // Get current Current Value
  float current_power = voltage*current;       // Compute current Power
  // Check if current power is greater than previously stored power
  if( current_power > stored_power ) {
    compare_voltages(voltage, true);           // Handle scenario in which current poer is greater than stored value
  }
  else {
    compare_voltages(voltage, false);          // Handle scenario in which current power is less than sotred value
  }
  store_power_voltage(voltage, current_power); // Store current power and voltage 
}

/*
 * compare_voltages : Handles difference between current voltage and stored voltage
 *                    -> Adjust Duty Cycle according to MPPT Algorithm
 */

void compare_voltages(float voltage, bool is_current_power_greater) {
  if(voltage > stored_voltage && is_current_power_greater) {
    //Increase Duty cycle
    increase_duty_cycle();
  }
  else if(voltage > stored_voltage && !is_current_power_greater) {
    // Decrease Duty cycle
    decrease_duty_cycle();
  }
  else if(voltage < stored_voltage && is_current_power_greater) {
    //Decrease Duty cycle
     decrease_duty_cycle();
  }
  else if(voltage < stored_voltage && !is_current_power_greater) {
    //Increase Duty cycle
    increase_duty_cycle();
  }
}

/*
 * store_power_voltage: Stores power and voltage in global variables
 */
void store_power_voltage(float voltage, float power) {
  stored_power   = power;
  stored_voltage = voltage;
}

/*
 * increase_duty_cycle: Increases duty cycle by 0.01 units of magnitude
 *                      -> Adds 0.01 units to Duty Cycle mutiplier
 *                      -> New Output Compare Register value is computed and written on ICR1 register
 *                         -> The new value of hardware register ICR1, will produce an increment of duty cycle
 */
void increase_duty_cycle() {
  duty = duty + 0.01;
  OCR1A = duty*100;
}

/*
 * increase_duty_cycle: Increases duty cycle by 0.01 units of magnitude
 *                      -> Adds 0.01 units to Duty Cycle mutiplier
 *                      -> New Output Compare Register value is computed and written on ICR1 register
 *                         -> The new value of hardware register ICR1, will produce an decrement of duty cycle
 */
void decrease_duty_cycle() {
  duty = duty - 0.01;
  OCR1A = duty*100;
}

///////////////////////// [ Log Handlers ] ////////////
void print_readings() {
  Serial.print("Voltage: ");
  Serial.print(current_voltage);
  Serial.print("    Current: ");
  Serial.print(current_current);
  Serial.print("    Boost Voltage: ");
  Serial.println(current_boost_voltage);
}
