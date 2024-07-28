/* Closed-loop differential drive:
 * Given translational and rotational velocities
 * in units of m/s and deg/s, respectively,
 * determine left and right motor commands.
 */

//======Advisor======
//===arbitration struct===
typedef struct layer LAYER; // C struct for subsumption task output

struct layer
{
  int cmd, // assertion command
    arg, // assertion argument
    flag; // subsumption flag (instead of layer state?)
};

// the robot has 2 subsumption behaviors, so define 2 LAYERS:
LAYER user,
  halt; // the default layer

const int job1Size = 2;

LAYER *job1[job1Size] = { &user, &halt };

LAYER *thisLayer = &halt; // output, layer chosen by arbitrator; global pointer to highest priority task asserting its flag, which is maintained by the "arbitration winner" signal

LAYER **job; // pointer to job priority list

int jobSize, // number of tasks in priority list
  arbitrate, // global flag to enable subsumption
  haltBot; // global flag to halt robot

volatile int userInvert,
  userEnable;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete


//======Encoders======
const byte esPins[] = 
{
  2, // encoder signal 1 pin
  19 // encoder signal 2 pin
};

const byte numEncoders = 2,
  pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

// values change in callback methods:
volatile int velPulseCounts[numEncoders],
  rotVels[numEncoders],
  prevRotVels[numEncoders],
  setVels[numEncoders];

//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 5, 6 };

//======Mobile Platform======
int wheelDiam = 64; // [mm]

//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
const int numMtrs = 2;

int reverse, // note: could be bool
  sensorsTmrCtr,
  maxOutVal,
  pubVelRate = 10;

double minLinearRes;

volatile double kp, ki, kd;

volatile int setTanVel, // 0-100%
  setRotVel, // 0-100%
  botVel; // global, current requested robot velocity

volatile int pubMtrCmds[numMtrs],
  signs[numMtrs],
  mtrOutAccums[numMtrs];
  
void setup() 
{
  initSystem();

  initBehaviors();
}

int initSystem()
{
  initNode("CruiseControl");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(115200);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(0, encoder1Callback, CHANGE);
  attachInterrupt(4, encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channel 1 */
  pinMode(mEnablePins[0], OUTPUT);
  pinMode(mSigPins[0], OUTPUT);

  /* Start Motor Channel 2 */
  pinMode(mEnablePins[1], OUTPUT);
  pinMode(mSigPins[1], OUTPUT);

  initSensorsTimer(); // ADD: publish rotational (and later translational) rotVel
}

void initSensorsTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = sensorsTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void initBehaviors()
{
  initVars();

  setParams();
  
  // set job1 as active job at startup time
  initJob1();
}

void initVars()
{
  setTanVel = 0; // [mm/s]

  setRotVel = 0; // [deg/s]

  pubMtrCmds[0] = 0;
  pubMtrCmds[1] = 0;

  signs[0] = 1;
  signs[1] = 1;

  mtrOutAccums[0] = 0;
  mtrOutAccums[1] = 0;

  rotVels[0] = 0;
  rotVels[1] = 0;

  prevRotVels[0] = 0;
  prevRotVels[1] = 0;
}

void setParams()
{
  setPIDGains(1.0, 0.0, 0.0);

  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  computeMinAngularRes();

  computeMinLinearRes();
  
  userInvert = 0;
  userEnable = 0;

  reverse = 0;

  arbitrate = 1;

  haltBot = 1;
}

void setPIDGains(double pg, double ig, double dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;
  
  kp = pg;

  ki = ig;

  kd = dg;

  Serial.print("Controller got kp:");
  Serial.print(kp, 3);
  Serial.print(", ki:");
  Serial.print(ki, 3);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();
}

void computeMinAngularRes()
{
  minAngularRes = 360.0 / pulsesPerRev;

  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
  Serial.println();
}

void computeMinLinearRes()
{
  float wheelCircumf;
  
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  wheelCircumf = piApprox * wheelDiam; // [mm]
  Serial.print("wheelCircumf = ");
  Serial.print(wheelCircumf);
  Serial.println(" mm");
  
  minLinearRes = wheelCircumf / pulsesPerRev; 
  Serial.print("Min. Lin. Res. (mm) ");
  Serial.println(minLinearRes);
  Serial.println();
}

int initJob1() // make job1 the active job
{
  job = &job1[0]; // global job priority list pointer

  jobSize = job1Size; // no. tasks in job1 list

  return 0;
}

// Currently treating loop like scheduler, but may be more like classifier
void loop() {}

//======Interrupt Service Routines======
void encoder1Callback()
{
  velPulseCounts[0]++;
  //determinePulseCount(1); // increment if forward, decrement if backward // OLD: pulseCount++;
}

void encoder2Callback()
{
  velPulseCounts[1]++;
  //determinePulseCount(2); // increment if forward, decrement if backward // OLD: pulseCount++;
}

/* The variable velPulseCount is read and zeroed by the
 * "ISR(TIMER1_OVF_vect)" routine, 
 * running 10 (change to maybe 20) times a second, which is
 * the sensor loop and update rate for this robot.
 * This routine copies the accumulated counts from
 * velPulseCount to the variable "rotVel" and
 * then resets velPulseCount to zero.
 */
void determinePulseCount(int eid)
{
  velPulseCounts[eid - 1]++;
  
//  if(signs[eid - 1] == 1)
//    velPulseCounts[eid - 1]++;
//  else
//    velPulseCounts[eid - 1]--;
}

/* Run Sensor Loop at x (maybe 10-20) Hz, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 */
ISR(TIMER1_OVF_vect) // sensors loop!
{
  TCNT1 = sensorsTmrCtr; // set timer

  userTask(); // accelerate forward and maintain given speed

  arbitrator(); // send highest priority to motors
}

/* Like common cruise behavior,
 * but allows user to request tan and rot vels,
 * rather than assuming top speed or 0.
 */
int userTask()
{
  extern LAYER user; // C structure for task output

  readUserInput(); // read: get requested tan and rot velocities. in this case, received by uio, which is a separate task(?)

  if(userEnable)
  {
    if(userInvert) // if inverted
    {
      user.cmd = 0; // request drive speed 0
      user.arg = 0; // request turn speed 0
    }
    else 
    {
      if(reverse) 
      {
        user.cmd = (int) round( -convertMMToPulses(setTanVel) / (double) pubVelRate ); // request -drive
        user.arg = (int) round( -convertDegToPulses(setRotVel) / (double) pubVelRate ); // request -turn
      }
      else
      {
        user.cmd = (int) round( convertMMToPulses(setTanVel) / (double) pubVelRate ); // request drive
        user.arg = (int) round( convertDegToPulses(setRotVel) / (double) pubVelRate ); // request turn
      }
    }
  
    Serial.print("cmd = ");
    Serial.print(user.cmd); // maybe change to user->cmd
    Serial.print(" pulses/");
    Serial.print(1.0 / pubVelRate);
    Serial.println("s");
  
    Serial.print("arg = ");
    Serial.print(user.arg); // maybe change to user->cmd
    Serial.print(" pulses/");
    Serial.print(1.0 / pubVelRate);
    Serial.println("s");
  
    user.flag = true; // (temp always) signal arbitrator we want control, unless disabled (would need to add cruiseEnable var)
  }
  else
    user.flag = false;
}

void readUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "g")
    {
      Serial.println("go");

      userInvert = 0;

      haltBot = 0;
    }
    else if(inputString.substring(0,1) == "s")
    {
      Serial.println("stop");

      userInvert = 1;

      haltBot = 1;
    }
    else if(inputString.substring(0,1) == "t") // given in m/s, convert to mm/s, so operations can be done with integer
    { 
      setTanVel = (int) round( inputString.substring(1, inputString.length()).toFloat() * 1000 ); // get string after 't'
    
      Serial.print("v_t = ");
      Serial.print(setTanVel / 1000.0);
      Serial.println(" m/s\n");

      userEnable = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,1) == "r") // given in deg/s
    {
      setRotVel = inputString.substring(1, inputString.length()).toInt(); // get string after 'r'
      
      Serial.print("v_r = ");
      Serial.print(setRotVel);
      Serial.println(" deg/s\n");

      userEnable = 1;

      haltBot = 0;
    }
    else if(inputString.substring(0,2) == "kp")
      kp = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kp'
    else if(inputString.substring(0,2) == "ki")
      ki = inputString.substring(2, inputString.length()).toFloat(); // get string after 'ki'
    else if(inputString.substring(0,2) == "kd")
      kd = inputString.substring(2, inputString.length()).toFloat(); // get string after 'kd'

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

int convertMMToPulses(int mm) 
{
  return (int) round( mm / minLinearRes );
}

int convertDegToPulses(int deg) 
{
  return (int) round( deg / minAngularRes );
}

/* "winnerId" feedback line 
 * from Arbitrate back to tasks:
 * Essentially global var containing ID of task 
 * that "won" this round of arbitration.
 * It can be used by the individual tasks
 * to determine if they have been subsumed.
 */
void arbitrator()
{
  int i = 0;

  if(arbitrate)
  {
    for(i=0; i < jobSize - 1; i++) // step through tasks
    {
      if(job[i]->flag) break; // subsume
    }

    thisLayer = job[i]; // global output winner
  }

  mtrCmd(thisLayer); // send command to motors; execute, given pointer to winning layer
}

void mtrCmd(LAYER *l)
{
  botVel = l->cmd; // [pulses/(1/pubVelRate)s], current requested velocity

  // ADD: convert mm to pulses and deg to pulses before computing setVels!
  // Compute control signals:
  setVels[0] = botVel + l->arg; // // left motor = velocity + rotation, (int) round( 1000 * setRotVel / ( minAngularRes * pubVelRate ) ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  setVels[0] = clip(setVels[0], 100, -100); // don't overflow +/- 100% full speed
  
  setVels[1] = botVel - l->arg; // right motor = velocity - rotation
  setVels[1] = clip(setVels[1], 100, -100); // don't overflow +/- 100% full speed

  Serial.print("Set Vels (pulses/(1/pubVelRate)s): ");
  for(int i=0; i < numMtrs; i++)
  {
    Serial.print(setVels[i]);
    Serial.print(" ");
  }
  Serial.println("\n");

  if(haltBot)
    stopMoving();
  else
    controlVel(); // PID
}

void stopMoving()
{
  for(int i=0; i < numMtrs; i++)
    digitalWrite(mEnablePins[i], LOW);
}

void controlVel()
{
  int aOutputs[numMtrs];

  // Read analog input (i.e. calc rot vel):
  speedometer();
  
  // Compute control signals:
  computeControlSignals(); 

  // Set analog outputs:
  for(int i=0; i < 2; i++)
    aOutputs[i] = (int) round( mtrOutAccums[i] / 256.0 );
  
  modulatePulseWidths(aOutputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
}

/* Read and zero velPulseCounts.
 * Copy and accumulate counts from velPulseCounts
 * to the rot. vel. variable and
 * then reset velPulseCounts to zero.
 */
void speedometer()
{
  for(int i=0; i < numMtrs; i++)
  {
    rotVels[i] = velPulseCounts[i] * signs[i]; // copy and accumulate counts from velPulseCount to rotVel
    velPulseCounts[i] = 0; // reset velPulseCount to zero
  }
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void computeControlSignals()
{
  int errs[numMtrs],
    P[numMtrs],
    I[numMtrs],
    D[numMtrs];
  
  int b = 1; // set point weight
  
  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);

  for(int i=0; i < numMtrs; i++)
  {
    Serial.print("setVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(setVels[i]);
    Serial.print(", rotVels[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(rotVels[i]);
  
    errs[i] = (int) round( ( b * setVels[i] - rotVels[i] ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    Serial.print(", errs[");
    Serial.print(i);
    Serial.print("] (pulses per 25.6 sec): ");
    Serial.println(errs[i]);
    
    P[i] = (int) round( errs[i] / kp ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))
    Serial.print("P[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(P[i]);

    D[i] = (int) round( ( ( rotVels[i] - prevRotVels[i] ) * 256 ) / kd ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
    Serial.print("D[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(D[i]);

    mtrOutAccums[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
    Serial.print("mtrOutAccums[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(mtrOutAccums[i]);

    prevRotVels[i] = rotVels[i]; // maintain history of previous measured rotVel

    mtrOutAccums[i] = clip(mtrOutAccums[i], maxOutVal, -maxOutVal); // accumulator
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < numMtrs; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }
  
  for(i=0; i < numMtrs; i++)
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    signs[mid] = -1;
  else if(signedVal >= 0)
    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");

//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  return map(magnitude, 0, 100, 54, 255); // cruise outputs
}
