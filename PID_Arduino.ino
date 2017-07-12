// -------------------- Import Libraries --------------------
#include <Servo.h>
#include <inttypes.h>
#include <stdlib.h>
// -------------------- End Libraries --------------------


// -------------------- Running Median Class --------------------
template <typename T, uint8_t N, T default_value> class FastRunningMedian 
{
  public:
    // ---------- Constructor ----------
    FastRunningMedian() 
    {
      _buffer_ptr = N;
      _window_size = N;
      _median_ptr = N/2;
  
      // Init buffers
      uint8_t i = _window_size;
      while( i > 0 ) {
        i--;
        _inbuffer[i] = default_value;
        _sortbuffer[i] = default_value;
      }
    };
    // ---------- End Constructor ----------

    // ---------- Get Median ----------
    T getMedian() { return _sortbuffer[_median_ptr];}
    // ---------- End Get Median ----------

    // ---------- Add Value ----------
    void addValue(T new_value) 
    {
      if (_buffer_ptr == 0)
        _buffer_ptr = _window_size;
      
      _buffer_ptr--;
      
      T old_value = _inbuffer[_buffer_ptr]; // retrieve the old value to be replaced
      if (new_value == old_value)       // if the value is unchanged, do nothing
        return;
      
      _inbuffer[_buffer_ptr] = new_value;  // fill the new value in the cyclic buffer
        
      uint8_t i = _window_size;
      while(i > 0) 
      {
        i--;
        if (old_value == _sortbuffer[i])
          break;
      }
      
      _sortbuffer[i] = new_value; // replace the value 
  
      if (new_value > old_value) 
      {
        for(uint8_t p=i, q=i+1; q < _window_size; p++, q++) 
        {
          if (_sortbuffer[p] > _sortbuffer[q]) 
          {
            T tmp = _sortbuffer[p];
            _sortbuffer[p] = _sortbuffer[q];
            _sortbuffer[q] = tmp;
          } 
          else return;
        }
      } 
      else 
      {
        for(int p=i-1, q=i; q > 0; p--, q--) 
        {
          if (_sortbuffer[p] > _sortbuffer[q]) 
          {
            T tmp = _sortbuffer[p];
            _sortbuffer[p] = _sortbuffer[q];
            _sortbuffer[q] = tmp;
          } 
          else return;
        }
      }
    }
    // ---------- End Add Value ----------
  
  private:
    uint8_t _buffer_ptr;
    uint8_t _window_size;
    uint8_t _median_ptr;
    T _inbuffer[N];
    T _sortbuffer[N];
};
// -------------------- End Running Median Class --------------------



// -------------------- Objects --------------------
Servo servo1; //servo on y axis
Servo servo2; // lower right servo
Servo servo3; // lower left servo
FastRunningMedian<unsigned int,100, 0> xValMedian;
FastRunningMedian<unsigned int,100, 0> yValMedian;
// -------------------- End Objects --------------------



// -------------------- Constants --------------------
// -- Working Constants: --
#define outMin -400//-200
#define outMax 600//400
#define r3 1.73205 // square root of 3

// -- Touchscreen Pins: --
#define x_5v A1
#define x_gnd A2
#define y_5v A5
#define y_gnd A4

// -- Servo Pins: --
#define servopin1 9
#define servopin2 10
#define servopin3 11
// -------------------- End Constants --------------------



// -------------------- Variables --------------------
// -- Position Vars: --
double xO, yO; //setpoint
double xCor, yCor; //read from touchscreen
double xI, yI; //actual (median filtered)
double lastXI = 0, lastYI = 0;

// -- Error Vars: --
double error[3];//holds current errror
double lastError[3] = {0,0,0};
double lastLastError[3] = {0,0,0};

// ----- Output Vars: -----
double output[3]; //Hold actual output

// -- Computations Vars: --
double ITerm[3] = {0,0,0};
double lastDVal[3] = {0,0,0};
double lastLastDVal[3] = {0,0,0};
unsigned long now, deltaT, lastT;

// -- Tunings Vars: --
double setkp[3] = {0.3,0.5,0.5};
double setki[3] = {0.1,0.15,0.15};
double setkd[3] = {0.1,0.14,0.14};
double setka[3] = {1,1,1};
double aggkp[3] = {0.7,0.9,0.9};
double aggki[3] = {0.02,0.02,0.02};
double aggkd[3] = {0.1,0.1,0.1};
double aggka[3] = {0,0,0};
double kp[3], ki[3], kd[3], ka[3]; // working tunings
int compT; //computation time in ms

// -- Control Vars: --
bool aggTuningSet = false, compute_bool = true, 
  computeDone = true, setpointChange = false;
char controlByte = '1';
int servo1Rotate, servo2Rotate, servo3Rotate;
int auxTimeCount = 0, methodCount = 0;
int squareCoor[4][2] = {{1000,700}, {1000,300}, {500,300}, {500,700}};
int circleCoor[73][2] = {{950,500}, {949,517}, {947,535},
                        {943,552}, {938,568}, {931,584}, {923,600}, {914,615},
                        {903,629}, {891,641}, {879,653}, {865,664}, {850,673},
                        {835,681}, {819,688}, {802,693}, {785,697}, {768,699},
                        {750,700}, {733,699}, {715,697}, {698,693}, {666,681},
                        {650,673}, {635,664}, {622,653}, {609,642}, {597,629},
                        {586,615}, {577,600}, {569,585}, {562,569}, {557,552},
                        {553,535}, {551,518}, {550,500}, {551,483}, {553,466},
                        {557,449}, {562,432}, {569,416}, {577,400}, {586,386}, 
                        {597,372}, {608,359}, {621,347}, {635,336}, {650,327}, 
                        {665,319}, {681,312}, {698,307}, {715,303}, {732,301},
                        {750,300}, {767,301}, {784,303}, {801,307}, {818,312},
                        {834,319}, {850,327}, {864,336}, {878,346}, {891,358},
                        {903,371}, {913,385}, {923,399}, {931,415}, {938,431},
                        {943,448}, {947,465}, {949,482}, {950,499}, {949,517}};          
// -------------------- End Variables --------------------



// -------------------- Begin Setup --------------------
void setup()
{
  Serial.begin(115200); //Begin serial communications
  analogReadResolution(12); // Read touch screen using Due's 12bit ADC
  
  // ---------- Servos ----------
  // attach servo to pwn pins:
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  servo3.attach(servopin3);
  // level the servos:
  servo1.write(1500);
  servo2.write(1500);
  servo3.write(1500);
  // ---------- End Servos ----------
  
  // ---------- PID Init ----------
  //  These values can be initially set or set in real time
  xO =750;
  yO = 500;
  compT = 10;
  setPIDTunings(false);
  // ---------- End PID Init ----------
}
// -------------------- End Setup --------------------



// -------------------- Begin Main Loop --------------------
void loop()
{
  readTouchData();

  // if control byte calls for PID calculations, do so:
  if(compute_bool)
  {
    if(controlPID())
    {
      writeServos();
      computeDone = true;
    }
  }
  
  // This block runs once per compT, after PID is calculated:
  if(computeDone)
  {
    sendCoordinates();
    recieveSerial();

    switch(controlByte)
    {
      case('1'): // Setpoints make a square
        auxTimeCount++;
        if(auxTimeCount > 100)
        {
          corners(); // changes setpoint
          auxTimeCount = 0;
          setpointChange = true;
        }
        else setpointChange = false;
        break;
    
      case('2'): // Setpoints make a circle
        auxTimeCount++;
        if(auxTimeCount > 2)
        {
          circle(); //changes setpoint
          auxTimeCount = 0;
          setpointChange = true;
        }
        else setpointChange = false;
        break;
    
      case('5'):
        rotate();
        break;
    
      default:
        break;
    }    
    computeDone = false;
  }
}
// -------------------- End Main Loop --------------------



// -------------------- PID Methods --------------------
// ---------- Control PID ----------
// Main control function that calls and controls
// PID algorithm, returns true if PID was recalculated
bool controlPID()
{
  // check the elapsed time since last compute:
  now = millis(); // current time
  deltaT = now - lastT; // last compute time
  
  if(deltaT >= compT) // if compT time has elapsed
  { 
    for(int i=0; i<3; i++) // for each servo
    { 
      readTouchData(); // continue checking touchscreen
      getCoordinates(); // get coordinate from median filter
      readTouchData();
      getError(i); // finds error values
      readTouchData();
      computePID(i); // computes output from PID algorithm
      readTouchData();
      limitOutput(i); // limit the output
    }
    lastT = now; //keep track of last compute time    
    return true; //PID was calculated
  }
  else return false; //PID wasn't calculated
}
// ---------- End Control PID ----------

// ---------- Compute PID ----------
// PID algorithm that calculates the output
// for each servo
void computePID(int i)
{

  // to mitigate touchscreen errors, make sure current
  // error is within 10 of last error. If not, ignore it.
  if(controlByte == '0' || setpointChange)
    if(abs(error[i] - lastError[i]) > 10) error[i] = lastError[i];
  
  // calculate each PID term individually:
  double PTerm = error[i] * kp[i];
  ITerm[i] += (ki[i] * error[i]);
  double DVal = (error[i] - lastError[i]);
  double DTerm = kd[i] * (((5*DVal) + (3*lastDVal[i]) + (2*lastLastDVal[i]))/10.0);
  double ATerm = ka[i] * (((DVal - lastDVal[i]) + (lastDVal[i] - lastLastDVal[i]))/2.0);
      
  // Calculate Output:
  output[i] = PTerm + ITerm[i] + DTerm + ATerm;

  // save some calculations for later:
  lastError[i] = error[i];
  lastLastDVal[i] = lastDVal[i];
  lastDVal[i] = DVal;
}
// ---------- End Compute PID ----------

// ---------- Limit Output ----------
// Limits output from Compute PID to keep
// servo output within user defined bounds
void limitOutput(int i)
{
  if(output[i] > outMax) output[i] = outMax;
  else if(output[i] < outMin) output[i] = outMin;
  if(ITerm[i] > outMax) ITerm[i] = outMax;
  else if(ITerm[i] < outMin) ITerm[i] = outMin;
}
// ---------- End Limit Output ----------

// ---------- Get Error ----------
//  Computes the difference in the current
//  position and the perpendicular line of
//  action for each servo
void getError(int i)
{
  if (i == 0) // for servo1: (on y axis)
    error[0] = (yI - yO);

  if (i == 1) // for servo2 (120deg CW from servo1)
    error[1] = (r3*(xI - xO) + (yO-yI))/2.0;

  if (i == 2) // for servo3 (120deg CCW from servo1)
    error[2] = (r3*(xO - xI) + (yO-yI))/2.0;
}
// ---------- End Get Error ----------

// ---------- Set PID Parameters ----------
//  Because compute time is variable, this
//  method normalizes PID tunings against time
//  You can also select aggressive or standard tunings
//  by passing in a boolean variable (true = aggressive)
void setPIDTunings(bool aggressive)
{
  if(!aggressive)
    for(int i = 0; i < 3; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
  else
    for(int i = 0; i < 3; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
}
// ---------- End Set Parameters ----------

// ---------- Write Servos ----------
// This method sends the PID output to the Servos
void writeServos()
{
  servo1.write(1610 - output[0]);
  servo2.write(1610 + output[1]);
  servo3.write(1590 + output[2]);
}
// ---------- End Write Servos ----------
// -------------------- End PID Methods --------------------



// -------------------- Touchscreen Methods --------------------
// ---------- Read Touch Data ----------
//  Reads the position from the 4 wire touchscreen
//  and maps it to a 1500x1000 coordinate system
void readTouchData()
{
  //read x-position
  pinMode(x_5v,  OUTPUT);
  pinMode(x_gnd,  OUTPUT);
  pinMode(y_5v,  INPUT);
  pinMode(y_gnd,  INPUT);
  digitalWrite(x_5v, HIGH);
  digitalWrite(x_gnd, LOW);
  xCor = map(analogRead(y_5v), 400, 3600, 1500, 0 );
    
  //read y-position
  pinMode(x_5v,  INPUT);
  pinMode(x_gnd,  INPUT);
  pinMode(y_5v,  OUTPUT);
  pinMode(y_gnd,  OUTPUT);
  digitalWrite(y_5v, HIGH);
  digitalWrite(y_gnd, LOW);
  yCor = map(analogRead(x_5v), 700, 3300, 0, 1000);

  xValMedian.addValue(xCor);
  yValMedian.addValue(yCor);
}
// ---------- End Read Touch Data ----------

// ---------- Get Coordinates ----------
// gets coordinate value from median filter
void getCoordinates()
{
  xI = xValMedian.getMedian();
  yI = yValMedian.getMedian();
}
// ---------- End Get Coordinates ----------
// -------------------- End Touchscreen Methods --------------------



// -------------------- Auxilary Methods --------------------
// ---------- Send Coordinates ----------
// Sends errors, setpoint, and actual position
// to the python frontend to be displayed
void sendCoordinates()
{
  SerialUSB.print(error[0]);
  SerialUSB.print(",");
  SerialUSB.print(error[1]);
  SerialUSB.print(",");
  SerialUSB.print(error[2]);
  SerialUSB.print(",");
  SerialUSB.print(xO);
  SerialUSB.print(",");
  SerialUSB.print(yO);
  SerialUSB.print(",");
  SerialUSB.print(xI);
  SerialUSB.print(",");
  SerialUSB.println(yI);
}
// ---------- End Send Coordinates ----------

// ---------- Recieve Serial ----------
//  For the front end to communicate with
//  the machine, instructional bytes will
//  be sent and recieved with this message
bool recieveSerial()
{
  if(Serial.available() > 0)
  {
    int incomingByte = Serial.read();
    controlByte = (char)incomingByte;
    Serial.println(controlByte);
    switchOnce();
    return true;
  }
  else return false;
}
// ---------- End Recieve Serial ----------

// ---------- Tuning Switch ----------
//  If there was a new instruction recieved
//  from the front end, change actions
void switchOnce()
{
  int tempVar = 0;
  switch(controlByte)
  {
    case('P'): // change setkp
      tempVar = Serial.read();
      setkp[0] = (tempVar - 48)*0.1;
      setkp[1] = (tempVar - 48)*0.1;
      setkp[2] = (tempVar - 48)*0.1;
      controlByte = '0';
      xO = 750;
      yO = 500;
      setPIDTunings(true);
      compute_bool = true;
      break;
    
    case('I'): // change setki
      tempVar = Serial.read();
      setki[0] = (tempVar - 48)*0.01;
      setki[1] = (tempVar - 48)*0.01;
      setki[2] = (tempVar - 48)*0.01;
      controlByte = '0';
      xO = 750;
      yO = 500;
      setPIDTunings(true);
      compute_bool = true;
      break;
    
    case('D'): //change setkd
      tempVar = Serial.read();
      setkd[0] = (tempVar - 45)*0.01;
      setkd[1] = (tempVar - 43)*0.01;
      setkd[2] = (tempVar - 43)*0.01;
      controlByte = '0';
      xO = 750;
      yO = 500;
      setPIDTunings(true);
      compute_bool = true;
      break;
    
    case('0'): // centers ball
      xO = 750;
      yI = 500;   
    case('1'): // makes ball go in a square
    case('2'): // makes ball go in circles
      compute_bool = true;
      break;
    
    case('5'): // rotates platform
      compute_bool = false;
      servo1Rotate = 1500;
      servo2Rotate = 1900;
      servo3Rotate = 1100;
      break;
    
    case('3'): // raises the platform
      servo1.write(1100);
      servo2.write(1900);
      servo3.write(1900);
      compute_bool = false;
      break;
    
    case('4'): // lowers the platform
      servo1.write(1900);
      servo2.write(1100);
      servo3.write(1100);
      compute_bool = false;
      break;
    
    default:
      break;
  }
}
// ---------- End Tuning Switch ----------

// ---------- Corners ----------
// setpoint moves through the 4 corners
// on the screen, making the ball travel
// in a square
void corners()
{
  xO = squareCoor[methodCount][0];
  yO = squareCoor[methodCount][1];
  methodCount++;
  if(methodCount > 3) methodCount = 0;
}
// ---------- End Corners ----------

// ---------- Circle ----------
// setpoint moves in a circle around
// the center of the screen
void circle()
{
  xO = circleCoor[methodCount][0];
  yO = circleCoor[methodCount][1];
  methodCount++;
  if(methodCount > 72) methodCount = 0;
}
// ---------- End Circle ----------

// ---------- Rotate ----------
// Rotates the platform through its (mostly)
// full range of motion as a demo
void rotate()
{
  servo1Rotate--;
  if(servo1Rotate < 1100) servo1Rotate = 1900;

  servo2Rotate++;
  if(servo1Rotate > 1900) servo2Rotate = 1100;

  servo3Rotate++;
  if(servo3Rotate > 1900) servo3Rotate = 1100;

  servo1.write(servo1Rotate);
  servo2.write(servo2Rotate);
  servo3.write(servo3Rotate);
}
// ---------- End Rotate ----------
// -------------------- End Auxilary Methods --------------------



