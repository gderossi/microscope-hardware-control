const bool DEBUG_MODE = false; //Change this to true to see initialized parameters printed over serial for debugging

//Pin setup
//If using a different device than the Teensy 3.6, change pin numbers
//accordingly. Ensure that pin types match those listed in comments
const int VOLUME_GALVO_PIN = A21;      //DAC output, A21 for Teensy 
const int RESONANT_SCANNER_PIN = A22;  //DAC output, A22 for Teensy
const int CAMERA_TRIGGER_PIN = 26;    //Digital output, 26 for Teensy
const int CHOPPER_INPUT_REFERENCE_PIN = 29; //PWM output, 29 for Teensy
const int CHOPPER_OUTPUT_REFERENCE_PIN = 30; //Digital input, 30 for Teensy
const int SERIAL_INDICATOR_PIN = 13; //LED Pin, 13 for Teensy
const int LASER_SHUTTER_1_PIN =  27; //Digital output, 27 for Teensy
const int LASER_SHUTTER_2_PIN = 28; //Digital output, 28 for Teensy
const int SINGLE_LASER_TRANSMITTER_PIN = 23; //Digital output, 23 for Teensy
const int SINGLE_LASER_RECEIVER_PIN = 22; //Digital input, 22 for Teensy

const int ODORANT_1_PIN = 0;
const int ODORANT_2_PIN = 2;
const int ODORANT_3_PIN = 4;
const int ODORANT_4_PIN = 6;
const int ODORANT_5_PIN = 8;
const int ODORANT_6_PIN = 10;
const int ODORANT_7_PIN = 12;
const int ODORANT_8_PIN = 24;
const int NULL_ODORANT = -1; //Not a pin, but used when initializing odorant order

//Constants
//This is the maximum time it takes for the volume galvo to settle 
const int VOLUME_GALVO_RESPONSE_TIME = 250; //Microseconds
//This is the maximum time it takes for the laser shutters to open/close
const int LASER_SHUTTER_RESPONSE_TIME = 5000; //Microseconds
const int BAUDRATE = 115200;   //Ignored by Teensy
const int DEFAULT_TIME_UNIT = 1000000; //Microseconds per second
const int CHOPPER_ALIGNMENT_DELAY = 5000; //Microseconds to wait for chopper start/stop

//Currently preallocating arrays for odorant and state orders, make this number bigger/smaller depending on what the use cases are
const int MAX_STATE_COUNT = 32;
const int MAX_ODORANT_COUNT = 32;

//User-defined settings, updated via serial connection with microscope computer
#define START '0'
#define SLICES_PER_VOLUME '1'
#define VOLUMES_PER_SECOND '2'
#define VOLUME_SCALE_MIN '3'
#define VOLUME_SCALE_MAX '4'
#define ODORANT_ORDER '5' 
#define STATE_ORDER '6'
#define LASER_MODE '7'
#define RESONANT_SCANNER_AMPLITUDE '8'
#define LOCATE_EXPERIMENT_DEVICE '?'
#define ABORT_EXPERIMENT '!'

#define BOTH_LASERS 0
#define LASER_1_ONLY 1
#define LASER_2_ONLY 2

int slicesPerVolume = -1;
int volumesPerSecond = -1;
float volumeScaleMin = -1;
float volumeScaleMax = -1;
int laserMode = -1;
float resonantScannerAmplitude = -1;
unsigned int analogResolution = 9;
int odorantOrder[MAX_ODORANT_COUNT];

#define NULL_STATE 0x00000000
#define DEFAULT_STATE 0x00010000
#define ODORANT_STATE 0x00020000
#define RESET_STATE 0x00030000
#define STATE_MASK 0xFFFF0000
#define DURATION_MASK 0x0000FFFF
unsigned int stateOrder[MAX_STATE_COUNT]; //Store state as high 16 bits and duration as low 16 bits

//Volume-generator galvo parameters
int volumeGalvoStepCount;
int volumeGalvoStepTime;
int volumeGalvoStepAmplitude;
int volumeGalvoMinAmplitude;
int volumeGalvoMaxAmplitude;
volatile bool newVolume = true;

//Camera parameters
int cameraExposureTime;
bool cameraOn = false;

//Control variables
bool resetState = false;
bool galvosEnabled = true;
bool cameraEnabled = true;
bool laserEnabled = false;

//Delta timing variables
unsigned long volumeGalvoDelta;
unsigned long cameraDelta = -1;
unsigned long stateDelta;

unsigned long volumeGalvoStart;
unsigned long cameraStart;
unsigned long stateStart;
unsigned long laserShutterStart;

unsigned long currentTime;

//Program control flow
#define SETUP 0
#define CALIBRATION 1
#define EXPERIMENT 2
#define RESET 3
int programState = SETUP;

//Variables for writing to output pins
int volumeGalvoDirection = 1;
int volumeGalvoPinValue = 0;

int resonantScannerPinValue = 0;

//Experiment control
unsigned int odorantIndex = 0;
unsigned int stateIndex = 0;
unsigned int currentState = NULL_STATE;
unsigned int previousState = NULL_STATE;

void setup() {
  Serial.begin(BAUDRATE);

  pinMode(VOLUME_GALVO_PIN, OUTPUT);
  pinMode(RESONANT_SCANNER_PIN, OUTPUT);
  pinMode(CAMERA_TRIGGER_PIN, OUTPUT);
  pinMode(CHOPPER_INPUT_REFERENCE_PIN, OUTPUT);
  pinMode(CHOPPER_OUTPUT_REFERENCE_PIN, INPUT);
  pinMode(SERIAL_INDICATOR_PIN, OUTPUT);
  pinMode(LASER_SHUTTER_1_PIN, OUTPUT);
  pinMode(LASER_SHUTTER_2_PIN, OUTPUT);
  pinMode(SINGLE_LASER_TRANSMITTER_PIN, OUTPUT);
  pinMode(SINGLE_LASER_RECEIVER_PIN, INPUT);
  pinMode(ODORANT_1_PIN, OUTPUT);
  pinMode(ODORANT_2_PIN, OUTPUT);
  pinMode(ODORANT_3_PIN, OUTPUT);
  pinMode(ODORANT_4_PIN, OUTPUT);
  pinMode(ODORANT_5_PIN, OUTPUT);
  pinMode(ODORANT_6_PIN, OUTPUT);
  pinMode(ODORANT_7_PIN, OUTPUT);
  pinMode(ODORANT_8_PIN, OUTPUT);

  analogWriteResolution(analogResolution);

  attachInterrupt(digitalPinToInterrupt(CHOPPER_OUTPUT_REFERENCE_PIN), startVolume, CHANGE);
}

void loop() {
  switch(programState)
  {
    case SETUP:
      setupLoop();
      break;
    case CALIBRATION:
      calibrationLoop();
      break;
    case EXPERIMENT:
      experimentLoop();
      break;
    case RESET:
      shutdownHardware();
      break;
  }
}

//Get user-defined parameters via serial connection
void setupLoop()
{
  char parameterSelector;
  String odorants;
  unsigned int stateCount;
  int o;

  odorantIndex = 0;
  stateIndex = 0;
  currentState = NULL_STATE;
  previousState = NULL_STATE;
  
  while(programState == SETUP)
  {
    if(Serial.available())
    {
      //Parse serial input and send confirmation back
      parameterSelector = Serial.read();
      if(parameterSelector == '\n' || parameterSelector == '\r')
      {
        continue;
      }
      
      switch(parameterSelector)
      {
        case SLICES_PER_VOLUME:
          slicesPerVolume = Serial.parseInt();
          Serial.print("Set 'Slices per volume' to ");
          Serial.println(slicesPerVolume);
          break;
        case VOLUMES_PER_SECOND:
          volumesPerSecond = Serial.parseInt();
          Serial.print("Set 'Volumes per second' to ");
          Serial.println(volumesPerSecond);
          break;
        case VOLUME_SCALE_MIN:
          volumeScaleMin = Serial.parseFloat();
          Serial.print("Set 'Volume scale min' to ");
          Serial.println(volumeScaleMin);
          break;
        case VOLUME_SCALE_MAX:
          volumeScaleMax = Serial.parseFloat();
          Serial.print("Set 'Volume scale max to ");
          Serial.println(volumeScaleMax);
          break;
        case ODORANT_ORDER:
          odorants = Serial.readStringUntil('\n');
          Serial.print("Odorant order: ");
          
          //Fill odorantOrder with NULL_ODORANT values
          for(int i=0; i < MAX_ODORANT_COUNT; i++)
          {
            odorantOrder[i] = NULL_ODORANT;
          }
          //Output index
          o = -1;
          for(unsigned int i=0; i < odorants.length(); i++)
          {
            o++;
            Serial.print(odorants[i]);
            Serial.print(", ");
            switch(odorants[i])
            {
              case '1':
                odorantOrder[o] = ODORANT_1_PIN;
                break;
              case '2':
                odorantOrder[o] = ODORANT_2_PIN;
                break;
              case '3':
                odorantOrder[o] = ODORANT_3_PIN;
                break;
              case '4':
                odorantOrder[o] = ODORANT_4_PIN;
                break;
              case '5':
                odorantOrder[o] = ODORANT_5_PIN;
                break;
              case '6':
                odorantOrder[o] = ODORANT_6_PIN;
                break;
              case '7':
                odorantOrder[o] = ODORANT_7_PIN;
                break;
              case '8':
                odorantOrder[o] = ODORANT_8_PIN;
                break;
              default:
                o = max(-1, o-1);
                break;
            }
          }
          Serial.println("");
          break;
        case STATE_ORDER:
          stateCount = Serial.parseInt();
          Serial.print("State order: ");

          //Fill stateOrder with NULL_STATE values
          for(int i=0; i < MAX_STATE_COUNT; i++)
          {
            stateOrder[i] = NULL_STATE;
          }
  
          for(unsigned int i=0; i < stateCount; i++)
          {
            stateOrder[i] = Serial.parseInt();
            Serial.print((stateOrder[i] & STATE_MASK) >> 16);
            Serial.print(", ");
            Serial.print(stateOrder[i] & DURATION_MASK);
            Serial.print("s; ");
          }
          Serial.println("");
          break;
        case LASER_MODE:
          laserMode = Serial.parseInt();
          Serial.print("Laser mode: ");
          if(laserMode == BOTH_LASERS)
          {
            Serial.println("Both Lasers");
          }
          else if(laserMode == LASER_1_ONLY)
          {
            Serial.println("Laser 1 Only");
          }
          else
          {
            Serial.println("Laser 2 Only");
          }
          break;
        case RESONANT_SCANNER_AMPLITUDE:
          resonantScannerAmplitude = Serial.parseFloat();
          Serial.print("Set resonant scanner amplitude to ");
          Serial.println(resonantScannerAmplitude);
          break;
        case LOCATE_EXPERIMENT_DEVICE:
          Serial.println("EXPERIMENT_DEVICE");
          break;
        case START:
          programState = CALIBRATION;
          break;
        default:
          Serial.print("Error: Parameter ");
          Serial.print(parameterSelector);
          Serial.println(" is invalid");
      }
    }
  }
  initializeParameters(DEBUG_MODE);
  Serial.println("Initial setup complete. Initializing calibration mode...");
}

//Set values for parameters
void initializeParameters(bool debug)
{
  volumeGalvoStepCount = slicesPerVolume;
  volumeGalvoStepTime = DEFAULT_TIME_UNIT / (volumesPerSecond * slicesPerVolume);            
  volumeGalvoStepAmplitude = ((1<<analogResolution) * (volumeScaleMax-volumeScaleMin)) / volumeGalvoStepCount;
  volumeGalvoMinAmplitude = (1<<analogResolution) * volumeScaleMin;
  volumeGalvoMaxAmplitude = volumeGalvoMinAmplitude + volumeGalvoStepAmplitude * (volumeGalvoStepCount-1);

  resonantScannerAmplitude = min(max(0, resonantScannerAmplitude), 1);
  resonantScannerPinValue = int(resonantScannerAmplitude * (1<<analogResolution));

  cameraExposureTime = volumeGalvoStepTime - VOLUME_GALVO_RESPONSE_TIME;

  volumeGalvoDelta = volumeGalvoStepTime;

  currentState = stateOrder[stateIndex] & STATE_MASK;
  stateDelta = (stateOrder[stateIndex] & DURATION_MASK) * DEFAULT_TIME_UNIT;

  if(laserMode == BOTH_LASERS)
  {
    analogWriteFrequency(CHOPPER_INPUT_REFERENCE_PIN, volumesPerSecond/2);
    analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 127);
  }
  else
  {
    singleLaserChopperAlignment();
  }
  
  if(debug)
  {
    debugParameters();
  }
}

//Print initial parameter values over serial connection
void debugParameters()
{
  Serial.print("volumeGalvoStepCount: ");
  Serial.println(volumeGalvoStepCount);
  Serial.print("volumeGalvoStepTime: ");
  Serial.println(volumeGalvoStepTime);
  Serial.print("volumeGalvoStepAmplitude: ");
  Serial.println(volumeGalvoStepAmplitude);
  Serial.print("volumeGalvoMinAmplitude: ");
  Serial.println(volumeGalvoMinAmplitude);
  Serial.print("volumeGalvoMaxAmplitude: ");
  Serial.println(volumeGalvoMaxAmplitude);
  Serial.print("resonantScannerAmplitude: ");
  Serial.println(resonantScannerAmplitude);
  Serial.print("cameraExposureTime: ");
  Serial.println(cameraExposureTime);
  Serial.print("cameraDelta: ");
  Serial.println(cameraDelta);
}

//A modified version of the main experiment loop which doesn't have states or odorants
//Constantly checks serial data for parameter updates and applies them live
void calibrationLoop()
{
  initializeHardware();
  currentTime = micros();
  volumeGalvoStart = currentTime;
  cameraStart = currentTime;
  
  while(programState == CALIBRATION)
  {
    currentTime = micros();
    updateHardware();
    checkSerial(); 
  }
  Serial.println("Calibration complete. Starting experiment...");
}

//Checks for and updates a subset of parameters as part of the calibration loop
void checkSerial()
{
  if(Serial.available())
  {
    char parameterSelector = Serial.read();
    
    if(parameterSelector == '\n' || parameterSelector == '\r')
    {
      return;
    }
    
    switch(parameterSelector)
    {
      case START:
        programState = EXPERIMENT;
        break;
      case SLICES_PER_VOLUME:
        slicesPerVolume = Serial.parseInt();
        initializeParameters(false);
        break;
      case VOLUMES_PER_SECOND:
        volumesPerSecond = Serial.parseInt();
        initializeParameters(false);
        break;
      case VOLUME_SCALE_MIN:
        volumeScaleMin = Serial.parseFloat();
        initializeParameters(false);
        break;
      case VOLUME_SCALE_MAX:
        volumeScaleMax = Serial.parseFloat();
        initializeParameters(false);
        break;
      case RESONANT_SCANNER_AMPLITUDE:
        resonantScannerAmplitude = Serial.parseFloat();
        initializeParameters(false);
        break;
      case ABORT_EXPERIMENT:
        programState = RESET;
        break;
      default:
        Serial.print("Error: parameter ");
        Serial.print(parameterSelector);
        Serial.println(" was not recognized");
    }
  }
}

//Runs the main loop and updates states and hardware
void experimentLoop()
{
  //Clear incoming Serial buffer
  while(Serial.available())
  {
    Serial.read();
  }
  
  initializeHardware();
  currentTime = micros();
  volumeGalvoStart = currentTime;
  cameraStart = currentTime;
  cameraDelta = -1;
  stateStart = currentTime;
  
  while(programState == EXPERIMENT)
  {
    currentTime = micros();
    updateStates();
    updateHardware();
    if(Serial.available())
    {
      programState = RESET;
    }
  }
}

void updateHardware()
{
  if(!newVolume || laserMode != BOTH_LASERS)
  {
    //Delta timing for camera
    if(currentTime - cameraStart >= cameraDelta)
    {
      if(cameraOn)
      {
        cameraOn = false;
        if(cameraEnabled)
        {
          digitalWrite(CAMERA_TRIGGER_PIN, LOW);  //Stop camera exposure
        }
        cameraDelta = -1;
      }
      else
      {
        cameraOn = true;
        if(cameraEnabled)
        {
          digitalWrite(CAMERA_TRIGGER_PIN, HIGH); //Start camera exposure
        }
        cameraDelta = cameraExposureTime;
      }

      cameraStart = currentTime;
    }
    
    //Delta timing for volume galvo
    if(currentTime - volumeGalvoStart >= volumeGalvoDelta)
    {
      //Update volume galvo position
      volumeGalvoPinValue += volumeGalvoDirection * volumeGalvoStepAmplitude;
      if(galvosEnabled)
      {
        analogWrite(VOLUME_GALVO_PIN, volumeGalvoPinValue);
      }

      //Update camera delta
      cameraDelta = VOLUME_GALVO_RESPONSE_TIME;
      cameraStart = currentTime;

      //Check if volume galvo is at the end of its range
      if(volumeGalvoPinValue <= volumeGalvoMinAmplitude || volumeGalvoPinValue >= volumeGalvoMaxAmplitude)
      {
        if(volumeGalvoPinValue <= volumeGalvoMinAmplitude)
        {
          volumeGalvoPinValue = volumeGalvoMinAmplitude - volumeGalvoStepAmplitude + 1;
        }
        else
        {
          volumeGalvoPinValue = volumeGalvoMaxAmplitude + volumeGalvoStepAmplitude - 1;
        }
        newVolume = true;
        volumeGalvoDirection *= -1;
        
        //Disable hardware after finishing volume
        if(resetState && galvosEnabled)
        {
          resetHardware();
        }

        //Reenable hardware after reset state when starting new volume
        else if(!resetState && !galvosEnabled)
        {
          galvosEnabled = true;
          openLaserShutters();
        }
        else if(!resetState && !cameraEnabled)
        {
          if(laserEnabled && currentTime - laserShutterStart >= LASER_SHUTTER_RESPONSE_TIME)
          {
            cameraEnabled = true;
          }
        }
      }
  
      //Reset delta clock
      volumeGalvoStart = currentTime;
    }
  }
}

void updateStates()
{
  //Delta timing for states
  if(currentTime - stateStart >= stateDelta)
  {
    //Update stateIndex and move to start of array if no valid states remaining
    stateIndex = (stateIndex + 1) % MAX_STATE_COUNT;
    if((stateOrder[stateIndex] & STATE_MASK) == NULL_STATE)
    {
      stateIndex = 0;
    }
    //Stop program if all odorants have been used once state loop resets
    if(stateIndex == 0 && (odorantIndex == MAX_ODORANT_COUNT || odorantOrder[odorantIndex] == NULL_ODORANT))
    {
      shutdownHardware();
    }

    //Update state and duration
    previousState = currentState;
    currentState = stateOrder[stateIndex] & STATE_MASK;
    stateDelta = (stateOrder[stateIndex] & DURATION_MASK) * DEFAULT_TIME_UNIT;

    //Turn off odorant if odorant state just concluded, and update odorantIndex
    if(previousState == ODORANT_STATE)
    {
      digitalWrite(odorantOrder[odorantIndex], LOW);
      odorantIndex = (odorantIndex + 1);
    }

    //If current state is an odorant state, turn on odorant
    if(currentState == ODORANT_STATE)
    {
      digitalWrite(odorantOrder[odorantIndex], HIGH);
    }

    if(previousState == RESET_STATE)
    {
      resetState = false;
    }

    if(currentState == RESET_STATE)
    {
      resetState = true;
    }

    stateStart = currentTime;
  }
}

//Triggered by interrupt from chopper, used to sync chopper with rest of hardware
void startVolume()
{
  newVolume = false;
}

void openLaserShutters()
{
  if(laserMode == BOTH_LASERS)
  {
    digitalWrite(LASER_SHUTTER_1_PIN, HIGH);
    digitalWrite(LASER_SHUTTER_2_PIN, HIGH);
  }
  else if(laserMode == LASER_1_ONLY)
  {
    digitalWrite(LASER_SHUTTER_1_PIN, HIGH);
  }
  else
  {
    digitalWrite(LASER_SHUTTER_2_PIN, HIGH);
  }

  laserEnabled = true;
  laserShutterStart = micros();
}

void closeLaserShutters()
{
  digitalWrite(LASER_SHUTTER_1_PIN, LOW);
  digitalWrite(LASER_SHUTTER_2_PIN, LOW);

  laserEnabled = false;
}

//Uses an IR LED/receiver setup to align the chopper for a given laser mode
void singleLaserChopperAlignment()
{
  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN, HIGH);
  
  while(true)
  {
    if((laserMode == LASER_1_ONLY && digitalRead(SINGLE_LASER_RECEIVER_PIN) == HIGH) ||
      (laserMode == LASER_2_ONLY && digitalRead(SINGLE_LASER_RECEIVER_PIN) == LOW))
      {
        break;
      }

    analogWriteFrequency(CHOPPER_INPUT_REFERENCE_PIN, volumesPerSecond/2);
    analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 127);
    delay(CHOPPER_ALIGNMENT_DELAY); //Wait for chopper to start spinning
    analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 0); //Stop chopper
    delay(CHOPPER_ALIGNMENT_DELAY); //Wait for chopper to stop spinning
  }

  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN, LOW);
}

void initializeHardware()
{
  analogWrite(RESONANT_SCANNER_PIN, resonantScannerPinValue);
  openLaserShutters();
}

void resetHardware()
{
  closeLaserShutters();
  analogWrite(RESONANT_SCANNER_PIN, 0);
  analogWrite(VOLUME_GALVO_PIN, volumeGalvoMinAmplitude);
  digitalWrite(CAMERA_TRIGGER_PIN, LOW);
  galvosEnabled = false;
  cameraEnabled = false;
  cameraOn = false;
}

void shutdownHardware()
{
  Serial.println("Shutting down hardware");
  closeLaserShutters();
  analogWrite(RESONANT_SCANNER_PIN, 0);
  analogWrite(VOLUME_GALVO_PIN, (1<<(analogResolution-1)));
  digitalWrite(CAMERA_TRIGGER_PIN, LOW);
  analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 0);
  programState = SETUP;
}
