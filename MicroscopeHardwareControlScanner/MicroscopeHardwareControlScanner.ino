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
const int SINGLE_LASER_TRANSMITTER_PIN_1 = 23; //Digital output, 23 for Teensy
const int SINGLE_LASER_RECEIVER_PIN_1 = A8; //Analog input, 22/A8 for Teensy
const int SINGLE_LASER_TRANSMITTER_PIN_2 = 21; //Digital output, 21 for Teensy
const int SINGLE_LASER_RECEIVER_PIN_2 = A6; //Analog input, 20/A6 for Teensy

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
const int SINGLE_LASER_RECEIVER_THRESHOLD = 100; //Threshold analog value for detecting IR
const int IR_SAMPLES = 5; //Number of IR receiver samples to use for averaging
const unsigned int ANALOG_RESOLUTION = 9; //Number of bits to use for analog writes

//Currently preallocating arrays for odorant and state orders, make this number bigger/smaller depending on what the use cases are
const int MAX_STATE_COUNT = 32;
const int MAX_ODORANT_COUNT = 32;

//User-defined settings, updated via serial connection with microscope computer
const char START = '0';
const char SLICES_PER_VOLUME = '1';
const char VOLUMES_PER_SECOND = '2';
const char VOLUME_SCALE_MIN = '3';
const char VOLUME_SCALE_MAX = '4';
const char ODORANT_ORDER = '5';
const char STATE_ORDER = '6';
const char LASER_MODE = '7';
const char RESONANT_SCANNER_AMPLITUDE = '8';
const char CAMERA_EXPOSURE_TIME = '9';
const char LOCATE_EXPERIMENT_DEVICE = '?';
const char ABORT_EXPERIMENT = '!';

const int BOTH_LASERS = 0;
const int LASER_1_ONLY = 1;
const int LASER_2_ONLY = 2;

const int DEFAULT_SLICES_PER_VOLUME = 20;
const int DEFAULT_VOLUMES_PER_SECOND = 20;
const int DEFAULT_VOLUME_SCALE_MIN = 0;
const int DEFAULT_VOLUME_SCALE_MAX = 1;
const int DEFAULT_RESONANT_SCANNER_AMPLITUDE = 1;
const int DEFAULT_CAMERA_EXPOSURE_TIME = -1;
int slicesPerVolume;
int volumesPerSecond;
float volumeScaleMin;
float volumeScaleMax;
int laserMode;
float resonantScannerAmplitude;
int cameraExposureTime;

int odorantOrder[MAX_ODORANT_COUNT];

const unsigned int NULL_STATE = 0x00000000;
const unsigned int DEFAULT_STATE = 0x00010000;
const unsigned int ODORANT_STATE = 0x00020000;
const unsigned int RESET_STATE = 0x00030000;
const unsigned int STATE_MASK = 0xFFFF0000;
const unsigned int DURATION_MASK = 0x0000FFFF;
unsigned int stateOrder[MAX_STATE_COUNT]; //Store state as high 16 bits and duration as low 16 bits

//Volume-generator galvo parameters
int volumeGalvoStepCount;
int volumeGalvoStepTime;
int volumeGalvoStepAmplitude;
int volumeGalvoMinAmplitude;
int volumeGalvoMaxAmplitude;
volatile bool newVolume;

//Camera parameters
bool cameraOn;

//Control variables
bool resetState;
bool galvosEnabled;
bool cameraEnabled;
bool laserEnabled;

//Delta timing variables
unsigned long volumeGalvoDelta;
unsigned long cameraDelta;
//unsigned long stateDelta;
unsigned long stateRemainingImageCount;

unsigned long volumeGalvoStart;
unsigned long cameraStart;
//unsigned long stateStart;
unsigned long laserShutterStart;

unsigned long currentTime;

//Program control flow
const int SETUP = 0;
const int CALIBRATION = 1;
const int EXPERIMENT = 2;
const int RESET = 3;
int programState;

//Variables for writing to output pins
int volumeGalvoDirection;
int volumeGalvoPinValue;

int resonantScannerPinValue;

//Experiment control
unsigned int odorantIndex;
unsigned int stateIndex;
unsigned int currentState;
unsigned int previousState;

//Metadata control
unsigned int odorantCount;
unsigned int stateCount;
unsigned int currentOdorState = 0;

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
  pinMode(SINGLE_LASER_TRANSMITTER_PIN_1, OUTPUT);
  pinMode(SINGLE_LASER_RECEIVER_PIN_1, INPUT);
  pinMode(SINGLE_LASER_TRANSMITTER_PIN_2, OUTPUT);
  pinMode(SINGLE_LASER_RECEIVER_PIN_2, INPUT);
  pinMode(ODORANT_1_PIN, OUTPUT);
  pinMode(ODORANT_2_PIN, OUTPUT);
  pinMode(ODORANT_3_PIN, OUTPUT);
  pinMode(ODORANT_4_PIN, OUTPUT);
  pinMode(ODORANT_5_PIN, OUTPUT);
  pinMode(ODORANT_6_PIN, OUTPUT);
  pinMode(ODORANT_7_PIN, OUTPUT);
  pinMode(ODORANT_8_PIN, OUTPUT);

  analogWriteResolution(ANALOG_RESOLUTION);

  attachInterrupt(digitalPinToInterrupt(CHOPPER_OUTPUT_REFERENCE_PIN), startVolume, CHANGE);

  programState = SETUP;
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

  initializeVariables();

  //Clear incoming Serial buffer
  while(Serial.available())
  {
    Serial.read();
  }
  
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
          odorants = Serial.readStringUntil('\r');
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
        case CAMERA_EXPOSURE_TIME:
          cameraExposureTime = Serial.parseInt();
          Serial.print("Set camera exposure time to ");
          Serial.println(cameraExposureTime);
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
  volumeGalvoStepAmplitude = ((1<<ANALOG_RESOLUTION) * (volumeScaleMax-volumeScaleMin)) / volumeGalvoStepCount;
  volumeGalvoMinAmplitude = (1<<ANALOG_RESOLUTION) * volumeScaleMin;
  volumeGalvoMaxAmplitude = volumeGalvoMinAmplitude + volumeGalvoStepAmplitude * (volumeGalvoStepCount-1);

  resonantScannerAmplitude = min(max(0, resonantScannerAmplitude), 1);
  resonantScannerPinValue = int(resonantScannerAmplitude * (1<<ANALOG_RESOLUTION));

  if(cameraExposureTime < 0)
  {
    cameraExposureTime = volumeGalvoStepTime - VOLUME_GALVO_RESPONSE_TIME;
  }
  else
  {
    cameraExposureTime = min(cameraExposureTime, volumeGalvoStepTime - VOLUME_GALVO_RESPONSE_TIME);
  }
  
  volumeGalvoDelta = volumeGalvoStepTime;

  currentState = stateOrder[stateIndex] & STATE_MASK;
  stateRemainingImageCount = (stateOrder[stateIndex] & DURATION_MASK) * volumesPerSecond * slicesPerVolume;

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

  //Clear incoming Serial buffer
  while(Serial.available())
  {
    Serial.read();
  }
  
  while(programState == CALIBRATION)
  {
    currentTime = micros();
    updateHardware();
    checkSerial(); 
  }
  if(programState == EXPERIMENT)
  {
    Serial.println("Calibration complete. Starting experiment..."); 
  }
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
      case CAMERA_EXPOSURE_TIME:
        cameraExposureTime = Serial.parseInt();
        initializeParameters(false);
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

  if(currentState != RESET_STATE)
  {
    initializeHardware();
  }
  else
  {
    resetState = true;
  }
  
  currentTime = micros();
  volumeGalvoStart = currentTime;
  cameraStart = currentTime;
  cameraDelta = -1;
  //stateStart = currentTime;
  
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
        stateRemainingImageCount -= 1;
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
        if(resetState && galvosEnabled && digitalRead(CHOPPER_OUTPUT_REFERENCE_PIN) == HIGH)
        {
          resetHardware();
        }

        //Reenable hardware after reset state when starting new volume
        else if(!resetState && !galvosEnabled && digitalRead(CHOPPER_OUTPUT_REFERENCE_PIN) == LOW)
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
  if(stateRemainingImageCount == 0)
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
    stateRemainingImageCount = (stateOrder[stateIndex] & DURATION_MASK) * volumesPerSecond * slicesPerVolume;

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

    //stateStart = currentTime;
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
  int receiverValue1, receiverValue2;
  
  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN_1, HIGH);
  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN_2, HIGH);
  
  while(true)
  {
    receiverValue1 = 0;
    receiverValue2 = 0;
    for(int i = 0; i<IR_SAMPLES; i++)
    {
      receiverValue1 += analogRead(SINGLE_LASER_RECEIVER_PIN_1);
      receiverValue2 += analogRead(SINGLE_LASER_RECEIVER_PIN_2);
    }
    receiverValue1 /= IR_SAMPLES;
    receiverValue2 /= IR_SAMPLES;
     
    if((laserMode == LASER_1_ONLY && receiverValue1 >= SINGLE_LASER_RECEIVER_THRESHOLD && receiverValue2 >= SINGLE_LASER_RECEIVER_THRESHOLD) ||
      (laserMode == LASER_2_ONLY && receiverValue1 < SINGLE_LASER_RECEIVER_THRESHOLD && receiverValue2 < SINGLE_LASER_RECEIVER_THRESHOLD))
      {
        break;
      }

    analogWriteFrequency(CHOPPER_INPUT_REFERENCE_PIN, volumesPerSecond/2);
    analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 127);
    delay(CHOPPER_ALIGNMENT_DELAY); //Wait for chopper to start spinning
    analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 0); //Stop chopper
    delay(CHOPPER_ALIGNMENT_DELAY); //Wait for chopper to stop spinning
  }

  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN_1, LOW);
  digitalWrite(SINGLE_LASER_TRANSMITTER_PIN_2, LOW);
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
  analogWrite(VOLUME_GALVO_PIN, (1<<(ANALOG_RESOLUTION-1)));
  digitalWrite(CAMERA_TRIGGER_PIN, LOW);
  analogWrite(CHOPPER_INPUT_REFERENCE_PIN, 0);
  programState = SETUP;
}

void initializeVariables()
{
  slicesPerVolume = DEFAULT_SLICES_PER_VOLUME;
  volumesPerSecond = DEFAULT_VOLUMES_PER_SECOND;
  volumeScaleMin = DEFAULT_VOLUME_SCALE_MIN;
  volumeScaleMax = DEFAULT_VOLUME_SCALE_MAX;
  laserMode = BOTH_LASERS;
  resonantScannerAmplitude = DEFAULT_RESONANT_SCANNER_AMPLITUDE;
  cameraExposureTime = DEFAULT_CAMERA_EXPOSURE_TIME;

  for(int i=0; i < MAX_ODORANT_COUNT; i++)
  {
    odorantOrder[i] = NULL_ODORANT;
  }
  for(int i=0; i < MAX_STATE_COUNT; i++)
  {
    stateOrder[i] = NULL_STATE;
  }

  newVolume = true;
  cameraOn = false;
  resetState = false;
  galvosEnabled = true;
  cameraEnabled = true;
  laserEnabled = false;
  
  volumeGalvoDirection = 1;
  volumeGalvoPinValue = 0;
  resonantScannerPinValue = 0;
  
  odorantIndex = 0;
  stateIndex = 0;
  currentState = NULL_STATE;
  previousState = NULL_STATE;
}
