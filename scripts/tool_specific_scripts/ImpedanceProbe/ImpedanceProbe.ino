#include <Wire.h>

// define macros
#define IA_ADDRESS              0x0D  //IA 
#define BLOCK_WRITE_CMD         0xA0  //block write command
#define BLOCK_READ_CMD          0xA1  //block read command
#define ADR_PTR_CMD             0xB0  //address pointer command

// define registers
#define CONTROL_REG             0x80
#define START_FREQUENCY_REG     0x82
#define FREQUENCY_INCREMENT_REG 0x85
#define NUM_INCREMENTS_REG      0x88
#define NUM_SETTLING_TIME_REG   0x8A
#define STATUS_REG              0x8F
#define TEMP_DATA_REG           0x92
#define REAL_DATA_REG           0x94
#define IMAG_DATA_REG           0x96

// define control register commands
#define INIT_START_FREQ         0x10
#define START_FREQ_SWEEP        0x20
#define INC_FREQ                0x30
#define REPEAT_FREQ             0x40
#define MEASURE_TEMP            0x90
#define PWR_DOWN                0xA0
#define STAND_BY                0xB0

// define frequency multiplier for start frequency calculation based on default 16 MHz clock speed
#define FREQUENCY_MULTIPLIER 33.5544

// define serial UART port
#define HWSERIAL Serial1

// initialize sweep parameters
struct SWEEP_PARAM
{
  bool calibration_Flag;
  int start_Freq;
  int freq_Step;
  int sweep_Samples;
  bool repeat_Flag;
}SweepParam;

// initialize pin variables
int ledPin = 13;
int sdaPin = 18;
int sclPin = 19;
int txPin = 1;
int rxPin = 0;
int selPin = 3;

// initialize blockWrite arrays
byte StartFreqData[3];
byte FrequencyIncData[3];
byte NumIncData[2];
byte SettlingTimeData[2];
byte ControlData[2];
byte realData[2];
byte imagData[2];

// initialize variables used for impedance analyzer data collection
int pga_gain = 1;
int voltage_range = 2000;
int16_t rVAL = 0;
int16_t iVAL = 0;

// SETUP AND LOOP FUNCTIONS

void setup() {
  delay(5000);
  Wire.setSDA(sdaPin);
  Wire.setSCL(sclPin);
  Wire.begin();

  Serial1.setTX(txPin);
  Serial1.setRX(rxPin);
  pinMode(rxPin, INPUT_PULLUP);
  HWSERIAL.begin(115200, SERIAL_8N1);
  
  pinMode(ledPin, OUTPUT);
  pinMode(selPin, OUTPUT);
  
  // Serial.begin(9600);
  
  SweepParam.calibration_Flag = false;
  SweepParam.start_Freq = 0;
  SweepParam.freq_Step = 0;
  SweepParam.sweep_Samples = 0;
  SweepParam.repeat_Flag = false;

//  while (!Serial1);
//  while (!Serial1 && millis() < 5000);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
  digitalWrite(ledPin, HIGH);

  String reception = "";
  // Serial.println("Booted up and waiting for command from UART.");
  HWSERIAL.clear();
  while(reception.length() < 22){
    if(HWSERIAL.available() > 0){
      char inChar = HWSERIAL.read();
      reception += inChar; 
      }
  }
  digitalWrite(ledPin, LOW);
  delay(100);
  digitalWrite(ledPin, HIGH);
  evaluateCommand(reception);

  int16_t* data_ptr = PerformFrequencySweep();

  for(int k = 0; k < (2*SweepParam.sweep_Samples); k++){
    //Serial.print((String)data_ptr[k] + ", ");
    //Serial.println(data_ptr[k], HEX);
    if(data_ptr[k] < 0){
      HWSERIAL.write(0x01);
    }
    else{
      HWSERIAL.write(0x00);
    }
    
    HWSERIAL.write(data_ptr[k] >> 8);
    HWSERIAL.write(data_ptr[k]);
  }

  // send some bits to indicate end of data
  HWSERIAL.write(0x00);
  HWSERIAL.write(0x01);
  HWSERIAL.write(0x02);
  HWSERIAL.write(0x03);

  delete[] data_ptr;
  delay(100);
}

// EVALUATING INCOMING MESSAGES OVER UART

void evaluateCommand(String cmd){
  // Serial.println(cmd);

  voltage_range = cmd.substring(1,5).toInt();
  SweepParam.calibration_Flag = cmd.substring(0,1).toInt();
  SweepParam.start_Freq = cmd.substring(5,11).toInt();
  SweepParam.freq_Step = cmd.substring(11,17).toInt();
  SweepParam.sweep_Samples = cmd.substring(17,20).toInt();
  SweepParam.repeat_Flag = cmd.substring(20,21).toInt();
  pga_gain = cmd.substring(21).toInt();

  // Serial.println((String)"Calibration flag: " + SweepParam.calibration_Flag);
  // Serial.println((String)"Start frequency: " + SweepParam.start_Freq);
  // Serial.println((String)"Frequency increment: " + SweepParam.freq_Step);
  // Serial.println((String)"Number of samples: " + SweepParam.sweep_Samples);
  // Serial.println((String)"Repeat flag: " + SweepParam.repeat_Flag);
  // Serial.println((String)"PGA Gain: " + pga_gain);
  // Serial.println((String)"Voltage range (mV): " + voltage_range);

  if(pga_gain == 5){
    // Serial.println("SEL connected to 3.3V");
    digitalWrite(selPin, HIGH);
  }
  else {
    // Serial.println("SEL connected to ground");
    digitalWrite(selPin, LOW);
  }
}

// BLOCK READING AND WRITING

void setRegisterPointer(uint8_t RegAddress)
{
  Wire.beginTransmission(IA_ADDRESS);
  Wire.send(ADR_PTR_CMD);  
  Wire.send(RegAddress);
  Wire.endTransmission();
}

void blockWrite(int numBytes, uint8_t *data) {
  // Serial.println("New command");
  
  Wire.beginTransmission(IA_ADDRESS);
  Wire.send(BLOCK_WRITE_CMD);
  Wire.send(numBytes);
  for(int i = 0; i < numBytes; i++)
  {
    // Serial.println(data[i], HEX); // display data being transmitted to slave device
    Wire.send(data[i]);
  }
  
  Wire.endTransmission();
}

void blockRead (int numBytes, uint8_t *buffer, uint8_t RegAddress)
{
  
  for(int i = 0; i<numBytes; i++)
  {
    setRegisterPointer(RegAddress);
    Wire.requestFrom(IA_ADDRESS, 1);
    while(Wire.available())
    {
      buffer[i] = Wire.receive();
    }
    Wire.endTransmission();
    RegAddress++;
  }
}


// ESTABLISHING SETTINGS AND COLLECTING DATA

void setSweepParam(bool cal_Flag, int begin_Freq, int f_Step, int nSamples, bool r_Flag){
  SweepParam.calibration_Flag = cal_Flag;
  SweepParam.start_Freq = begin_Freq;
  SweepParam.freq_Step = f_Step;
  SweepParam.sweep_Samples = nSamples;
  SweepParam.repeat_Flag = r_Flag;
}

void setStartFrequency(unsigned int freq){
  unsigned long result = freq * FREQUENCY_MULTIPLIER;

  // Serial.println(result, HEX);

  StartFreqData[2] = (uint8_t) (result & 0xFF);   
  StartFreqData[1] = (uint8_t) ((result >> 8) & 0xFF); 
  StartFreqData[0] = (uint8_t) ((result >> 16) & 0xFF);
  
  setRegisterPointer(START_FREQUENCY_REG);
  blockWrite(3, StartFreqData);
}

void setFrequencyIncrement(unsigned int deltaFreq)
{
  unsigned long result = deltaFreq * FREQUENCY_MULTIPLIER;
  
  FrequencyIncData[2] = (uint8_t) (result & 0xFF);
  FrequencyIncData[1] = (uint8_t) ((result>>8) & 0xFF);
  FrequencyIncData[0] = (uint8_t) ((result>>16) & 0xFF);
  
  setRegisterPointer(FREQUENCY_INCREMENT_REG);
  blockWrite(3, FrequencyIncData);
}

void setNumSamples(unsigned int samples)
{
  // NOTE: samples must be lower than 512
  NumIncData[1] = (uint8_t)(samples & 0xFF);
  NumIncData[0] = (uint8_t)((samples>>8) & 0xFF);
  
  setRegisterPointer(NUM_INCREMENTS_REG);
  blockWrite(2, NumIncData);
}

void setSettlingTime(unsigned int settlingTime, unsigned int settlingFactor)
{ 
  SettlingTimeData[1] = (uint8_t)(settlingTime & 0xFF);
  SettlingTimeData[0] = (uint8_t)((settlingTime >> 8)& 0xFF);    

  switch(settlingFactor)  {
    case 0:
      SettlingTimeData[0] |= 0x00; 
      break;
    case 1:
      SettlingTimeData[0] |= 0x2; //D10, D9 --> 01 
      break;
    case 3: 
      SettlingTimeData[0] |= 0x6; //D10, D9 --> 11
      break; 
  }
   
  setRegisterPointer(NUM_SETTLING_TIME_REG);
  blockWrite(2, SettlingTimeData);
}

void setControlRegister(unsigned int function, unsigned int gain)
{
  // Starts with the bits representing the desired PmodIA function
  ControlData[0] = 0x00;
  ControlData[0] |= function; 
  // Serial.println(ControlData[0], HEX);
  
  // Adds the voltage range bits (note values are in mV)
  switch(voltage_range){
    case 200:
      ControlData[0] |= 0x2; break;
    case 400:
      ControlData[0] |= 0x4; break;
    case 1000:
      ControlData[0] |= 0x6; break;
    case 2000:
      ControlData[0] |= 0x0; break;
    default:
      break;
  }

  // Adds the gain bits
  switch(gain)
  {
    case 1: // x1
      ControlData[0] |= 0x1; break;
    case 5: // x5
      ControlData[0] |= 0x0; break;
    default:
      break;
  }

  // Serial.println("Final Control Data");
  // Serial.println(ControlData[0], HEX);
  
  setRegisterPointer(CONTROL_REG);
  Wire.beginTransmission(IA_ADDRESS);
  Wire.send(CONTROL_REG);
  Wire.send(ControlData[0]);
  Wire.endTransmission();
}

int16_t* PerformFrequencySweep(){
  uint8_t BIT_1, BIT_2;
  int16_t* data_array = new int16_t[2 * SweepParam.sweep_Samples]; // 2x the number of samples because this has real and imaginary parts
  int index = 0;
  
  setStartFrequency(SweepParam.start_Freq);
  setFrequencyIncrement(SweepParam.freq_Step);
  setNumSamples(SweepParam.sweep_Samples); 
  setSettlingTime(10, 3);

  // Start data collection
  // 1. Place AD5933 into standby mode;
  setControlRegister(STAND_BY, pga_gain);
  delay(50);

  // 2. Initialize the Start Frequency Command to the Control Register
  setControlRegister(INIT_START_FREQ, pga_gain);
  delay(50);

  // 3. Program Start Frequency Sweep Command in the Control Register
  setControlRegister(START_FREQ_SWEEP, pga_gain);
  delay(50);

  for(int num = 0; num < SweepParam.sweep_Samples; num++){
    // 4. Poll Status register to check if the DFT conversion is complete 
    setRegisterPointer(STATUS_REG);

    do{
      Wire.requestFrom(IA_ADDRESS,1);
      BIT_1 = Wire.receive();
      BIT_1 = BIT_1 & B00000010;
    }while(BIT_1 != 0x02);

    // 5. Read values from real and imaginary registers and add to array
    blockRead(2, realData, REAL_DATA_REG);
    rVAL = realData[0];
    rVAL = rVAL << 8;
    rVAL |= realData[1]; 
    data_array[index] = rVAL; 
    index++;
  
    blockRead(2, imagData, IMAG_DATA_REG);
    iVAL = imagData[0];
    iVAL = iVAL << 8;
    iVAL |= imagData[1];
    data_array[index] = iVAL; 
    index++;
    
    // 6. Poll Status Register to check if frequency sweep is done
    do{
      Wire.requestFrom(IA_ADDRESS,1);
      BIT_2 = Wire.receive();
      BIT_2 = BIT_2 & B00000100;
    }while(BIT_2 != 0x04);

    if(SweepParam.sweep_Samples == 0 ){       
      break;
    }
    else if(SweepParam.repeat_Flag){ // Repeat Frequency Command if flagged 
      setControlRegister(REPEAT_FREQ, pga_gain);
      delay(10);
    }
    else{
      setControlRegister(INC_FREQ, pga_gain);
      delay(10);
    }
    
  }

  // 7. Program the AD5933 into Power Down Mode
  setControlRegister(PWR_DOWN, pga_gain);

  // Note that this data array will have every even index (i.e. 0, 2, 4, etc) as the real data
  // and every odd index (i.e. 1, 3, 5, etc) as the imaginary data
  return data_array;
}
