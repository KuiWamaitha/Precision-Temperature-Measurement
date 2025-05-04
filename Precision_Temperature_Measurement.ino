#include <Arduino.h>
#include <SPI.h>


     //Modular section 1: SPI initialization and communication
//SPI initialization and communication
#define ADC_CS_PIN      10    //chip select pin on MCP3008
#define ADC_CHANNEL_PT100 0   
#define ADC_CHANNEL_PT1000 1  

//Initialize SPI for communication with MCP3008 ADC
void initSPI() {
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH); //deselect ADC initially
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz clock
}

//Read ADC channel (0-7) via SPI
uint16_t readADC(uint8_t channel) {
    channel &= 0x07;//validate channel number

  uint8_t command = 0x18 | channel; //combine start bit, single-ended mode, and channel number
  
  //Pull CS low to select ADC
  digitalWrite(ADC_CS_PIN, LOW);
  
  //Send command and read response
  uint8_t highByte = SPI.transfer(command);
  uint8_t lowByte = SPI.transfer(0x00); //dummy byte to clock out the result
  
  //Pull CS high to deselect ADC
  digitalWrite(ADC_CS_PIN, HIGH);//Pull CS high to deselect ADC
  
  
  return ((highByte & 0x03) << 8) | lowByte;//combine the two bytes and return 10-bit result
}


     //Modular section 2: RTD measurement and data acquisition
//Read raw ADC value for PT100
uint16_t readPT100() {
  return readADC(ADC_CHANNEL_PT100);
}

//Read raw ADC value for PT1000
uint16_t readPT1000() {
  return readADC(ADC_CHANNEL_PT1000);
}



      //Modular Section 3: Temperature Conversion
//Constants for Callendar-Van Dusen equation
const float RTD_A = 3.9083e-3;
const float RTD_B = -5.775e-7;
const float RTD_C = -4.183e-12;  // For temperatures below 0°C

//Convert ADC reading to temperature for a given RTD type
float adcToTemperature(uint16_t adcValue, bool isPT1000) {
  const float VREF = 3.3f;          // ADC reference voltage
  const float I_SOURCE_PT100 = 0.4e-3f;   // 0.4mA current source for PT100
  const float I_SOURCE_PT1000 = 0.2e-3f;  // 0.2mA current source for PT1000
  const float AMP_GAIN = 100.0f;    // Instrumentation amplifier gain
  
  //Calculate RTD resistance from ADC value
  float voltage = (adcValue / 1023.0f) * VREF;
  float rtdVoltage = voltage / AMP_GAIN;
  float rtdResistance;
  
  if (isPT1000) {
    rtdResistance = rtdVoltage / I_SOURCE_PT1000;
  } else {
    rtdResistance = rtdVoltage / I_SOURCE_PT100;
  }
  
  // Normalize resistance to R0 (100 for PT100, 1000 for PT1000)
  float R0 = isPT1000 ? 1000.0f : 100.0f;
  float Z = rtdResistance / R0;
  
  // Callendar-Van Dusen implementation
  float temperature;
  
  if (Z >= 1.0f) {
    // For positive temperatures (simpler quadratic)
    temperature = (-RTD_A + sqrtf(RTD_A * RTD_A - 4 * RTD_B * (1 - Z))) / (2 * RTD_B);
  } else {
    // For negative temperatures (full equation)
    temperature = (sqrtf(pow(RTD_A, 2) - 4 * RTD_B * (1 - Z)) - RTD_A) / (2 * RTD_B);
    
    // Very low temperatures may need more precise calculation
    if (temperature < -100.0f) {
      // Iterative solution would be better here for maximum accuracy
      temperature = (RTD_A + 2 * RTD_B * temperature) * temperature + (1 - Z);
      temperature = (sqrtf(pow(RTD_A, 2) - 4 * RTD_B * (1 - Z)) - RTD_A) / (2 * RTD_B);
    }
  }
  
  return temperature;
}


    //Section 4: RTD auto-detection
enum RTDType {
  RTD_UNKNOWN,
  RTD_PT100,
  RTD_PT1000
};

//Detect RTD type based on resistance measurement
RTDType detectRTDType() {
  //Read both channels
  uint16_t adc100 = readPT100();
  uint16_t adc1000 = readPT1000();
  
  // Convert to resistances
  const float VREF = 3.3f;
  const float AMP_GAIN = 100.0f;
  
  float r100 = ((adc100 / 1023.0f) * VREF / AMP_GAIN) / 1.0e-3f;
  float r1000 = ((adc1000 / 1023.0f) * VREF / AMP_GAIN) / 0.1e-3f;
  
  // Determine if reading makes sense for each RTD type
  if (r100 > 80.0f && r100 < 140.0f) {   
    return RTD_PT100;
  } else if (r1000 > 800.0f && r1000 < 1400.0f) { 
    return RTD_PT1000;
  }
  
  return RTD_UNKNOWN;
}


void setup() {
  Serial.begin(115200);
  while (!Serial); 
  
  // Initialize SPI
  initSPI();
  
  Serial.println("RTD Temperature Measurement System Initialized");
}

void loop() {
  // Auto-detect RTD type
  RTDType rtdType = detectRTDType();
  
  // Read and convert both sensors
  uint16_t adc100 = readPT100();
  float temp100 = adcToTemperature(adc100, false);
  
  uint16_t adc1000 = readPT1000();
  float temp1000 = adcToTemperature(adc1000, true);
  
  // Determine which reading to use based on detection
  float finalTemp;
  if (rtdType == RTD_PT1000) {
    finalTemp = temp1000;
    Serial.print("Detected PT1000. ");
  } else if (rtdType == RTD_PT100) {
    finalTemp = temp100;
    Serial.print("Detected PT100. ");
  } else {
    // Default to PT1000 if detection fails
    finalTemp = temp1000;
    Serial.print("Detection failed, using PT1000. ");
  }
  
  // Print results
  Serial.print("PT1000: ");
  Serial.print(temp1000);
  Serial.print("°C, PT100: ");
  Serial.print(temp100);
  Serial.print("°C, Selected: ");
  Serial.print(finalTemp);
  Serial.println("°C");
  
  delay(1000); // Wait 1 second between measurements to minimize self-heating
}