/*
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.
 */

#include <SPI.h> // necessary library for SPI data transmission to DAC7311

static const int spiClk = 1000000; // 1 MHz
SPIClass * hspi = NULL;
static uint8_t cs = 15; // using digital pin 10 for DAC7311 chip select

  double offset=0;
  double amplitude=0.5;
//  double amplitude=1;
  const double frequency = 84;
  const int samples_per_cycle = int(round(1/(float(frequency) * .000512)));
  const double frequency_base = 6;
  const int samples_per_cycle_base = int(round(1/(float(frequency_base) * .000512)));
  double wave_list[samples_per_cycle_base];
  int wave_list_dacwrite[samples_per_cycle_base];
  int sample_now;

volatile uint8_t should_send = 0;   //this gets set to 1 every 512us by an interrupt.

// Stop button is attached to PIN 0 (IO0)
#define BTN_STOP_ALARM    0

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
    should_send = 1;
}


void setup() {
  Serial.begin(115200);

  if(offset > 2.002)
  {
    offset = 2.001;
  }
  if(offset < -2.002)
  {
    offset = -2.001;
  }
  if(amplitude > 2.002)
  {
    amplitude = 2.001;
  }

  for( int x=0; x<samples_per_cycle_base;x++)
  {
    wave_list[x] = amplitude*(sin(2.0*(3.1415926)*(double(x)/double(samples_per_cycle_base)))) + offset;
//    wave_list[x] = 0;
    if ((x >= samples_per_cycle_base/12) && (x < samples_per_cycle_base/2 - samples_per_cycle_base/12))
//    if ((x >= samples_per_cycle_base/12) && (x < samples_per_cycle_base - samples_per_cycle_base/12))
    {
      wave_list[x] = wave_list[x] + amplitude*(4.0/5.0)*(sin(2.0*(3.1415926)*(double(x)/double(samples_per_cycle)))) + offset;
    }
  }
  wave_list[samples_per_cycle_base - 1] = (wave_list[0] + wave_list[samples_per_cycle_base - 2])/2.0;
  sample_now=0;

    double value_in_mA;

  for( int x=0; x<samples_per_cycle_base;x++)
  {
    value_in_mA=wave_list[x];
    if(value_in_mA > 2.002) //check positive limit
    {
      value_in_mA = 2.001;
    }
    if(value_in_mA < -2.002)//check negative limit
    {
      value_in_mA = -2.001;
    }
    wave_list_dacwrite[x] = (int(round(((16383*1.0866)/5)*(2.5-value_in_mA))));
  }
  hspi = new SPIClass(HSPI);
  hspi->begin(); 
  pinMode(15, OUTPUT); //HSPI SS

  // Set BTN_STOP_ALARM to input mode
  pinMode(BTN_STOP_ALARM, INPUT);

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
//  timerAlarmWrite(timer, 1000000/512, true);
  timerAlarmWrite(timer, 1000000/(samples_per_cycle_base*frequency_base), true);
//  timerAlarmWrite(timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void DACwrite(uint16_t value){  //Why is this a separate function? So you can easily play with
                                //the DAC without following my specific read/write/buffer protocol!
                                                                
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);        //pull *CS LOW ("Hey, DAC, listen! We're talking to you!")
//  SPI.transfer16(value);        //write the value to the DAC
  hspi->transfer16(value);        //write the value to the DAC
  digitalWrite(cs, HIGH);       //pull *CS HIGH ("DAC, we're done talking to you. Output the value we sent you.")
  hspi->endTransaction();

//      Serial.write(value);               //Request more data for our buffer! (Writing the byte 0xFF tells the computer we're ready for more data.)
  
}

void loop() {
//  // If Timer has fired
//  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
//    uint32_t isrCount = 0, isrTime = 0;
//    // Read the interrupt count and time
//    portENTER_CRITICAL(&timerMux);
//    isrCount = isrCounter;
//    isrTime = lastIsrAt;
//    portEXIT_CRITICAL(&timerMux);
//    // Print it
////    Serial.print("onTimer no. ");
////    Serial.print(isrCount);
////    Serial.print(" at ");
////    Serial.print(isrTime);
////    Serial.println(" ms");
//  }
//  // If button is pressed
//  if (digitalRead(BTN_STOP_ALARM) == LOW) {
//    // If timer is still running
//    if (timer) {
//      // Stop and free timer
//      timerEnd(timer);
//      timer = NULL;
//    }
//  }

  if(should_send == 1)
  {
    should_send = 0;                  //and note that we shouldn't send more data until timer2 overflows again.
    
//    double value_in_mA;

    if( sample_now>samples_per_cycle_base)
    {
      sample_now=0;
    }
//    value_in_mA=wave_list[sample_now];
//    if(value_in_mA > 2.002) //check positive limit
//    {
//      value_in_mA = 2.001;
//    }
//    if(value_in_mA < -2.002)//check negative limit
//    {
//      value_in_mA = -2.001;
//    }
//    int dacwrite = (int(round(((16383*1.0866)/5)*(2.5-value_in_mA))));
    DACwrite(wave_list_dacwrite[sample_now]);
    sample_now++;
  }

}
