/*
 * CONFIRMED OPERATIONAL: 2017-02-17
 * SYSTEM DEVELOPED ON SOFTWARE VERSIONS:
 * Arduino 1.8.1
 * Python 3.2.5
 * PyGame 1.9.2pre
 * PySerial 2.6
 * SYSTEM DEVELOPED ON ARDUINO HARDWARE: 
 * "MINI USB Nano V3.0 ATmega328P CH340G 5V 16M"
 */

//This ring buffer takes data from USB > serial and stores it.
//The byte "0xFF" is transmitted out over serial > USB when
//there's no incoming data and there's space in the buffer
//for another 256 bytes.
//Output samples are pushed to the DAC at 1,953.125 Hz (every 512us).
//In my tests, Timer 2 CTC mode, which would (theoretically) allow a nice 
//round-number sample rate, DOES NOT PLAY NICELY with the other 
//libraries used here, <SPI.h>, and Serial.h (implicit in Arduino IDE).
//------------------------------------
//After many hours of poking and prodding, I have decided that this code,
//which appears stable, is sufficient.


 
#include <SPI.h> // necessary library for SPI data transmission to DAC7311

static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//static uint8_t cs = 10; // using digital pin 10 for DAC7311 chip select
static uint8_t cs = 15; // using digital pin 10 for DAC7311 chip select
static uint8_t cs2 = 27; // using digital pin 10 for DAC7311 chip select

volatile uint8_t loadpoint = 0;     //the index of the location in databuffer[]
                                    //that our loaded data goes up to (moves up as we write)

volatile uint8_t readpoint = 0;     //the index of the location in databuffer[]
                                    //that we haven't read above (moves up as we read)

volatile uint16_t databuffer[256];  //our buffer array of unsigned 16-bit integers

volatile uint8_t should_send = 0;   //this gets set to 1 every 512us by an interrupt.
                                    //It's polled/reset to 0 in loop()



//-------------------------------------------------------------------------------
//  INTERRUPT SERVICE ROUTINE (FANCY SPECIAL FUNCTION):
//
//  WHEN TIMER2 OVERFLOWS, NO MATTER WHERE WE ARE**, WE GO HERE,
//  SET THE VOLATILE GLOBAL VARIABLE "should_send" TO 1,
//  AND THEN WE GO BACK TO WHATEVER WE WERE DOING BEFORE
//
// **except within certain parts of the Serial library. Unless
// you feel like going through a lot of TIMSK register descriptions in the
// 660-page ATMEGA328P datasheet, and cross-referencing them with the
// interrupt register bits set in the Arduino Serial library files,
// just... don't worry about it.
// However, as a result of interference by all the hidden interrupt
// timing/permissions/disabling/enabling that takes place in the Arduino Serial library...
// Don't set timer2 any faster than it's set in   setup() !
// (timer2 overflow interrupt every 512us)
// SPEEDING UP TIMER2 WILL NOT INCREASE THE OUTPUT SAMPLE RATE AS YOU WOULD EXPECT!!
//-------------------------------------------------------------------------------
//ISR(TIMER2_OVF_vect){           //When timer2 overflows,
//  should_send = 1;              //it's time to send more data to the DAC.
//}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------
//  REQUEST DATA IF READY
//-------------------------------------------------------------------------------
void request_data_if_ready(void){
                                //(readpoint - loadpoint) == number of buffer addresses available.
                                //Total number of buffer addresses is 256, and we take data in 256-byte blocks.
                                //Our buffer addresses each require two bytes to fill.
                             
  uint8_t buffer_locations_available = readpoint - loadpoint;                              
  if((loadpoint == readpoint) || ((buffer_locations_available >= 96) && ((buffer_locations_available % 16) == 0)) ){    
    //If we have space for more data in the buffer, either (buffer is completely empty), or (buffer has space for another 256-byte block)
      Serial.write(0xFF);               //Request more data for our buffer! (Writing the byte 0xFF tells the computer we're ready for more data.)
                                      //the"magic number" 96 comes from: 128 buffer locations will be written, but given 115200 baud serial, 
                                      //((256bytes to receive *8bits per byte*(1/115200bits per second))/0.000512seconds per buffer pop = 34.72222222buffer locations lost during receiving
                                      // so 93.277 "actual" locations are written per 128-location data block, ideally, ignoring USB latency...
                                      //in my empirical tests on my system USB latency is 16 ms one-way, so 32ms latency round-trip (63 ticks)
  }

}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------
//  LOAD DATA IF AVAILABLE
//-------------------------------------------------------------------------------
void load_data_if_available(void){      //Loads data, as long as there is data to load. 
  uint8_t buffer_locations_filled = loadpoint - readpoint;
        //###############################################################
                 //   BE WARNED!! THIS FUNCTION DOES NOT CHECK IF
                 //   LOADING MORE DATA WILL OVERWRITE YOUR BUFFER!
                 //   IT DIRECTLY LOADS WHATEVER DATA IS AVAILABLE!
        //###############################################################
                  
                                        //It is assumed that you will request data using
                                        //request_data_if_ready(), which checks that you have
                                        //enough remaining buffer capaciity...
                                        //*before* it asks for more data.
  
  if(Serial.available() > 31){        //As long as there are thirty-two or more bytes to load,
    for(uint8_t i=0;i<16;i++){
    uint8_t highbyte = Serial.read();   //First we save the high byte, then
    uint8_t lowbyte = Serial.read();    //We save the low byte, and
    loadpoint++;                        //We increment the load pointer.
                                            
                                        //"RING BUFFER" : Incrementing moves loadpoint up one slot,
                                        //away from readpoint -- the space below loadpoint
                                        //but above readpoint is where the buffer lives; therefore
                                        //note, with care: haphazardly writing data from the computer
                                        //to the Arduino, thus overfilling the buffer until loadpoint
                                        //wraps around to the same location (aka value) as readpoint, will
                                        //cause output_data_if_ready() to think that the buffer is empty!
                                        //The buffer is a ring because loadpoint and readpoint are both uint8_t,
                                        //so they automatically wrap around when they go above 255, and no extra
                                        //test/reset/whatever code is needed to make the buffer behave 
                                        //like a nice ring.
                                                                                
    databuffer[loadpoint] =             //To convert our two 8-bit bytes into a single uint16_t:
    
      ((uint16_t)highbyte << 8)         //We cast highbyte (let's pretend its value is 0xFA) from
                                        //unsigned 8-bit to unsigned 16-bit integer, yielding 0x00FA,
                                        //then perform a logical shift left 8 places, yielding 0xFA00.
                                        
        | (uint16_t)lowbyte;            //We then cast lowbyte (we'll pretent lowbyte is 0xCE) to uint16_t
                                        //as well, yielding 0x00CE. We perform a bitwise OR between 
                                        //lowbyte (now 0x00CE) and shifted highbyte (0xFA00) to obtain 
                                        //0xFACE, a uint16_t we can store in databuffer[loadpoint].
    }
  }
  if((buffer_locations_filled == 0) && (Serial.available() > 1)){ //if the buffer's completely empty, and there are at lest two bytes available...
    uint8_t highbyte = Serial.read();   //First we save the high byte, then
    uint8_t lowbyte = Serial.read();    //We save the low byte, and
    loadpoint++;                        //We increment the load pointer.                                       
                                                                          
    databuffer[loadpoint] =
    
      ((uint16_t)highbyte << 8)         
                                        
        | (uint16_t)lowbyte;            
  }
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------
//  OUTPUT DATA IF READY
//-------------------------------------------------------------------------------
void output_data_if_ready(void){        //Outputs data, as long as:
  if(should_send == 1){                 //The timer2 overflow interrupt service routine says it's time
                                        //to send data (it's been 512 microseconds, buddy, you gonna stand here all day?),
    if((loadpoint - readpoint) != 0){   //and we actually have data to send. If we do,
      readpoint++;                      //move the pointer up one,
      DACwrite(databuffer[readpoint]);  //we output the 16 bits at the current location of the read pointer,
//  delay(1);
      DAC2write(databuffer[readpoint]);  //we output the 16 bits at the current location of the read pointer,
      
      should_send = 0;                  //and note that we shouldn't send more data until timer2 overflows again.
    }
  }
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------
//  DAC WRITE FUNCTION
//-------------------------------------------------------------------------------
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
void DAC2write(uint16_t value){  //Why is this a separate function? So you can easily play with
                                //the DAC without following my specific read/write/buffer protocol!
                                                                
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(cs2, LOW);        //pull *CS LOW ("Hey, DAC, listen! We're talking to you!")
//  SPI.transfer16(value);        //write the value to the DAC
  hspi->transfer16(value);        //write the value to the DAC
  digitalWrite(cs2, HIGH);       //pull *CS HIGH ("DAC, we're done talking to you. Output the value we sent you.")
  hspi->endTransaction();

      Serial.write(value);               //Request more data for our buffer! (Writing the byte 0xFF tells the computer we're ready for more data.)
  
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

  double offset=0;
  double amplitude=2;
  const double frequency = 78;
  const int samples_per_cycle = int(round(1/(float(frequency) * .000512)));
  const double frequency_base = 6;
  const int samples_per_cycle_base = int(round(1/(float(frequency_base) * .000512)));
  double wave_list[samples_per_cycle_base];
  int sample_now;


//-------------------------------------------------------------------------------
//  SETUP FUNCTION
//------------------------------------------------------------------------------- 
void setup(){                   //This function configures our timer, buffer, and serial communications.
  Serial.begin(115200);         //start serial at 115200 baud, no fancy stuff
  Serial.println(samples_per_cycle_base);

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

  for( int x; x<=samples_per_cycle_base;x++)
  {
    wave_list[x] = amplitude*(sin(2*(3.1415926)*(double(x)/double(samples_per_cycle_base)))) + offset;
    if ((x >= samples_per_cycle_base/12) && (x < samples_per_cycle_base/2 - samples_per_cycle_base/12))
    {
      wave_list[x] = wave_list[x] + amplitude*(4/5)*(sin(2*(3.1415926)*(double(x)/double(samples_per_cycle)))) + offset;
    }
  }
  wave_list[samples_per_cycle_base - 1] = (wave_list[0] + wave_list[samples_per_cycle_base - 2])/2.0;
  sample_now=0;
  Serial.println(wave_list[sample_now]);

  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  //alternatively route through GPIO pins of your choice
  //hspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(); 
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(5, OUTPUT); //VSPI SS
  pinMode(15, OUTPUT); //HSPI SS
  pinMode(27, OUTPUT); //HSPI SS
  digitalWrite(cs, HIGH);       //pull *CS HIGH ("DAC, we're done talking to you. Output the value we sent you.")
  digitalWrite(cs2, HIGH);       //pull *CS HIGH ("DAC, we're done talking to you. Output the value we sent you.")
  
  cli();                        //first we disable interrupts,
                                //because an interrupt happening during this setup stuff could really interfere with our code!
//-------------------------------------------------------------------------------
                                //set up data buffer:
  for(uint16_t i=0;i<256;i++){  //for each value in the data buffer,
    databuffer[i]= 0;           //store a zero in the data buffer.
  }                             //(if we don't do this, the compiler might not allocate any memory for the buffer)
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------  
                                //SPI setup steps:
//  pinMode(cs, OUTPUT);          //we use this pin (D10) as an output for *CS ( * means active low) chip select pin
//  SPI.begin();                  //start SPI
  
//  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE1)); 
//  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE1)); 
//  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
//  hspi->beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE1)); 
                                //max SPI transmit speed is 50MHz, most significant bit first, SPI mode 1 (CPOL = 0, CPHA = 1)
//-------------------------------------------------------------------------------
                                //setting up timer2 to generate an overflow interrupt every 512us:
                                
//  TCCR2A &= 0xFC;               //TCCR2A configuration:
                                //set timer2 to normal (not CTC) mode.
                                //timer2 simply always runs, overflows at >255 (it's a uint8_t), and sets an interrupt flag when it overflows.
                           
//  TCCR2B |= 0x03;               //TCCR2B configuration:
//  TCCR2B &= 0xF3;               //set clock prescaler (divider) to 32, so effective frequency is:
                                //16000000_CPU_SPEED/(32_PRESCALER * 256_CLOCKS_TO_OVERFLOW) = 1,953.125 Hz, overflow every 512us                                    
  
//  TIMSK2 |= (1 << TOIE2);       //TIMSK2 (Timer Interrupt Mask 2) configuration:
                                //enable overflow interrupt. Timer Overflow Inerrupt Enable 2(as in, the one for timer2)
                                
//  TCNT2 = 0x00;                 //TCNT2 (Timer/Counter register 2) configuration:
                                //set the timer2 counter to zero, to be sure it ACTUALLY STARTS at zero when our loop starts.
                                //Who knows what value it powered on with?
//-------------------------------------------------------------------------------
                                //all our sensitive one-time setup stuff is done, so we can now safely  
  sei();                        //enable interrupts
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------



//-------------------------------------------------------------------------------
//  MAIN LOOP FUNCTION
//-------------------------------------------------------------------------------
void loop(){                    //Loop forever:
  should_send = 1;              //it's time to send more data to the DAC.
//  output_data_if_ready();       //If we have data to output and it's time to send it, send it!
//-------------------------------------------------------------------------------
  should_send = 1;              //it's time to send more data to the DAC.
//  request_data_if_ready();      //If we have enough space in the buffer, and we're 
                                //not in the process of loading more data already,
                                //request another 256 bytes.
//-------------------------------------------------------------------------------
  should_send = 1;              //it's time to send more data to the DAC.
//  load_data_if_available();     //If we have data coming in, load it into the buffer.

//  DACwrite(0x06D0);

  double value_in_mA;

  if( sample_now>samples_per_cycle_base)
  {
    sample_now=0;
  }
  value_in_mA=wave_list[sample_now];
    if(value_in_mA > 2.002) //check positive limit
    {
      value_in_mA = 2.001;
    }
    if(value_in_mA < -2.002)//check negative limit
    {
      value_in_mA = -2.001;
    }
    int dacwrite = (int(round(((16383*1.0866)/5)*(2.5-value_in_mA))));
    DACwrite(dacwrite);
    sample_now++;
  Serial.println(value_in_mA);


//    samples_per_cycle = int(round(1/(float(frequency) * .000512)))
//    wave_list = [0]* samples_per_cycle
//    if(waveform == "six_hz_and_sine"):
//        frequency_base = 6
//        samples_per_cycle_base = int(round(1/(float(frequency_base) * .000512)))
//        wave_list = [0]* samples_per_cycle_base
//        for x in range (0, samples_per_cycle_base):
//            wave_list[x] = amplitude*(math.sin(2*(math.pi)*(x/samples_per_cycle_base))) + offset
//            #if ((x >= 0) and (x < samples_per_cycle_base/2)):
//            if ((x >= samples_per_cycle_base/12) and (x < samples_per_cycle_base/2 - samples_per_cycle_base/12)):
//                wave_list[x] = wave_list[x] + amplitude*(4/5)*(math.sin(2*(math.pi)*(x/samples_per_cycle))) + offset
//        wave_list[samples_per_cycle_base - 1] = (wave_list[0] + wave_list[samples_per_cycle_base - 2])/2.0

    
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
/*   ALL HAIL MACHINE EMPIRE   */
