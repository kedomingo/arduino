/**
 * Dual frequency generator controlled by rotary encoder
 *
 * This supports frequencies up to the clockspeed of the chip being used
 * For atmega328p, this is 16MHz. Calculation of the appropriate pre-scalers
 * as well as setting the timing registers for both Timer 1 and Timer 2 are included in the code.
 * 
 * IMPORTANT: The frequencies are halved on the Arduino pro mini 3.5v (8MHz). I'm not 
 * really sure why since the clockspeed constant F_CPU should be 8000000 in the pro mini
 * and should adjust the other values accordingly, but the oscilloscope shows otherwise.
 * I didn't bother spending time with it, so just DOUBLE THE FREQUENCY ON THE 8MHz ARDUINO PRO MINI
 *
 * Pins:
 * Timer 1 generates a PWM signal on pin OC1A (Digital 9)
 * Timer 2 generates a PWM signal on pin OC2B (Digital 3)
 * 
 * Duty cycle adjustment using rotary encoder on digital pins 4, 5, 6 for CLK, DT, SW
 * Toggles between frequency 1 and frequency 2 duty cycle adjustment using 
 * the push button on the rotary encoder.
 * 
 * @author Kyle Domingo https://github.com/kedomingo/arduino
 * 
 * main sources:
 *  https://wolles-elektronikkiste.de/en/timer-and-pwm-part-2-16-bit-timer1
 *  https://wolles-elektronikkiste.de/en/timer-and-pwm-part-1-8-bit-timer0-2
 *
 *
 * License:
 * Creative Commons Attribution-NonCommercial-ShareAlike (CC-BY-NC-SA)
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 */

// Timer behavior modes
#define TIMER_OPERATION_MODE_NORMAL 1
#define TIMER_OPERATION_MODE_PWM 2

// Rotary Encoder Inputs
#define ROTARY_CLK 4
#define ROTARY_DT 5
#define ROTARY_SW 6

#define FREQ_1_SELECT 1
#define FREQ_2_SELECT 2

// This is the slowest frequency timer 2 can handle in PWM mode due to its low resolution
#define MIN_PWM_TIMER_2 61

// ----------------------------------
// Class definitions
// ----------------------------------

class Timer {
  protected:
    /**
     * Wave frequency in Herz. 1-F_CPU
     */
    unsigned long frequencyHz = 0;
    
    /**
     * PWM Duty cycle in whole percent. 1-100
     */
    unsigned int dutyCycle = 0;

    /**
     * The cycle pre-scaler
     */
    unsigned int preScaler;

    /**
     * In normal mode, counter starts at this point until timer overflow is hit
     */
    unsigned long counterStart;

    /**
     * In pwm mode, counter starts at 0 and uses CTC mode to compare the counter 
     * when it hits this top
     */
    unsigned long counterTop;

    /**
     * This timer's operation mode
     */
    unsigned int operationMode;

  public: 
    Timer() {}
    
    Timer(unsigned long frequencyHz) {
      this->frequencyHz = max(0, min(F_CPU, frequencyHz));
      this->dutyCycle = 100;
    }
    
    Timer(unsigned long frequencyHz, int dutyCycle) {
      this->frequencyHz = max(0, min(F_CPU, frequencyHz));
      this->dutyCycle = max(0, min(100, dutyCycle));
    }

    void interrupt() {
      switch (operationMode) {
        case TIMER_OPERATION_MODE_NORMAL:
          this->interruptNormal();
      }
    }
    
    virtual void normalMode();
    virtual void pwmMode();
    virtual void adjustDutyCycle(int newDutyCycle);

  protected:
    virtual void interruptNormal();
};

/**
 * This is the 16 bit timer1
 */
class Timer1: public Timer {

  public:
    explicit Timer1(): Timer() {}
    
    explicit Timer1(unsigned long frequencyHz): Timer(frequencyHz) {}
    
    explicit Timer1(unsigned long frequencyHz, int dutyCycle): Timer(frequencyHz, dutyCycle) {}

    void normalMode() {
      operationMode = TIMER_OPERATION_MODE_NORMAL;
      
      preScaler = getPreScaler(frequencyHz);
      Serial.print("Prescaler value 1: ");
      Serial.println(preScaler);
      
      // Compare Output Mode Bits: OC1A OC1B disconnected on Normal Port Operation Mode
      TCCR1A = 0;
    
      // Set pre-scaler bits
      TCCR1B = getPreScalerBitSettings(preScaler);
    
      // Interrupt on timer (TCNT1) overflow 
      TIMSK1 |= (1<<TOIE1);
    
      // Set counter start
      TCNT1 = counterStart = getCounterStart(frequencyHz, preScaler);
      Serial.print("Start 1: ");
      Serial.println(counterStart);
    }

    void pwmMode() {
      operationMode = TIMER_OPERATION_MODE_PWM;

      preScaler = getPreScaler(frequencyHz);
      Serial.print("PWM Prescaler value 1: ");
      Serial.println(preScaler);
      
      counterTop = getCounterTop(frequencyHz, preScaler);
      Serial.print("PWM TOP value 1: ");
      Serial.println(counterTop);
      

      // Set Mode 14: Fast PWM, ICR1 TOP, Update of OCR1x at BOTTOM, TOV Flag set on TOP
      // WGM11 is set at TCCR1A; WGM12 and WGM13 are set at TCCR1B 
      // (See https://wolles-elektronikkiste.de/en/timer-and-pwm-part-2-16-bit-timer1 : Timer/Counter1 Control Register TCCR1x)
      
      // Clear OC1A/OC1B on Compare Match / Set OC1A/OC1B at Bottom (COM1A1)
      // + Set waveform generation mode bit
      TCCR1A = (1<<COM1A1) + (1<<WGM11);

      // Set pre-scaler bits
      // + Set waveform generation mode bits
      TCCR1B = getPreScalerBitSettings(preScaler) + (1<<WGM13) + (1<<WGM12);

      Serial.print("PWM Duty cycle step: ");
      Serial.println(getDutyCycleSteps(counterTop, dutyCycle));

      // In this mode, ICR1 is TOP, OCR1x at BOTTOM      
      ICR1 = counterTop;
      OCR1A = getDutyCycleSteps(counterTop, dutyCycle);

      // In PWM mode, DDRB refers to data direction register for digital pins 8-13
      // PB1 is OC1A, PB2 is OC1B, (digital pin 9, 10 respectively)
      // This will set pin 9 (OC1A) as pwm output [because OCR1A is set as bottom?]
      DDRB |= (1<<PB1);
    }

    /**
     * Adjust OCR1A Based on new duty cycle setting
     */
    void adjustDutyCycle(int newDutyCycle) {
      dutyCycle = newDutyCycle;
      
      if (operationMode != TIMER_OPERATION_MODE_PWM) {
        return;
      }
      
      OCR1A = getDutyCycleSteps(counterTop, dutyCycle);
    }

  protected:
    /**
     * Bit settings for timer 1
     */
    uint32_t getPreScalerBitSettings(int preScaler) {
      switch (preScaler) {
        case 1024:
          Serial.println("Returning bit settings for 1024");
          return (1<<CS12) + (1<<CS10);
        case 256:
          Serial.println("Returning bit settings for 256");
          return (1<<CS12);
        case 64:
          return (1<<CS11) + (1<<CS10);
        case 8:
          return (1<<CS11);
        default:
          return (1<<CS10);
      }
    }

    /**
     * Get the appropriate pre-scaler.
     * The resulting scale should be such that the 1 pulse is less than 65536 steps of the counter
     * i.e. counterStart is not negative because the pulse exceeds the max counter 65536
     */
    uint32_t getPreScaler(uint32_t frequency) {
      if (getCounterStart(frequency, 1) > 0) {
        return 1;
      }
      if (getCounterStart(frequency, 8) > 0) {
        return 8;
      }
      if (getCounterStart(frequency, 64) > 0) {
        return 64;
      }
      if (getCounterStart(frequency, 256) > 0) {
        return 256;
      }
      return 1024;
    }
    
    /**
     * In normal mode,  start-65536 is used
     */
    int32_t getCounterStart(uint32_t frequency, int preScaler) {
      float start = ((float) F_CPU) / (preScaler * frequency);

      // We cannot return 65536, otherwise the formula for the frequncy
      // could encounter a division by zero. Round it up to 1
      if (round(start) == 0) {
        return 65535;
      }
      
      return 65536 - round(start);
    }

    /**
     * In PWM mode, 0-top is used
     */
    uint32_t getCounterTop(uint32_t frequency, int preScaler) {
      float top = ((float) F_CPU) / (preScaler * frequency);

      return round(top) - 1;
    }

    /**
     * In PWM mode, the duty cycle is a fraction of the pulse width (0-counterTop)
     */
    uint32_t getDutyCycleSteps(uint32_t counterTop, int dutyCycle) {
      float step = ((float) counterTop * dutyCycle) / 100; 
      
      return round(step);
    }

    void interruptNormal() {
      // Reset start counter
      TCNT1 = counterStart;
      
      Serial.println("    ISR TIMER1_OVF_vect");
    }
};

/**
 * This is for the 8-bit timer timer2
 */
class Timer2: public Timer {
  private:
    int slowerFactor;
    
  public:
    explicit Timer2(): Timer() {}
    
    explicit Timer2(unsigned long frequencyHz): Timer(frequencyHz) {}
    
    explicit Timer2(unsigned long frequencyHz, int dutyCycle): Timer(frequencyHz, dutyCycle) {}
    
    void normalMode() {
      operationMode = TIMER_OPERATION_MODE_NORMAL;
      
      setBestPreScalerAndFactor(frequencyHz);
      Serial.print("Prescaler value 2: ");
      Serial.println(preScaler);
    
      Serial.print("Slower factor: ");
      Serial.println(slowerFactor);
      
      // Compare Output Mode Bits: OC2A OC2B disconnected on Normal Port Operation Mode
      TCCR2A = 0;
    
      // Set counter start
      TCNT2 = counterStart = getCounterStart(frequencyHz, preScaler, slowerFactor);
        
      // Set pre-scaler bits
      TCCR2B = getPreScalerBitSettings(preScaler);
    
      // Interrupt on timer (TCNT1) overflow 
      TIMSK2 |= (1<<TOIE1);
    
      Serial.print("Start 2: ");
      Serial.println(counterStart);
    }

    /**
     * For timer 2 PWM, slowest possible is around 61Hz.
     * This uses a prescaler of 1024 and top of 255
     */
    void pwmMode() {
      operationMode = TIMER_OPERATION_MODE_PWM;

      if (frequencyHz < MIN_PWM_TIMER_2) {
        Serial.print("WARNING: Timer2 does not support frequencies less than ");
        Serial.print(MIN_PWM_TIMER_2);
        Serial.print("Hz. Setting frequency to ");
        Serial.println(MIN_PWM_TIMER_2);
        frequencyHz = MIN_PWM_TIMER_2;
      }

      setBestPwmPreScalerAndTop(frequencyHz, dutyCycle);
      Serial.print("PWM Prescaler value 2: ");
      Serial.println(preScaler);

      Serial.print("PWM TOP value 2: ");
      Serial.println(counterTop);
      Serial.print("PWM Duty cycle step: ");
      Serial.println(getDutyCycleSteps(counterTop, dutyCycle));

      // Set Mode 7: Fast PWM, OCRA TOP, Update of OCRx at BOTTOM, TOV Flag set on TOP
      // WGM22 is set at TCCR2B; WGM20 and WGM21 are set at TCCR2A
      // (See https://wolles-elektronikkiste.de/en/timer-and-pwm-part-1-8-bit-timer0-2 : The Timer/Counter Control Registers TCCRxy)
      
      // Clear OC2B on Compare Match, Set OC2B at bottom (COM2B1)
      // + Set waveform generation mode bits
      TCCR2A = (1<<COM2B1) + (1<<WGM21) + (1<<WGM20);

      // Set pre-scaler bits
      // + Set waveform generation mode bit
      TCCR2B = (1<<CS22) + (1<<CS21) + (1<<CS20) + (1<<WGM22); //getPreScalerBitSettings(preScaler) + (1<<WGM22);

      // In this mode, OCRA is TOP, OCRB is bottom and determines duty cycle
      OCR2A = counterTop;
      OCR2B = getDutyCycleSteps(counterTop, dutyCycle);

      // In PWM mode, DDRD refers to data direction register for digital pins 0-7
      // PB3 is OC2A, PD3 is OC2B (digital pin 11, 3 respectively).
      
      // This will set pin 3 (OC2B) as pwm output [because because of the setting WGM settings "Clear OC2B on Compare Match, Set OC2B at bottom"]
      DDRD |= (1<<PD3);
    }

    /**
     * Adjust OCR2B Based on new duty cycle setting
     */
    void adjustDutyCycle(int newDutyCycle) {
      dutyCycle = newDutyCycle;
      
      if (operationMode != TIMER_OPERATION_MODE_PWM) {
        return;
      }
      
      OCR2B = getDutyCycleSteps(counterTop, dutyCycle);
    }

  protected: 
    /**
     * Bit settings for timer 2
     */
    uint32_t getPreScalerBitSettings(int preScaler) {
      switch (preScaler) {
        case 1024:
          return (1<<CS22) + (1<<CS21) + (1<<CS20);
        case 256:
          return (1<<CS22) + (1<<CS21);
        case 128:
          return (1<<CS22) + (1<<CS20);
        case 64:
          return (1<<CS22);
        case 32:
          return (1<<CS21) + (1<<CS20);
        case 8:
          return (1<<CS21);
        default:
          return (1<<CS20);
      }
    }
    
    /**
     * Get the appropriate prescaler for timer 2 in normal mode
     */
    uint32_t setBestPreScalerAndFactor(uint32_t frequency) {
      float preScalers[] = {1, 8, 32, 64, 128, 256, 1024};
      float scaleFactors[] = {1, 2, 4, 8, 10, 20, 30, 40, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};

      float bestPreScaler = 1;
      float bestScaleFactor = 1;
      float bestError = 1000;
      
      float error;
      float freq;
      long start;

      for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 10; j++) {
          
          start = getCounterStart(frequency, preScalers[i], scaleFactors[j]);

          // 128 is an arbitrary limit for a good place to start. 
          // No idea what a "good value" should be as I could not find a definite criteria
          if (start > 0 && start < 128) {
                     
            freq = ((float)F_CPU) / (preScalers[i] * (256 - start)) / scaleFactors[j];
            error = abs(frequency-freq);
            
            if (error < bestError) {
              bestError = error;
              bestPreScaler = preScalers[i];
              bestScaleFactor = scaleFactors[j];
            }
          }
        }
      }

      preScaler = bestPreScaler;
      slowerFactor = bestScaleFactor;
    }

    /**
     * Get the appropriate prescaler for timer 2 in PWM mode
     */
    uint32_t setBestPwmPreScalerAndTop(uint32_t frequency, int dutyCycle) {
      float preScalers[] = {1, 8, 32, 64, 128, 256, 1024};

      float bestPreScaler = 1;
      float bestTop = 255;
      // In some frequencies, we cannot find a good enough result, due to limited resolution
      // Make an initial error of 100KHz
      float bestError = 100000;
      
      float error;
      float freq;

      for (byte i = 0; i < 7; i++) {
        for (float top = 0; top < 256; top++) {
          freq = ((float) F_CPU) / (preScalers[i] * (top + 1));
          error = abs(freq - frequency);

          if (error < bestError) {
            bestError = error;
            bestPreScaler = preScalers[i];
            bestTop = top;
          }
        }
      }
      
      this->counterTop = bestTop;
      this->preScaler = bestPreScaler;
    }

    /**
     * In normal mode, we start at counterStart and let the counter overflow to 256
     */
    int32_t getCounterStart(uint32_t frequency, uint32_t preScaler, uint32_t slowerFactor) {
      float start = ((float) F_CPU) / (frequency * preScaler * slowerFactor);

      // We cannot return 256 due to rounding error. Otherwise the formula for frequency 
      // could encounter a divsion by zero. Round it up to 1
      if (round(start) == 0) {
        return 255;
      }
      
      return 256 - round(start);
    }

    /**
     * In PWM mode, the duty cycle is a fraction of the pulse width (0-counterTop)
     */
    uint32_t getDutyCycleSteps(uint32_t counterTop, int dutyCycle) {
      float step = ((float) counterTop * dutyCycle) / 100; 
      
      return round(step);
    }

    /**
     * In normal operation, TCNT2 overflows at 256. This is very fast for 16M cycles
     * so we use a factor to scale down the events performed.
     * For example, for a 1Hz frequency, this factor will be 16000000/256 = 62,500
     * i.e., it will take 62500 rounds of 0-255 to reach 16000000
     * So events will only trigger after 62500x256 steps = 16000000 steps = 1 sec.
     */
    void interruptNormal() {
      static int counter = 0;
      
      // Reset start counter
      TCNT2 = counterStart;
      
      if (++counter < slowerFactor) {
        return;
      }
    
      counter = 0;

      Serial.println("ISR TIMER2_OVF_vect");
    }
};

class RotaryEncoder {
  private:
    int clkPin, dtPin, swPin;

    int currentStateCLK;
    int lastStateCLK;
    String currentDir ="";
    unsigned long lastButtonPress = 0;
    
  public:
    RotaryEncoder() {}
  
    RotaryEncoder(int clkPin, int dtPin, int swPin) {
     this->clkPin = clkPin; 
     this->dtPin = dtPin; 
     this->swPin = swPin; 
    }

    void setupInputs() {
      pinMode(clkPin, INPUT);
      pinMode(dtPin, INPUT);
      pinMode(swPin, INPUT_PULLUP);

      // Read the initial state of ROTARY_CLK
      lastStateCLK = digitalRead(clkPin);
    }
      
    /**
     * Get the rotation value of the rotary encoder.
     * Returns -1 when rotated counter clockwise
     * Returns 1 when clockwise
     * Returns 0 when stationary
     */
    int getRotation()
    {
      currentStateCLK = digitalRead(clkPin);
      int result = 0;
      
      // If last and current state of clkPin are different, then pulse occurred
      // React to only 1 state change to avoid double count
      if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    
        // If the dtPin state is different than the clkPin state then
        // the encoder is rotating CCW
        if (digitalRead(dtPin) != currentStateCLK) {
          result = -1;
        } else {
          result = 1;
        }
      }
    
      // Remember ROTARY_CLK state
      lastStateCLK = currentStateCLK;
    
      return result;
    }
    
    boolean isButtonPressed()
    {
      // Read the button state
      int btnState = digitalRead(swPin);
      boolean result = false;
      
      //If we detect LOW signal, button is pressed
      if (btnState == LOW) {
        //if 50ms have passed since last LOW pulse, it means that the
        //button has been pressed, released and pressed again
        if (millis() - lastButtonPress > 50) {
          result = true;
        }
    
        // Remember last button press event
        lastButtonPress = millis();
      }
    
      return result;
    }
};



// ----------------------------------
// Global variables
// ----------------------------------

// 13 KHz
unsigned long frequencyHz1 = 13000;
int dutyCycleFrequency1Percent = 10;
Timer1 timer1;

// 13 KHz
unsigned long frequencyHz2 = 13000;
int dutyCycleFrequency2Percent = 30;
Timer2 timer2;

RotaryEncoder rotary;
// Rotary encoder will adjust freq2 duty cycle on boot
// Press the button to switch to freq1 
int adjustmentSelect = FREQ_2_SELECT;

// ----------------------------------
// Setup
// ----------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("init.");
  
  // Disable interrupts
  noInterrupts(); 

  timer1 = Timer1(frequencyHz1, dutyCycleFrequency1Percent);
  timer1.pwmMode();
  
  timer2 = Timer2(frequencyHz2, dutyCycleFrequency2Percent);
  timer2.pwmMode();

  rotary = RotaryEncoder(ROTARY_CLK, ROTARY_DT, ROTARY_SW);
  rotary.setupInputs();

  // Re-enable interrupts
  interrupts();
}

// ----------------------------------
// Interrupts
// ----------------------------------

/**
 * Timer 1 counter overflow vector Interrupt for Timer1 (if running in normal mode)
 */
ISR(TIMER1_OVF_vect)
{
  timer1.interrupt();
}

/**
 * Timer 2 counter overflow vector Interrupt for Timer2 (if running in normal mode)
 */
ISR(TIMER2_OVF_vect)
{
  timer2.interrupt();
}

// ----------------------------------
// Loop
// ----------------------------------

void loop() {
  int rotation = rotary.getRotation();

  if (rotation != 0) {
    if (adjustmentSelect == FREQ_1_SELECT) {
      dutyCycleFrequency1Percent = min(100, max(0, dutyCycleFrequency1Percent + (rotation * 4)));
      Serial.print("Frequency 1 duty cycle %: ");
      Serial.println(dutyCycleFrequency1Percent);
      timer1.adjustDutyCycle(dutyCycleFrequency1Percent);
    }
    else {
      dutyCycleFrequency2Percent = min(100, max(0, dutyCycleFrequency2Percent + (rotation * 4)));
      Serial.print("Frequency 2 duty cycle %: ");
      Serial.println(dutyCycleFrequency2Percent);
      timer2.adjustDutyCycle(dutyCycleFrequency2Percent);
    }
  }
  
  if (rotary.isButtonPressed()) {
    if (adjustmentSelect == FREQ_1_SELECT) {
      adjustmentSelect = FREQ_2_SELECT;
    }
    else {
      adjustmentSelect = FREQ_1_SELECT;
    }
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}
