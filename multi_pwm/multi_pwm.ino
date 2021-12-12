#define CLOCKSPEED 16000000
#define TIMER_OPERATION_MODE_NORMAL 1
#define TIMER_OPERATION_MODE_PWM 2

// ----------------------------------
// Class definitions
// ----------------------------------

class Timer {
  protected:
    /**
     * Wave frequency in Herz. 1-CLOCKSPEED
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
    unsigned int counterStart;

    /**
     * In pwm mode, counter starts at 0 and uses CTC mode to compare the counter 
     * when it hits this top
     */
    unsigned int counterTop;

    /**
     * This timer's operation mode
     */
    unsigned int operationMode;

  public: 
    Timer() {}
    
    Timer(unsigned long frequencyHz) {
      this->frequencyHz = max(0, min(CLOCKSPEED, frequencyHz));
      this->dutyCycle = 100;
    }
    
    Timer(unsigned long frequencyHz, int dutyCycle) {
      this->frequencyHz = max(0, min(CLOCKSPEED, frequencyHz));
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
      

      // Set Mode 14: Fast PWM, ICR1 TOP, Update of OCR1x at BOTTOM, TOV Flag set on TOP
      // WGM11 is set at TCCR1A; WGM12 and WGM13 are set at TCCR1B 
      // (See https://wolles-elektronikkiste.de/en/timer-and-pwm-part-2-16-bit-timer1 : Timer/Counter1 Control Register TCCR1x)
      
      // Clear OC1A/OC1B on Compare Match / Set OC1A/OC1B at Bottom (COM1A1)
      // + Set waveform generation mode bit
      TCCR1A = (1<<COM1A1) + (1<<WGM11);

      // Set pre-scaler bits
      // + Set waveform generation mode bits
      TCCR1B = getPreScalerBitSettings(preScaler) + (1<<WGM13) + (1<<WGM12);

      Serial.print("PWM TOP value 1: ");
      Serial.println(counterTop);
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
     * Get the appropriate prescaler for timer 1
     */
    uint32_t getPreScaler(uint32_t frequency) {
      if (1024 * frequency < CLOCKSPEED) {
        return 1024;
      }
      if (256 * frequency < CLOCKSPEED) {
        return 256;
      }
      if (64 * frequency < CLOCKSPEED) {
        return 64;
      }
      if (8 * frequency < CLOCKSPEED) {
        return 8;
      }
      return 1;
    }
    
    /**
     * In normal mode,  start-16000000 is used
     */
    uint32_t getCounterStart(uint32_t frequency, int preScaler) {
      float start = CLOCKSPEED / (preScaler * frequency);
      return 65536 - round(start);
    }

    /**
     * In PWM mode, 0-top is used
     */
    uint32_t getCounterTop(uint32_t frequency, int preScaler) {
      float top = CLOCKSPEED / (preScaler * frequency);
      
      return round(top) - 1;
    }

    /**
     * In PWM mode, the duty cycle is a fraction of the pulse width (0-counterTop)
     */
    uint32_t getDutyCycleSteps(uint32_t counterTop, int dutyCycle) {
      float step = counterTop * dutyCycle / 100; 
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
      
      preScaler = getPreScaler(frequencyHz);
      Serial.print("Prescaler value 2: ");
      Serial.println(preScaler);
    
      slowerFactor = getTimer2SlowerFactor(preScaler);
      Serial.print("Slower factor: ");
      Serial.println(slowerFactor);
      
      // Compare Output Mode Bits: OC2A OC2B disconnected on Normal Port Operation Mode
      TCCR2A = 0;
    
      // Clear TCCR2B
      TCCR2B = 0;
    
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
    uint32_t getPreScaler(uint32_t frequency) {
      int slowerFactor = getTimer2SlowerFactor(1024);
      if (getCounterStart(frequency, 1024, slowerFactor) > 0) {
        return 1024;
      }
      slowerFactor = getTimer2SlowerFactor(256);
      if (getCounterStart(frequency, 256, slowerFactor) > 0) {
        return 256;
      }
      slowerFactor = getTimer2SlowerFactor(128);
      if (getCounterStart(frequency, 128, slowerFactor) > 0) {
        return 128;
      }
      
      slowerFactor = getTimer2SlowerFactor(64);
      if (getCounterStart(frequency, 64, slowerFactor) > 0) {
        return 128;
      }
      
      slowerFactor = getTimer2SlowerFactor(32);
      if (getCounterStart(frequency, 32, slowerFactor) > 0) {
        return 32;
      }
      
      slowerFactor = getTimer2SlowerFactor(8);
      if (getCounterStart(frequency, 8, slowerFactor) > 0) {
        return 8;
      }
      return 1;
    }

    /**
     * Get the appropriate prescaler for timer 2 in PWM mode
     */
    uint32_t setBestPwmPreScalerAndTop(uint32_t frequency, int dutyCycle) {
      float preScalers[] = {1, 8, 32, 64, 128, 256, 1024};

      float bestPreScaler = 1;
      float bestError = 2;
      float bestTop = 255;
      
      float error;
      float freq;

      for (byte i = 0; i < 7; i++) {
        for (float top = 0; top < 256; top++) {
          freq = ((float) CLOCKSPEED) / (preScalers[i] * (top + 1));
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
    int getCounterStart(uint32_t frequency, uint32_t preScaler, uint32_t slowerFactor) {
      float start = CLOCKSPEED / (frequency * preScaler * slowerFactor);
      return 256 - round(start);
    }

    /**
     * Get a value less than half of 256 to get a "good value" for counter start
     * Not really sure what a good value is. 
     * Look at https://wolles-elektronikkiste.de/en/timer-and-pwm-part-1-8-bit-timer0-2
     * under "Setting an exact frequency"
     */
    uint32_t getTimer2SlowerFactor(uint32_t preScaler) {
      float factor = CLOCKSPEED / (preScaler * 128);
      return round(factor);
    }

    /**
     * In PWM mode, the duty cycle is a fraction of the pulse width (0-counterTop)
     */
    uint32_t getDutyCycleSteps(uint32_t counterTop, int dutyCycle) {
      float step = counterTop * dutyCycle / 100; 
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

// ----------------------------------
// Global variables
// ----------------------------------

// 4 Hz
unsigned long frequencyHz1 = 61;
Timer1 timer1;


// 1 Hz
unsigned long frequencyHz2 = 61;
Timer2 timer2;

// ----------------------------------
// Setup
// ----------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("init.");
  
  // Disable interrupts
  noInterrupts(); 

  //timer1 = Timer1(frequencyHz1, 10);
  //timer1.pwmMode();
  
  timer2 = Timer2(frequencyHz2, 20);
  timer2.pwmMode();

  // Re-enable interrupts
  interrupts();
}

// ----------------------------------
// Interrupts
// ----------------------------------

/**
 * Timer 1 counter overflow vector Interrupt for Timer1
 */
ISR(TIMER1_OVF_vect)
{
  timer1.interrupt();
}

/**
 * Timer 2 counter overflow vector Interrupt for Timer2
 */
ISR(TIMER2_OVF_vect)
{
  timer2.interrupt();
}

// ----------------------------------
// Loop
// ----------------------------------

void loop() {
}
