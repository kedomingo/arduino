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
    // virtual void pwmMode();

  protected:
    virtual void interruptNormal();
    // virtual void interruptPwm();
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
      
      preScaler = getPreScalerForFrequency(frequencyHz);
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
    uint32_t getPreScalerForFrequency(uint32_t frequency) {
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
    
    
    uint32_t getCounterStart(uint32_t frequency, int preScaler) {
      float start = CLOCKSPEED / (preScaler * frequency);
      return 65536 - round(start);
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
      
      preScaler = getPreScalerForFrequency(frequencyHz);
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
     * Get the appropriate prescaler for timer 2
     */
    uint32_t getPreScalerForFrequency(uint32_t frequency) {
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
unsigned long frequencyHz1 = 4;
Timer1 timer1;


// 1 Hz
unsigned long frequencyHz2 = 1;
Timer2 timer2;

// ----------------------------------
// Setup
// ----------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("init.");
  
  // Disable interrupts
  noInterrupts(); 

  timer1 = Timer1(frequencyHz1);
  timer2 = Timer2(frequencyHz2);

  timer1.normalMode();
  timer2.normalMode();

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
