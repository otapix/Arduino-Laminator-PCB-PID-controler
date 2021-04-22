
#include <PID_v1.h>
// config ///////////////////////////////////////////////////////////

#define use_display_and_temp_config 1 // jezeli chcesz uzywac wyswietlacza i konfiguracji temperatury
#define use_serial                  1 // jezeli chcesz uzywac seriala
#define use_serial_pid_tune         1 // jezeli chcesz uzywac seriala do ustawiania parametrow PID
#define use_cooling_switch_button   1 // 1 - jezeli chcesz uzywac przelacznika do wlaczenie chlodzenia,
                                      // 0 - jezeli chcesz uzyc przycisku do wlaczenie chlodzenia

#define MAX6675_SO                  5  //  9 // PD5
#define MAX6675_CS                  6  // 10 // PD6
#define MAX6675_SCK                 7  // 11 // PD7

#define MOTOR_PIN                   3  //  1 // PD3
#define MOTOR_SPEED_SET_PIN         A1 // 24 // PC1

#define MOTOR_TIME_ON               200      // ms > 0
#define MOTOR_TIME_OFF_MIN          100      // ms > 0
#define MOTOR_TIME_OFF_MAX          8000     // ms > 0

#define HEATER_PIN                  4  //  2 // PD4
#define HEATER_COOLING_PIN          2  // 32 // PD2

#define HEATER_TEMP                 170      // *C
#define HEATER_TEMP_MIN             20       // *C
#define HEATER_TEMP_MAX             190      // *C

#if use_serial
    #define SERIAL_BAUD             115200
#endif

#if use_display_and_temp_config
    #include <EEPROM.h>
    #include <TM1637Display.h>
    #define DISPLAY_BRIGHTNES       2  // 0 - 7
    #define DISPLAY_CLK             A4 // 27 // PC4   // Set the CLK pin connection to the display
    #define DISPLAY_DIO             A5 // 26 // PC5   // Set the DIO pin connection to the display
    TM1637Display display(DISPLAY_CLK, DISPLAY_DIO);
#endif

// config ///////////////////////////////////////////////////////////

uint16_t heater_temp                = HEATER_TEMP;
uint16_t motor_time_on              = MOTOR_TIME_ON;
uint16_t motor_time_off             = MOTOR_TIME_OFF_MIN;
uint16_t motor_time                 = motor_time_off;
bool motor_state                    = LOW;
bool heater_state                   = LOW;
bool heater_cooling_state           = LOW;

uint16_t WindowSize                 = 1000;
// bool pOn = P_ON_M;                   // PID control mode
bool pOn = P_ON_E;
double Kp=15.0, Ki=0.06, Kd=0.0;        // PID tuning
double Setpoint, Input, Output;

PID PIDtemp(&Input, &Output, &Setpoint, Kp, Ki, Kd, pOn, DIRECT);


/// functions definition ////////////////////////////////////////////////
double readMAX6675() {
    static uint16_t v = 0;
    static uint32_t update_temp_time;
    if(millis() - update_temp_time >= 200) {
        update_temp_time = millis();
        pinMode(MAX6675_CS, OUTPUT);
        pinMode(MAX6675_SO, INPUT);
        pinMode(MAX6675_SCK, OUTPUT);
        digitalWrite(MAX6675_CS, LOW);
        _delay_ms(1);
        v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
        v <<= 8;
        v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
        digitalWrite(MAX6675_CS, HIGH);
        if (v & 0x4) { // Bit 2 indicates if the thermocouple is disconnected
            return -1;
        }
        v >>= 3;  // The lower three bits (0,1,2) are discarded status bits
    }
    return v*0.25;  // The remaining bits are the number of 0.25 degree (C) counts
}


#if use_display_and_temp_config
    void updateDisplay(int value) {
        uint16_t interval = 250;
        static uint32_t displayTime = 0;
        if(millis() - displayTime >= interval) {
            displayTime = millis();
            // display.showNumberDecEx(value, 0, false, 3, 1);
            display.showNumberDec(value, false, 3, 1);
        }
    }

    void updateDisplayState(void) {
        uint8_t sate[] = { 0x00 };
        if(motor_state){
            sate[0] = 0b00001000;
        }
        if(heater_state){
            sate[0] = 0b00000001;
        }
        if(heater_state && motor_state){
            sate[0] = 0b00001001;
        }
        if(heater_cooling_state){
            sate[0] = 0b01011000;
        }
        display.setSegments(sate, 1, 0);
    }

    void setupHeatingTemp(void) {
        EEPROM.get(0, heater_temp);
        if(digitalRead(HEATER_COOLING_PIN)) {
            uint8_t s[] = { 0b01110100, 0b00000000 };
            uint8_t state[] = {s[0]};
            display.showNumberDec(heater_temp, false);
            display.setSegments(state, 1, 0);
            uint32_t update_heating_temp = millis();
            int16_t t1 = analogRead(MOTOR_SPEED_SET_PIN);
            while(millis() - update_heating_temp < 5000) {
                int16_t t2 = analogRead(MOTOR_SPEED_SET_PIN);
                if((t2 > t1 + 100) || (t2 < t1 - 100)) {
                    bool x = 1;
                    while(millis() - update_heating_temp < 15000) {
                        heater_temp = analogRead(MOTOR_SPEED_SET_PIN);
                        heater_temp = map(heater_temp, 0, 1023, HEATER_TEMP_MIN, HEATER_TEMP_MAX);
                        x = !x;
                        state[0] = s[x];
                        display.showNumberDec(heater_temp, false);
                        display.setSegments(state, 1, 0);
                        _delay_ms(150);
                    }
                    EEPROM.put(0, heater_temp);
                    state[0] = s[0];
                    display.showNumberDec(heater_temp, false);
                    display.setSegments(state, 1, 0);
                    _delay_ms(2000);
                }
                _delay_ms(100);
            }
        }
    }
#endif


#if use_serial && use_serial_pid_tune

  /********************************************
   * Serial Communication functions / helpers
   ********************************************/
  union {                // This Data structure lets
    byte asBytes[24];    // us take the byte array
    float asFloat[6];    // sent from processing and
  }                      // easily convert it to a
  foo;                   // float array

  void SerialReceive(){
    int index=0;
    byte Auto_Man = -1;
    byte Direct_Reverse = -1;
    while(Serial.available()&&index<26)
    {
      if(index==0) Auto_Man = Serial.read();
      else if(index==1) Direct_Reverse = Serial.read();
      else foo.asBytes[index-2] = Serial.read();
      index++;
    }

    if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
    {
      Setpoint=double(foo.asFloat[0]);
      //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
                                            //   value of "Input"  in most cases (as
                                            //   in this one) this is not needed.
      if(Auto_Man==0)                       // * only change the output if we are in
      {                                     //   manual mode.  otherwise we'll get an
        Output=double(foo.asFloat[2]);      //   output blip, then the controller will
      }                                     //   overwrite.

      double p, i, d;                       // * read in and set the controller tunings
      p = double(foo.asFloat[3]);           //
      i = double(foo.asFloat[4]);           //
      d = double(foo.asFloat[5]);           //
      PIDtemp.SetTunings(p, i, d);            //

      if(Auto_Man==0) PIDtemp.SetMode(MANUAL);// * set the controller mode
      else PIDtemp.SetMode(AUTOMATIC);             //

      if(Direct_Reverse==0) PIDtemp.SetControllerDirection(DIRECT);// * set the controller Direction
      else PIDtemp.SetControllerDirection(REVERSE);          //
    }
    Serial.flush();                         // * clear any random data from the serial buffer
  }

  void SerialSend(){
    Serial.print("PID ");
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.print(PIDtemp.GetKp());
    Serial.print(" ");
    Serial.print(PIDtemp.GetKi());
    Serial.print(" ");
    Serial.print(PIDtemp.GetKd());
    Serial.print(" ");
    if(PIDtemp.GetMode()==AUTOMATIC) Serial.print("Automatic");
    else Serial.print("Manual");
    Serial.print(" ");
    if(PIDtemp.GetDirection()==DIRECT) Serial.println("Direct");
    else Serial.println("Reverse");
    Serial.println();
  }

#endif


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


// setup ///////////////////////////////////////////////////////////
void setup() {
    analogReference(DEFAULT);

    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(HEATER_COOLING_PIN, INPUT_PULLUP);

    digitalWrite(MOTOR_PIN, motor_state);
    digitalWrite(HEATER_PIN, heater_state);

    #if use_serial
        Serial.begin(SERIAL_BAUD);
    #endif

    #if use_display_and_temp_config
        display.setBrightness(DISPLAY_BRIGHTNES);

        setupHeatingTemp();
    #endif

    Setpoint = heater_temp;

    PIDtemp.SetOutputLimits(0, WindowSize);
    PIDtemp.SetSampleTime(250);
    PIDtemp.SetMode(AUTOMATIC);
}

// main ///////////////////////////////////////////////////////////
void loop() {
    Input = readMAX6675();

    #if use_cooling_switch_button
        heater_cooling_state = !digitalRead(HEATER_COOLING_PIN);
    #else
        if(!digitalRead(HEATER_COOLING_PIN)) {
            heater_cooling_state = !heater_cooling_state;
            while(!digitalRead(HEATER_COOLING_PIN)) {
                _delay_ms(50);
            }
        }
    #endif

    if(heater_cooling_state) {
        // cooling /////////////////////////////////////////////////////////
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(MOTOR_PIN, HIGH);
    } else {
        // motor ///////////////////////////////////////////////////////////
        motor_time_off = analogRead(MOTOR_SPEED_SET_PIN);
        motor_time_off = map(motor_time_off, 0, 1023, 0, MOTOR_TIME_OFF_MAX);

        if(motor_state) {
            motor_time = motor_time_on;
        } else {
            motor_time = motor_time_off;
        }

        static uint32_t update_motor_state = 0;
        if(millis() - update_motor_state >= motor_time) {
            update_motor_state = millis();
            motor_state = !motor_state;
            if(motor_time_off < MOTOR_TIME_OFF_MIN && !motor_state) {
                motor_time = 0;
                motor_state = !motor_state;
            }
            digitalWrite(MOTOR_PIN, motor_state);
        }

        // heater ///////////////////////////////////////////////////////////
        PIDtemp.Compute();

        static uint32_t WindowStartTime = millis();
        if (millis() - WindowStartTime > WindowSize) {
            WindowStartTime += WindowSize;
        }
        if (Output < millis() - WindowStartTime) {
            heater_state = LOW;
        } else {
            heater_state = HIGH;
        }
        digitalWrite(HEATER_PIN, heater_state);
    }

    // display ///////////////////////////////////////////////////////////
    #if use_display_and_temp_config
        updateDisplay((int)Input);
        updateDisplayState();
    #endif

    // serial ///////////////////////////////////////////////////////////
    #if use_serial
        static uint32_t update_serial_time;
        if(millis() - update_serial_time >= 500) {
            update_serial_time = millis();
            #if use_serial_pid_tune
                SerialReceive();
                SerialSend();
            #else
                Serial.print(Setpoint);
                Serial.print(",");
                Serial.print(Input);
                Serial.print(",");
                Serial.print(heater_state);
                Serial.print(",");
                Serial.println(motor_state);
            #endif
        }
    #endif
}
