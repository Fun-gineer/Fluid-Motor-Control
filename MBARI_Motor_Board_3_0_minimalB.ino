/*
  Motor Control using Dual H-Bridge DRV8833 motor driver and motor control Boards for MBARI device and pump.

  This routine allows a two motor controlled hardware components, a pump and a devicec to operate simultaneously with marginal interference with one another.

  Uses PWM control of motor speed and direction

  commands are passed through Serial communication as seen in the input file.

  debug commands are printed using the DebugUtils.h defined commands
*/
#include "definitions.h" //define what communication and devices to use
#include "PinChangeInt.h" //used for channel B on a non interrupt pin

//include I2C library for communication
#ifdef I2C
  #include <Wire.h> //400kbaud communication on Raspberry Pi
#endif


//Serial Output flags for debugging
//#include <DebugUtils.h> //if included in actual library
//#define DEBUG
#include "DebugUtils.h" //if included in sketch folder
//Initialize variables for all components

unsigned long time2 = 0, timeSleep = 0, timeCalcDelta = 3, timePrint = 0, timePrintDelta = 100;
boolean sleepState, overloadFlag = LOW; //turn on H-Bridge
//boolean firstTimeFlag=true; // This is the flag set to determine first time run

float oscillationTime; // Used for oscillation of syringe pumps
int numberOfOscillations, oscFlag = 0;
unsigned long nextUpdate;      //time (in milliseconds) after which we should do something. Found by adding some offset to the current time. This determines the rate at which updates are issued to the boards.

#ifdef pump

    #define motorB  //define which motor it is controlled by
    #define pi 3.14159265359


    //Define the constants to be used for each pump type

    float demand, demandi, sign;

    #ifdef pump_441541
      double error = 0, errorOld, errorFollowing = 1000, derivative, integral, pGain = 0.4, dGain = 0.5, iGain = 5.0e-3;
      #ifdef pitch0_5
        double pitch = 0.5, slope = 5.21, offset = 7.75; // 8V supply on motor control board
      #endif
      #ifdef pitch2_54
        double pitch = 2.54, slope = 0.982, offset = 8.38; // 8V supply on motor control board
      #endif
      #ifdef pitch4_76
        double pitch = 4.76, slope = 0.533, offset = 10.3; // 8V supply on motor control board
      #endif
      double gearRatio = 63950067.0 / 21125.0, cpt = 128.0, diam = 3.0 / 8.0 * 25.4, area = pi * diam * diam / 4, factor = gearRatio * cpt / (pitch*area);
      float volLim = 1100.00;
    #endif //pump_441541


    #ifdef pump_484563
      double error = 0, errorOld, errorFollowing = 2000, derivative, integral, pGain = 3.0, dGain = 0.5, iGain = 5.0e-3;
      #ifdef pitch0_5
        double pitch = 0.5, slope = 5.21, offset = 7.75; // 8V supply on motor control board
      #endif
      #ifdef pitch2_54
        double pitch = 2.54, slope = 0.25, offset = 9.7; // 8V supply on motor control board
      #endif
      #ifdef pitch4_76
        double pitch = 4.76, slope = 0.533, offset = 10.3; // 8V supply on motor control board
      #endif
      double gearRatio = 135005697.0 / 54925.0, cpt = 32.0 * 4.0, diam = 14.1, area = pi * diam * diam / 4, factor = gearRatio * cpt / (pitch*area); //2458:1 gear ratio
      float volLim = 2900.00;
    #endif //pump_484563


    #ifdef pump_487923
      double error = 0, errorOld, errorFollowing = 2000, derivative, integral, pGain = 2.0, dGain = 1.0, iGain = 5.0e-3;
      #ifdef pitch0_5
        double pitch = 0.5, slope = 5.21, offset = 7.75; // 8V supply on motor control board
      #endif
      #ifdef pitch2_54
        double pitch = 2.54, slope = .198, offset = 25.0; // 8V supply on motor control board
      #endif
      #ifdef pitch4_76
        double pitch = 4.76, slope = 0.533, offset = 10.3; // 8V supply on motor control board
      #endif
      double gearRatio = 601692057.0 / 537824.0, cpt = 128.0 * 2.0, diam = 7.0 / 16.0 * 25.4, area = pi * diam * diam / 4, factor = gearRatio * cpt / (pitch*area); //1119:1 gear ratio
      float volLim = 1500.00;
    #endif//pump_487923
    int outputb = 0, outputOld = 0, outputi, outputError = 45;
    byte Fault;
    boolean refillState = LOW;

    //define variables for volume and time calculations
    float rem_volb, vol, voli, volDis, volDelta, volDeltaPrevious, posLim = volLim * factor, rate, frate, refill_rateb, frateMax = floor(((250 - outputError) - offset) / slope);
    unsigned long time, timei, timePrev, timeDelta, timeDeltaOverload, timeOverload, timeCalc = 0;

    //Define variables for encoder interrupt
    volatile float pos = 0, skip = 0;
    volatile boolean A, B, Aold, Bold, Aswitch, Bswitch;
    volatile boolean Dir;
#endif //pump


#ifdef peristaltic
  #define motorA
  uint32_t injectTime = 10500, cleanTime = 20000, timep, timecalca = 0; //duration to run peristaltic for injection and cleaning based on 125PWM at 8V and 8" Length x 0.01" ID tubing and 5uL injection
  uint32_t addTime[4] = {0, 0, 4000, 10000}; //adjust peristaltic inject time for different volumes
  int injectSpeed = 125, cleanSpeed = 255, outputa; //PWM value for injection at 8V
  boolean signa;
#endif


#ifdef device
  #define motorA
  double degree, motorPosition, motorPositioni, errora, motorPositionLast, deltaPosition;   //Define variables for use in analog encoder motor control
  int sensorAvgCount = 3;        //Number of times to average analog position sensor
  double overShootVal = 0.2;      //Overshoot Allowed
  unsigned long timeCalca = 0, timeai;   //calculate A motor at same frequency but slight offset from B motor
  int maxMotorSpeed = 250, minMotorSpeed = 150, outputa, outputaLast; //Define motorspeed min and max for rotating devices
  boolean signa;
  double rand_posa; //variable for holding the arbitratry injector position
  double fixed_posa; //variable for holding the fixed injector position
#endif


#ifdef injector
  //for injector
  int counts = 4; //number of different positions
  int dist;
  //double degreeStep = 360.0 / counts; //determine degree spacing between positions
  //double degreeOffset[4] = {0, -3, -1.5, +1};
  double initial = 89; //define initial position for analog encoder controlled device (degrees) - Added 25 degrees on 3/18/16
  double posa[4] = {initial, initial + 90.0, initial + 180, initial + 271}; //define position array - Added 4 degrees to 270 on 3/18/16
  #ifdef printSerial
    char pos1[] = "2.5 uL", pos2[] = "5 uL", pos3[] = "10 uL", pos4[] = "25 uL";
    char* action[4] = {pos1, pos2, pos3, pos4}; //define character matrix for print
  #endif
  int homePos = 0; //define which position is home
  int pcount = homePos; //initialize position counter
#endif


#ifdef coalescer
  //for silo coalescer
  // 98 deg = in1, 158 deg = in2 338 deg = outlet
  int counts = 3;
  int dist;
  //double degreeStep = 360.0 / counts; //determine degree spacing between positions
  //double degreeOffset[3] = {0, 0, 0}; //todo: swtich 2nd to -60.0
  //double initial = 93; //define initial position for analog encoder controlled device (degrees)
  double posa[3] = {0, 120, 180};
  char pos1[] = "Sample Inlet", pos2[] = "Cassette Inlet", pos3[] = "Outlet";
  char* action[3] = {pos1, pos2, pos3};
  int homePos = 1; //define which position is inlet 1
  int pcount = homePos; //initialize position counter
#endif


#ifdef mixer
  // 60 deg = in1, 120 deg = in2 180 deg = in3, 240 deg = Out, 300 = coalesce, 360 = Inject line
  int counts = 6;
  int dist;
  //double degreeStep = 360.0 / counts; //determine degree spacing between positions
  //double degreeOffset[6] = {0};
  //double initial = 16.0; //define initial position for analog encoder controlled device (degrees)
  double posa[6] = {0}; //initialize "posa", the rest is defined in Setup()
  int homePos = 0; //define which position is inlet 1
  int pcount = homePos; //initialize position counter
#endif


#ifdef cassette
  //for cassette
  int counts = 4; //number of different positions
  int dist;
  double degreeStep = 360.0 / counts; //determine degree spacing between positions
  double degreeOffset[4] = {0, 0, 0, 0};
  double initial = 57.0; //define initial position for analog encoder controlled device (degrees)
  double posa[4] = {0, 90.0, 180.0, 270.0}; //define position array
  #ifdef printSerial
    char pos1[] = "22 uL", pos2[] = "5 uL", pos3[] = "7.2 uL", pos4[] = "11.3 uL";
    char* action[4] = {pos1, pos2, pos3, pos4}; //define character matrix for print
  #endif
  int homePos = 0; //define which position is home
  int pcount = homePos; //initialize position counter
#endif //cassette


#ifdef valve
  #define motorA
  unsigned long timev = time2, timevi = 0, timevDelta = 75; //define time for valve movement
  double valvePos = 0, valvePosPrev = 0; //define variables for tracking position
  boolean valveSign = LOW;       //define inlet HIGH, outlet LOW;
  int sensorAvgCount = 3;        //Number of times to average analog position sensor
  int maxMotorSpeed = 255, minMotorSpeed = 150, outputa; //Define motorspeed min and max for rotating devices
  boolean signa = HIGH;
#endif //valve

#ifndef fmapadjust
  #define fmapadjust 0 //correct for errors in encoder output range
#endif //fmapadjust

/*============End Device Selection ==========*/

// Variables for I2C Communication
#ifdef I2C
  boolean flag = false;
  char buf[20] = {0};
  char err_status[3] = {'0', '0', '\n'};

  // Flag for telling the Arduino that the different values of temperature zones are ready now
  // So the next time a GT command comes on the I2C, we are ready to send out the values to the Pi
  boolean data_ready = false;
  // can have single buffer to get data of remaining volume
  // Data is in AB.CD format hence, 5 bits for each zone
  // and 1 extra bit for '\n'
  //char rem_vol[8]={'0','0','0','0','0','0','0','\n'};
  char rem_vol[9] = {'0', '0', '0', '0', '0', '0', '0', '0', '\n'};
  char inj_pos[7] = {'0', '0', '0', '0', '0', '0', '\n'};

  char holdVariable[7] = {0}; // Temp variable to store double/float value when reading from Pi
  char stringValue[8] = {0}; //Variable for double to string conversion of data.
#endif
// End Variables for I2C Communication








#include "pins.h" //define all the pins

void setup() {

    // Set up I2C on the Arduino. This setup needs to be before Serial
  #ifdef I2C
    rem_vol[7] = '\n';
    inj_pos[6] = '\n';

    //  convert2String ((volLim-vol));
    //
    //  for(int k=0;k<6;k++) // k iterates from 0 to 5 filling bit 0 to bit 5 into array
    //  {
    //    rem_vol[k]=stringValue[k];
    //  }
    //
    //  rem_vol[6]='\n';

    /* -----------Addresses for boards---------
      Temperature Control Board              == 04
      Injector + PCR injector oil syringe    == 06
      Droplet Generation oil syringe         == 08
      Peristaltic + Separation oil syringe   == 09
      LED_PMT board                          == A
    */

    Wire.begin(Address);                // join i2c bus with address #5
    Wire.onReceive(receiveEvent); // register event (ISR) for receiving I2C data
    Wire.onRequest(sendevent);    // register event (ISR) for sending I2C data
  #endif
    // End I2C setup

    // Start Serial connection
  #ifdef printSerial
    Serial.begin(115200);
    Serial.setTimeout(50); //times out parseInt() command faster
  #endif

    //define output pin for HBridge Sleep
    pinMode(nSLEEP, OUTPUT);
    digitalWrite(nSLEEP, sleepState); //enable H-bridge motor controller

    //define LDO output pin for 5v encoder control, control with sleepState
    pinMode(nEncoder, OUTPUT);
    digitalWrite(nEncoder, HIGH); //enable 5V LDO for encoder power supply
    delay(1000);

  #ifdef pump
      //define input pin for limit switch
      //pinMode(limitPin1, INPUT);
      //digitalWrite(limitPin1,HIGH); //use pullup resistor.

      //interrupt for tracking motor position
    #ifdef pump_441541
      attachInterrupt(0, countb, RISING); //tracks channel A pin Rise on interrupt 0
      //attachInterrupt(1,countbB,CHANGE); //tracks channel B pin change on interrupt 1
    #endif

    #ifdef pump_484563
      #define Change //use pin change on both interrupts
      attachInterrupt(0, countbA, CHANGE); //tracks channel A pin change on interrupt 0
      PCintPort::attachInterrupt(encoderBPinB, &countbB, CHANGE); //tracks channel B pin change on arbitrary pin interrupt
    #endif

    #ifdef pump_487923
      attachInterrupt(0, countbA, RISING); //tracks channel A pin change on interrupt 0
      PCintPort::attachInterrupt(encoderBPinB, &countbB, RISING); //tracks channel B pin change on arbitrary pin interrupt
    #endif
  #endif

  #ifdef motorA
    //define output pins for H-Bridge motor control
    pinMode(HBridgeAPin1, OUTPUT);
    pinMode(HBridgeAPin2, OUTPUT);

    //configure input pins for analog encoders
    pinMode(ADC0, INPUT);
    pinMode(ADC1, INPUT);

    //set PWM Timer 2 clock speed to higher speed for
    TCCR2B = TCCR2B & 0b11111000 | 0x01; //31,372.55 Hz

    //initializea(homePos); //move device to home position at startup
  #endif

  #ifdef motorB
    //Define input pins for encoder
    pinMode(encoderBPinA, INPUT);
    pinMode(encoderBPinB, INPUT);

    //define output pins for H-Bridge motor control
    pinMode(HBridgeBPin1, OUTPUT);
    pinMode(HBridgeBPin2, OUTPUT);

    //set PWM Timer 1 clock speed to higher speed for pins 9 & 10
    TCCR1B = TCCR1B & 0b11111000 | 0x01; //31,372.55 Hz
  #endif

    //Tathagata, I moved this from the loop to the setup but I think we should move this to an initialize routine! which we call here.
    // If injector is active, when the Arduino starts get the current injector position to update RPi GUI
  #ifdef device
    // Get the current encoder position
    double tempMotorPosition = readSensorValuea(); //get position
    //Update the current injector position buffer to be sent back to the PI
	#ifdef I2C
		convert2String (tempMotorPosition, 6);

		for (int k = 0; k < 6; k++) // k iterates from 1 to 6 filling bit 1 to bit 6 into array
		{
		  inj_pos[k] = stringValue[k];
		}

		inj_pos[6] = '\n';  // setting last bit to be '\n' to easily parse at Pi side.
	 #endif //I2C
  #endif //device

  #ifdef valve
    valvePosPrev = valvePos = analogRead(ADC0);

    if (valvePos > 300) valveSign = HIGH; //define Inlet HIGH; Outlet LOW
    update_rem_volb();
  #endif

  #ifdef pump
    A = bitRead(PIND, encoderBPinA);
    B = bitRead(PIND, encoderBPinB);
    getvol();
  #endif

  #ifdef cassette
    for (int j = 0; j < counts; j++) {
      posa[j] = initial + j * degreeStep + degreeOffset[j];
    }
  #endif

  #ifdef mixer
    for (int j = 0; j < counts; j++) {
      posa[j] = initial + j * degreeStep + degreeOffset[j];
    }
  #endif


}









void loop() {

    // Give highest priority to the I2C commands (only get temperature commands) received.
  #ifdef I2C
    if (flag == true)
    {
      flag = false; //reset the flag
      input_I2C();      // function which checks all the received get or set commands from the Pi.
      //buf[0]='0';
    }
    // If not received anything, do the normal work.
    else
    {
  #endif //I2C

      // Do whatever the Arduino is supposed to do here...
  #ifdef printSerial
      input();
  #endif //printSerial



  time2 = millis(); //take the current time every loop

  #ifdef motorB
      if (oscFlag) {
        if (numberOfOscillations + 2 > oscFlag ) { // Checks to see if we have oscillated enough times
          if (time2 >= nextUpdate) {            // Checks to see if we have oscillated for long enough
            outputb = 0;
            moveMotorb(outputb);
            getvol();
            if (oscFlag & 0x01) {
              sign = 1.0; Dir = HIGH;
              volDis = (volLim - vol);
              updateb();
            } else {
              refillb();
            }
            nextUpdate = time2 + (oscillationTime * 60000);
            oscFlag++;
          }
        } else {
          frate = 0;
          outputb = 0;
          moveMotorb(outputb);
          getvol();
          oscFlag = 0;
        }
      }
      /*if(oscFlag) {
         check if we've oscillated enough times (how big is oscFlag?)
       	check if we've oscillated for long enough (how big is currentMillis?)
       	change the sign, updateb(), update oscFlag
       	TODO: we'll need to get currentMillis when we first set oscFlag when we receive the "O" command, so we can calculate when we should be switching
        }*/
  #endif //motorB

  #ifdef pump
      if (outputb != 0) {
        if (time2 - timeCalc > timeCalcDelta) { //set frequency for calculations
          timeSleep = time2;
          dprintv("I am here");
          calcb(); //calculate speed and flow parameters
          limitb(); //determine when to stop motor
          timeCalc = time2; //reset calc counter
          #ifdef printSerial
                  if (time2 - timePrint > timePrintDelta) { //output data at 10 Hz
                  #ifdef printSerial
                    printsb(); //output parameters of interest
                  #endif
                    timePrint = time2; //reset print counter
                  }
          #endif //printSerial
        }
      }
  #endif //pump

  #ifdef device
      if (outputa != 0) {
        if (time2 - timeCalca > timeCalcDelta) { //set frequency for calculations
          timeSleep = time2;
          dprintv("I am here");
          limita(); //calculate when to stop motor

          #ifdef pump
              if (outputb != 0) timeCalca = timeCalc + 1; //slight offset from pump timeCalc but same frequency of timeCalcDelta
              else timeCalca = time2; //set it's own frequency if pump not operating
          #else //pump
              timeCalca = time2;
          #endif //pump

          #ifdef printSerial
                  if (time2 - timePrint > timePrintDelta - 5) { //output data at 20 Hz
                    printsa(); //output parameters of interest
                    #ifndef pump
                        timePrint = time2; //reset print counter
                    #else //pump
                        if (outputb == 0) timePrint = time2; //reset print counter
                    #endif //pump
                  }
          #endif //printSerial
        }
      }
  #endif //device

  #ifdef peristaltic
      if (outputa != 0) {
        timeSleep = time2;
        if (time2 > timep) {
          outputa = 0;
          moveMotora(outputa);
        }
      }
  #endif //peristaltic

  #ifdef valve
      if (outputa != 0) {
        if (time2 - timev > timevDelta) { //set frequency for calculations (time2 is updated continually while timev stays the same (timev is the starting time))
          timeSleep = time2;
          valvePos = analogRead(ADC0);
          #ifdef printSerial
                  Serial.print("valve position: "); Serial.print(valvePos);
                  Serial.print(" | Previous valve position: "); Serial.print(valvePosPrev);
                  Serial.print(" | valveSign: "); Serial.println(valveSign);
          #endif //printSerial
          if (valveSign == LOW) {  //defined as inlet HIGH, outlet LOW;
            if (valvePos > valvePosPrev + 3 && valvePos < 200) { //look for sign inversion in direction
              outputa = 0;
              moveMotora(outputa);
              #ifdef printSerial
                  Serial.println("Ready to dispense");
              #endif  //printSerial
              #ifdef I2C
                  update_rem_volb();
                  err_status[0] = '0'; err_status[1] = '0'; //set flag to signify valve stopped moving
              #endif //I2c
            }
            if (valvePosPrev > valvePos) { //update value if current leads previous
              valvePosPrev = valvePos; //track previous valve position
            }
          }
          else { // if valveSign == HIGH
            if (valvePos < valvePosPrev - 3 && valvePos > 400) { //look for sign inversion in direction
              outputa = 0;
              moveMotora(outputa);
              #ifdef printSerial
                  Serial.println("Ready to Refill");
              #endif //printSerial
              #ifdef I2C
                update_rem_volb();
                err_status[0] = '0'; err_status[1] = '0'; //set flag to signify valve stopped moving
              #endif //I2c
            }
            if (valvePos > valvePosPrev) { //update value if previous value trails current value
              valvePosPrev = valvePos; //track previous valve position
            }
          } //else of if (valveSign == LOW)
          timev = time2;
        } //timecalc measurement
        if (time2 > timevi + 15000) {
          outputa = 0;
          moveMotora(outputa);
          #ifdef printSerial
                  Serial.println("error, valve switching timed out");
          #endif //printSerial
          #ifdef I2C
                  err_status[0] = '9'; err_status[1] = '7'; //set device motor stall error flag to be sent to I2C
          #endif
        }
      }//outputa !=0
  #endif //valve

      //put H-Bridge to sleep after x milliseconds of no updates, needs 1ms to wake up
      if (sleepState == HIGH) {
        if (time2 - timeSleep > 10000) {
          sleep();
          #ifdef printSerial
                  Serial.println("--------------Going into sleep mode---------------");
          #endif
        }
      }
  #ifdef I2C
    }
  #endif
}

/*=========================End Loop ===========================*/















//TOGGLES THE VARIABLE sleepState AND WRITES THE NEW OUPUT ON THE SLEEPSTAE PIN nSLEEP
void sleep() {
  sleepState = !sleepState;
  timeSleep = time2; //make sure timeSleep is updated for next cycle
  digitalWrite(nSLEEP, sleepState);
}



// Following functions for the I2C communication
#ifdef I2C

  void receiveEvent(int howMany) {
    int a = 0;

    // set flag here to indicate that an I2C command has been received.
    flag = true;

    while (Wire.available() > 0)
    {
      buf[a] = Wire.read();
      //Serial.println(buf[a]);
      a++;
    }
  }


  void sendevent()
  {
    int k = 0;
    if (strncmp(buf, "T", 1) == 0){
      // check flags and return the error and status flags concatenated together with a '\n' at the end back to the pi.

      // error flags can be set here or in the Loop code depending on how you want to implement.

      // Set err_status[0] to error flag (0 or 1) and err_status[1] to status flag (0 or 1)
      // err_status[2] is ALWAYS '\n'. Already set when initializing. NO NEED TO CHANGE!

      Wire.write(err_status);
    }
    //Check for handshake
    else if (strncmp(buf, "H", 1) == 0) {
      err_status[0] = '0';
      err_status[1] = '0';
      Wire.write("000\n");
    }

    // Check for remaining volume command and send the remaining volume.
    else if (strncmp(buf, "RM", 2) == 0)
    {
      Wire.write(rem_vol);
      //Serial.print("rem_vol =");Serial.println(rem_vol);
  		#ifdef printSerial
    	  dprintv(rem_vol);
      //Serial.print(__FILE__); Serial.print(" : "); Serial.print(__LINE__); Serial.print(" : ");
      //Serial.print(rem_vol); Serial.println(" | remaining volume ");
      //Serial.print(buf); Serial.println(" | buffer");
  		#endif
    }

  	#ifdef device
    // Send the current injector position
    else if (strncmp(buf, "CP", 2) == 0)
    {
      Wire.write(inj_pos);
      if (outputa == 0) {
        err_status[0] = '0'; err_status[1] = '0'; //set flag to signify injector stopped moving
      }
      else {
        err_status[0] = '0'; err_status[1] = '7'; // still moving, keep status flag to reflect that the injector is moving
      }

    }
  	#endif

    else
    {
      // If received command is not get temperature or get status, i.e. it was a set/start/stop command send OK back ASAP!
      //flag=true;
      Wire.write("OK\n");

    }

  }


  // Function to convert character to float/double
  float charToFloat(char *charValue) {
    String strFromChar = String(charValue);
    char floatbuf[10]; // make this at least big enough for the whole string
    strFromChar.toCharArray(floatbuf, sizeof(floatbuf));
    float f = atof(floatbuf);
    return f;

  }


  // Function to convert double/float to character and do the formatting as XX.XX
  void convert2String (double numberVal, int str_len) {

    dtostrf (numberVal, str_len, 2, stringValue);
    //  if (str_len>6){
    //    if (numberVal < 1000.0 && numberVal >= 100.0)
    //     stringValue[0] = '0';
    //    else if (numberVal < 100.0 && numberVal >=10.0){
    //     stringValue[0] = '0';
    //     stringValue[1] = '0';
    //    }
    //    else if (numberVal < 10.0 && numberVal >=1.0){
    //     stringValue[0] = '0';
    //     stringValue[1] = '0';
    //     stringValue[2] = '0';
    //    }
    //    else if (numberVal < 1.0){
    //     stringValue[0] = '0';
    //     stringValue[1] = '0';
    //     stringValue[2] = '0';
    //     stringValue[3] = '0';
    //    }
    //  }
    //  else{
    //   if (numberVal < 1000.0 && numberVal >=100.0){
    //  }
    //  else if (numberVal < 100.0 && numberVal >=10.0){
    //   stringValue[0] = '0';
    //  }
    //  else if (numberVal < 10.0 && numberVal >=1.0){
    //   stringValue[0] = '0';
    //   stringValue[1] = '0';
    //  }
    //  else if (numberVal < 1.0){
    //     stringValue[0] = '0';
    //     stringValue[1] = '0';
    //     stringValue[2] = '0';
    //    }
    //  }
  }

#endif  //end of I2C function declarations





#ifdef pump

  //Perform movement and speed calculations for syringe pump control
  void calcb() {

    timePrev = time;
    time = micros();
    if (time < timePrev) { //account for micros() reset every 70 minutes
		#ifdef printSerial
			Serial.println("micros reset: ");
		#endif
		volDeltaPrevious += volDelta; //for tracking actual volume change
		timei = time; //reset initial time for flow rate tracking
		demandi = demand; //reset initial demand position for flow rate tracking
		voli = vol; //reset initial volume for flow rate tracking
    }
    timeDelta = time - timei; //calculate total time change since pump start
    volDelta = vol - voli; //calculate total volume change since pump start
    vol = pos / factor; //calculate ul based on global position change
    rate = (volDelta) / timeDelta * 6.0e7; //calculate flow rate in ul/min based on volume change since last input time

    //Update the remaining volume buffer to be sent back to the Pi
    #ifdef I2C
      update_rem_volb();
    #endif

    //Determine new demand position
    float volError = abs(volDis - volDelta - volDeltaPrevious);
    demand = demandi + frate * (timeDelta / 6.0e7) * factor * sign; //calculate demand position based on desired flowrate and timeDelta
    if ((volError) < 0.1 && frate > 10) {
      if (sign > 0) {
        demand = constrain(demand, 0, demandi + volDis * factor); //if close, only increment by small amount
      }
      else {
        demand = constrain(demand, demandi + volDis * factor, posLim); //if close, only increment by small amount
      }
    }
    demand = constrain(demand, 0 - 25000, posLim + 25000); //prevent call for position beyond syringe limit

    //Calculate PID parameters
    errorOld = error;  //store the previous error
    error = (demand - pos) * sign; //calculate new error, switch sign depending on desired direction
    derivative = error - errorOld; //positive derivative means speed up, negative derivative means slow down
    integral = integral + error;  //increase integral means speed up, decrease integral means slow down
    integral = constrain(integral, 0, (250.0 / iGain)); //limit integral so iGain*max_integral=250, set iGain range to define responsiveness

    //Compute new PID output and constrain values
    outputb = round(pGain * error + dGain * derivative + iGain * integral);
    outputb = constrain(outputb, 1, 255); //prevent exceeding 8bit PWM value, do not output 0 (stop)
    if ((volError) < 0.1 && frate > 10) outputb = constrain(round(outputb * (volError) * 10), offset + 1, outputb); //slow down when you get closer

    //Determine error Following fault, could perhaps go into limits routine
    if (abs(error) > errorFollowing) {
    #ifdef printSerial
        Serial.println("Following Error");
    #endif
    #ifdef I2C
        err_status[0] = '5'; err_status[1] = '9'; //set following error flag to be sent to I2C
    #endif
        outputb = 0;
    }

    //update output to reduce position following error
    if (outputb != outputOld) { //check if output needs to be updated
      dprintv("updating motor output to correct for speed");
      moveMotorb(outputb);
      outputOld = outputb; //store old value to reduce unecessary motorMove commands
    }

  }



  // FIND OFFSET VALUES NEEDED TO CORRECT FOR MOTOR INNACURACIES
  // - moves motor and calculates slope and offset for speed control
  //   input gearRatio, pitch, piston diameter, encoder cpt
  void characterizeb() {

    dprint("Characterizing");
    float rateCharacterization[2];
    double rates[2] = {30, 200};   //two motor spin rates will be used to characterize
    float slopec = 0, offsetc = 0;    //the two things we're going to find (we're mainly after the correction factor 'offsetc')
    sign = 1;
    //wake up if sleeping
    if (sleepState == LOW) {
      #ifdef printSerial
          Serial.println("---------------Waking up------------");
      #endif
      sleep();
      delay(1);
    }
    //move the motor at both test rates and calculate the actual flow rates...
    for (int j = 0; j < 2; j++) {
      outputb = rates[j];  //the motor speed (expressed as flow rate) being used in the current test
      unsigned long timeOldc = micros();    //get the current time
      float posOldc = pos;    //store the old position
      moveMotorb(outputb);
      delay(2000); //measure for 2 seconds
      float volc = (pos - posOldc) / factor; //calculate ul of flow that occurred based on global position change of the pump ("pos" is changed automatically in the code as the motor spins)
      float timec = micros();   //get the current time again after the 2000ms delay
      unsigned long timeDeltac = timec - timeOldc; //calculate total time elapsed since last update
      float ratec = (volc) / timeDeltac * 6.0e7; //calculate flow rate in ul/min based on volume change since last input time
      rateCharacterization[j] = ratec;   //store the REAL flow rate (as opposed to the fake "guessed" flow rates used in rates[] )
      Serial.print("posstart: ");
      Serial.println(posOldc);
      Serial.print("posend: ");
      Serial.println(pos);
      Serial.print("factor: ");
      Serial.println(factor);
      Serial.print("dv: ");
      Serial.println(volc);
      Serial.print("dt: ");
      Serial.println(timeDeltac);
      Serial.println("dv/dt: ");
      Serial.println(ratec);
    }
    outputb = 0;
    moveMotorb(outputb);
    //calculate slope (correction factor for entered flow rate to actual flow rate)
    //to use "offsetc" you add it to any value for flow rate you would normally enter - it's an additive correction factor
    slopec = (rates[1] - rates[0]) / (rateCharacterization[1] - rateCharacterization[0]);  //the flow rate correction factor (predicted/actual)
    offsetc = (rates[0] - rateCharacterization[0] * slopec + rates[1] - rateCharacterization[1] * slopec) / 2; //calculate offset (the average of two errors between the assumed flow rate and actual flow rate that would need to be entered)
    #ifdef printSerial
      Serial.print("slope | ");
      Serial.println(slopec, 5);
      Serial.print("offset | ");
      Serial.println(offsetc, 5);
    #endif

  }

#endif //pump






//determine shortest path to desired position
#ifdef device

  void closesta() {

    motorPosition = readSensorValuea();
    errora = motorPosition - posa[pcount]; //compare current position to desired position. posa is a constant array containing the predefined angular values of the target positions
    if (abs(errora) > overShootVal) {
      dprintv("moving to position"); dprintv(pcount);
      //find direction to move and set direction
      if ((abs(errora) > 180) ^ (errora < 0)) signa = HIGH;
      else signa = LOW;
      dprintv(signa);
      #ifdef I2C
          // Set the status flags to reflect that the injector is moving
          err_status[0] = '0'; err_status[1] = '7';
      #endif //I2c
        updatea();
    }
    else {
      dprintv("At Position"); dprintv(motorPosition);
      #ifdef I2C
          // Set the status flags to reflect that the injector is at position
          err_status[0] = '0'; err_status[1] = '0';
      #endif //I2C
        motorPositionLast = motorPosition;
      #ifdef printSerial
          printsa();
      #endif //printSerial
    }

  }

#endif //device







//interrupt routing for counts on encoder channel A & B
#ifdef pump
#ifdef pump_441541
  void countb() {
    if (bitRead(PIND, encoderBPinB) == LOW) { //determine direction from comparison of channel B to Channel A on rising condition
      pos++; //increment position
      Dir = HIGH; //define direction
    }
    else {
      pos--;  //decrement position
      Dir = LOW; //define direction
    }
  }
  #else //pump_441541
  //interrupt on A changing state
  #ifdef Change
  void countbA() {
    Aold = A;
    //Bold = B;
    A = bitRead(PIND, encoderBPinA);
    //B = bitRead(PIND, encoderBPinB);
    if (Aold == A || Bswitch == LOW) skip++;
    else if (A == B) {
      pos--, Dir = LOW;
    } else {
      pos++, Dir = HIGH;
    }
    Aswitch = HIGH; Bswitch = LOW; //for debounce
  }
  //interrupt on B changing state
  void countbB() {
    //Aold = A;
    Bold = B;
    //A = bitRead(PIND, encoderBPinA);
    B = bitRead(PIND, encoderBPinB);
    if (Bold == B || Aswitch == LOW) skip--;
    else if (A != B) {
      pos--, Dir = LOW;
    } else {
      pos++, Dir = HIGH;
    }
    Aswitch = LOW; Bswitch = HIGH; //for debounce
  }
  #else //Change

  void countbA() {
    if (bitRead(PIND, encoderBPinB) == LOW) { //determine direction from comparison of channel B to Channel A on rising condition
      pos++; //increment position
      Dir = HIGH; //define direction
    }
    else {
      pos--;  //increment position
      Dir = LOW; //define direction
    }
  }
  void countbB() {
    if (bitRead(PIND, encoderBPinA) == HIGH) { //determine direction from comparison of channel B to Channel A on rising condition
      pos++; //increment position
      Dir = HIGH; //define direction
    }
    else {
      pos--;  //increment position
      Dir = LOW; //define direction
    }
  }
  #endif //Change
#endif //pump_441541
#endif //pump







//Housekeeping routines

void faults() {
  //check for following errors
  //check for limits detected
  //check if initialization failed or previous position not properly saved
  dprint("Fault Detected");
}

void initialize() {
  //get home position from last save state
  //after read, savestate=0, after save, savestate=1;
  //if savestate==0, fault
  //check if need to refill
  //initialize variables
  //set calculation speed, (may require change of PID parameters)
  //
  dprint("Initializing");

}


void save() {
  //save pos to eeprom or send to master controller
  //savestate=1;
  dprint("Saving");
}

//move device to home position
#ifdef device
void initializea(int _position) {

  //initialize variables
  dprintln("initializing");
  pcount = _position;
  dprintv(pcount);
  outputaLast = outputa; //store current position

  closesta();
}
#endif //device


//receive inputs for both device and pump using character, or
#ifdef printSerial

  void input() {

    if (Serial.available() > 0) {
      int inByte = Serial.peek();
      if (isalpha(inByte)) {
        dprintv(char(inByte));

		switch (inByte) {

		  #ifdef device
            case 'r':  //move right 1 count
              pcount++;
              signa = HIGH; //High means move right
              if (pcount > counts - 1) {
                pcount = 0;
              }
              Serial.print("moving right to position"); Serial.print(pcount);
              updatea();
              break;
            case 'l':  //move left 1 count
              pcount--;
              signa = LOW; //LOW means move left
              if (pcount < 0) {
                pcount = counts - 1;
              }
              dprintv("moving left to position"); dprintv(pcount);
              updatea();
              break;
            case 'd':  //define position using command of d and a # with no spaces or special characters  e.g. d0
              pcount = Serial.parseInt();
              pcount = constrain(pcount, 0, counts - 1);
              closesta();
              break;
            case 'a':  //define position using command of d and a # with no spaces or special characters  e.g. d0
              rand_posa = Serial.parseFloat();
              rand_posa = constrain(rand_posa, 0.0, 360.0); //constrain value to between 0-360
              dist = 360;
        	  for(int i=0; i < counts; i++) {
        	  	if(abs(posa[i]-rand_posa)<dist) {
        		  dist = abs(posa[i]-rand_posa);
        		  pcount = i;
        	    }
        	  }
              //pcount = floor(rand_posa / degreeStep); //determine pcount for desired position
              //initial = rand_posa - pcount * degreeStep; //define initial position as degree-degreeStep*pcount
              /*for (int j = 0; j < counts; j++) {
                posa[j] = initial + j * degreeStep + degreeOffset[j];
              }*/
              closesta();
              break;
          #endif //device

          #ifdef peristaltic
                  case 'i' : //peristaltic pump inject
                    dprintv("inject");
                    timep = time2 + injectTime;
                    outputa = injectSpeed;
                    updatea();
                    break;
                  case 'b' :  //peristaltic bleach
                    dprintv("bleach");
                    timep = time2 + cleanTime;
                    outputa = cleanSpeed;
                    updatea();
                    break;
          #endif //peristaltic

          #ifdef valve
                  case 'v':
                    valveSign = !valveSign; //toggle valve open/closed position
                    outputa = maxMotorSpeed;
                    timev = timevi = time2;
                    updatea(); //set motor moving
                    break;
          #endif //valve

          case 's': //stop all motors
			  #ifdef motorB
						outputb = 0; moveMotorb(outputb);
						refillState = LOW;
			  #endif
			  #ifdef motorA
						outputa = 0; moveMotora(outputa);
			  #endif
			  getvol();
			  dprintv("stopped all motors");
			  break;

          #ifdef pump
                  case 'h':
                    refill_rateb = 300;
                    refillb();
                    break;
                  case 'g':
                    getvol();
                    break;
          #endif //pump

		  case 'z':
            save();
            break;
            #ifdef device
                    case 'i':
                      initialize(); // rem_vol --> change to po by multiplying factor
                      break;
            #endif //device
            #ifdef pump
                    case 'c':
                      characterizeb();
                      break;
            #endif //pump
            case 'q':
            sleep();
            break;
        }
      } //isalpha(inbyte)

      //numbers used to pass flow rate; or flowrate,volume command.  values can be integer or decimal
      //e.g. 7 issues command of 7ul/min flow rate; e.g. 10,5 issues command of 10uL/min and 5uL volume
  #ifdef pump
      else {
        dprintv("I am here");
        frate = Serial.parseFloat(); //read in flow rate input
        if (Serial.available() > 0) {
          volDis = Serial.parseFloat(); //read in volume input
        }
        else { //if no volume input detected, automatically set limit to max syringe limit
          volDis = volLim;
          dprintv(volDis);
        }

        //check for negative commands and convert to use desired convention (positive flow rate value, negative volume)
        if (frate < 0 || volDis < 0) {
          frate = abs(frate); //define flow rates as positive values
          volDis = -abs(volDis); //define volume dispense as negative values
          sign = -1.0; Dir = LOW;
        }
        else sign = 1.0, Dir = HIGH;

        //constrain variables and output command to move
        frate = constrain(frate, 0, frateMax); //constrain flow to limit of motor (limit of ~30 for 5V supply)
        volDis = constrain(volDis, -vol, volLim - vol); //constrain volume dispense to syringe limitations
        if (frate != 0) updateb(); //only update if not 0
      }
  #endif //pump

      while (Serial.available() > 0) Serial.read(); //clear Serial
      dprintv("I am here");
    } //end of Serial.available>0

  }//end of input
#endif //printSerial












/*=============List of Commands=================*/
/*
  ------syringe pump commands-----------
  (buf,"O",1)==0) oscillation motion of syringe pump with given volume, oscillation time, and number of oscillations
  (buf,"FV",2)==0) pump with flow rate given by buf[3] to buf[8] and volume to be dispensed given by buf[7] to buf[11]
  (buf,"RF",2)==0) refill the pump with the refill rate given by buf[2] to buf[6]
  (buf,"SV",2)==0) Get the remaining volume from the Pi in buf[2] to buf[7].

  ------peristaltic pump commands-------
  (buf,"X",1)==0)   run peristaltic for duration of injectTime at cleanSpeed
  (buf,"CL",2)==0)  run peristaltic for duration of cleanTime at cleanSpeed

  ----------device commands-------------
  (buf,"FWD",3)==0) move device forward by one step
  (buf,"BKD",3)==0) move device backward by one step
  (buf,"IP",2)==0)  Move device to the arbitrary position given by buf[2] to buf[6] //note: updates all positions to be relative to that new position
  (buf,"PP",2)==0)  Move device to the the four pre-determined positions given by buf[2] to buf[7]

*/

#ifdef I2C
  void input_I2C() {
    Serial.print("buf:"); Serial.println(buf);




  #ifdef pump
    if (strncmp(buf, "O", 1) == 0) {

      // Gets flow rate to be oscillated
      for (int i = 0; i < 6; i++) {
        holdVariable[i] = buf[i + 1];
      }
      frate = charToFloat(holdVariable);

      // Gets amount of time to oscillate in each direction
      for (int i = 0; i < 6; i++) {
        holdVariable[i] = buf[i + 7];
      }
      oscillationTime = charToFloat(holdVariable);

      // Gets amount of oscillations to be done
      for (int i = 0; i < 6; i++) {
        holdVariable[i] = buf[i + 13];
      }
      numberOfOscillations = atoi(holdVariable);

      // Checks for negative values and convert to positive values
      if (frate < 0 || oscillationTime < 0 || numberOfOscillations) {
        frate = abs(frate);                               // Defines frate as positive
        oscillationTime = abs(oscillationTime);           // Defines oscillation time as positive
        numberOfOscillations = abs(numberOfOscillations); // Defines number of oscillations as positive
      }
      refill_rateb = frate;
      nextUpdate = millis();
      oscFlag = 1;
    }

    if (strncmp(buf, "FV", 2) == 0)
    {
      // Start the syringe pump with flow rate given by buf[3] to buf[8] and volume to be dispensed given by buf[7] to buf[11]

      //Get the flow rate
      for (int i = 0; i < 6; i++)
      {
        holdVariable[i] = buf[i + 4];
      }
      frate = charToFloat(holdVariable);
      //check the sign. If negative mltiply by -1.
      //if (buf[3] == '-')
      //{
      // frate = frate * -1.0;
      //}

      //Get the volume to be dispensed
      for (int i = 0; i < 6; i++)
      {
        holdVariable[i] = buf[i + 11];
      }
      volDis = charToFloat(holdVariable);
      //check the sign. If negative mltiply by -1.
      if (buf[10] == '-')
      {
        volDis = volDis * -1.0;
      }

      //check the volume/flow rate selector byte. If volume selected set volDis = 0.
      if (buf[2] != 'V')
      {
        volDis = 0.0;
      }


      dprintv("I am here");
      if (volDis == 0.0) { //if no volume input detected, automatically set limit to remaining volume
        volDis = (volLim - vol);
        dprintv(volDis);
      }

      //check for negative commands and convert to use desired convention (positive flow rate value, negative volume)
      if (frate < 0 || volDis < 0) {
        frate = abs(frate); //define flow rates as positive values
        volDis = -abs(volDis); //define volume dispense as negative values
        sign = -1.0; Dir = LOW;
      }
      else sign = 1.0; Dir = HIGH;
      //constrain variables and output command to move
      frate = constrain(frate, 0, frateMax); //constrain flow to limit of motor (limit of ~30 for 5V supply)
      volDis = constrain(volDis, -vol, volLim - vol); //constrain volume dispense to syringe limitations

      if (frate != 0)
      {
        // Set the status flags to reflect that the pump is moving
        err_status[0] = '5'; err_status[1] = '7'; //set flag to signify syringe moving to be sent to I2C
        updateb();  //only update if not 0
      }


    }

    else if (strncmp(buf, "RF", 2) == 0)
    {
      //Perform task to refill the pump with the refill rate given by buf[2] to buf[6]

      //Get the Refill Rate
      for (int i = 0; i < 5; i++)
      {
        holdVariable[i] = buf[i + 2];
      }
      refill_rateb = charToFloat(holdVariable);

      refillb();

    }

    else if (strncmp(buf, "P", 1) == 0)
    {
      //Stop the syringe pump in Channel B

      outputb = 0; moveMotorb(outputb);
      refillState = LOW;
      // Set the status flags to reflect that the pump is stopped
      err_status[0] = '5'; err_status[1] = '0'; //set flag to signify pumpmoving to be sent to I2C
      getvol();

    }

    else if (strncmp(buf, "SV", 2) == 0)
    {
      // Get the remaining volume from the Pi in buf[2] to buf[8].

      // Get the Remaining Volume
      for (int i = 0; i < 7; i++)
      {
        holdVariable[i] = buf[i + 2];
        rem_vol[i] = holdVariable[i];
      }
      rem_volb = charToFloat(holdVariable);
      //Serial.print("rem_volb =");Serial.println(rem_volb);
      // get the syringe position from the remaining volume by first calculating the volume dispensed.
      pos = (volLim - rem_volb) * factor;
      //Serial.print("pos= ");Serial.println(pos);

    }

  #endif //pump







  #ifdef peristaltic
  #ifndef pump
    if (strncmp(buf, "X", 1) == 0)
  #else
    else if (strncmp(buf, "X", 1) == 0)
  #endif
    {
      // Start Peristaltic pump.
      dprintv("inject");
      // Get the time
      for (int i = 0; i < 7; i++)
      {
        holdVariable[i] = buf[i + 1];	//transfer buf into holdVariable
      }
      injectTime = uint32_t ((charToFloat(holdVariable)) * 1000);
      timep = time2 + injectTime;
      outputa = injectSpeed;
      updatea();
    }
    else if (strncmp(buf, "CL", 2) == 0)
    {
      // Clean Peristaltic pump.
      dprintv("bleach");
      //Get the time
      for (int i = 0; i < 7; i++)
      {
        holdVariable[i] = buf[i + 2];

      }
      cleanTime = uint32_t ((charToFloat(holdVariable)) * 1000);
      timep = time2 + cleanTime;
      outputa = cleanSpeed;
      updatea();
    }
  #endif //peristaltic








  #ifdef device
  #ifndef pump
    if (strncmp(buf, "FWD", 3) == 0)
  #else
    else if (strncmp(buf, "FWD", 3) == 0)
  #endif

    {
      // Move the injector forward to the next position.
      pcount++;
      signa = HIGH; //High means move right
      if (pcount > counts - 1) {
        pcount = 0;
      }
      dprintv("moving right to position"); dprintv(pcount);
      // Set the status flags to reflect that the injector is moving
      err_status[0] = '0'; err_status[1] = '7';
      updatea();


    }
    else if (strncmp(buf, "BKD", 3) == 0)
    {
      // Move the injector backward to the previous position.
      pcount--;
      signa = LOW; //LOW means move left
      if (pcount < 0) {
        pcount = counts - 1;
      }
      dprintv("moving left to position"); dprintv(pcount);
      // Set the status flags to reflect that the injector is moving
      err_status[0] = '0'; err_status[1] = '7';
      updatea();

    }
    else if (strncmp(buf, "AP", 2) == 0)
    {
      // Move the injector to the arbitrary position given by buf[2] to buf[6]

      //Get the position requested
      for (int i = 0; i < 5; i++)
      {
        holdVariable[i] = buf[i + 2];
      }
      rand_posa = constrain(charToFloat(holdVariable), 0.0, 360.0); //constrain value to between 0-360 degrees
      //pcount = floor(rand_posa / degreeStep); //determine pcount for desired position
      dist = 360;
      for(int i=0; i < counts; i++) {
      	if(abs(posa[i]-rand_posa)<dist) {
      		dist = abs(posa[i]-rand_posa);
      		pcount = i;
      	}
      }
      /*initial = rand_posa - pcount * degreeStep; //define initial position as degree-degreeStep*pcount
      for (int j = 0; j < counts; j++) {
        posa[j] = initial + j * degreeStep + degreeOffset[j];
      }*/
      closesta();
    }
    else if (strncmp(buf, "IP", 2) == 0)
    {
      // Move the injector to the the four pre-determined positions given by buf[2] to buf[7]

      //Get the position requested
      for (int i = 0; i < 6; i++)
      {
        holdVariable[i] = buf[i + 2];
      }
      fixed_posa = charToFloat(holdVariable);
      pcount = (int) fixed_posa;
      Serial.print("fixed_posa"); Serial.println(fixed_posa);
      Serial.print("pcount"); Serial.println(pcount);
      closesta();
    }

  #endif //device






  #ifdef valve
    else if (strncmp(buf, "VP", 2) == 0)
    {
      #ifdef printSerial
        Serial.println("switching valve");
      #endif //printSerial
      valveSign = !valveSign; //toggle valve open/closed position
      outputa = maxMotorSpeed;
      timev = timevi = time2;
      updatea();
      err_status[0] = '0'; err_status[1] = '7'; //set flag to signify valve is moving
    }
  #endif //valve

  }

#endif //I2C







//check to see if device position limits exceeded or not
#ifdef device
void limita() {

  //initialize variables
  dprintv("I am here");
  outputaLast = outputa; //store value for update
  motorPosition = readSensorValuea(); //get position
  dprintv(motorPositionLast); dprintv(motorPosition); dprintv(outputa);//

  //check for anamolous position encoding readout values
  deltaPosition = abs(motorPosition - motorPositionLast); //determine amount of position change since last reading
  motorPositionLast = motorPosition; //store value for next loop
  if (deltaPosition > 2) { //continue calculations if deltaposition less than defined value, otherwise do nothing and print error
    dprintv("Encoder Rollover Anomoly")
  }
  else {

    //what to do if desired direction is to the right
    if (signa == HIGH) {
      if (pcount == 0 && motorPosition > posa[counts - 2] - 1) { //check for special case of encoder 360 rollover
        motorPosition = motorPosition - 360; //account for rollover of encoder
      }
      //check to see of reached position yet
      if (motorPosition > posa[pcount] - overShootVal) {
        outputa = 0;
        moveMotora(outputa); //stop motor
        delay(1); //for stability
        motorPosition = readSensorValuea(); //verify correct position
        dprintv("arrived at"); dprintv(motorPosition);

        //check for overshoot, correct if necessary
        if (motorPosition > (posa[pcount] + overShootVal)) {
          dprintv("overshoot detected")
          signa = !signa; //reverse direction
          updatea();
        }
        else {
          #ifdef printSerial
                    printsa();
          #endif
        }
      }
      //else keep going, slow down when you get close
      else {
        outputa = constrain(100 * abs((posa[pcount] - motorPosition)), minMotorSpeed, maxMotorSpeed); //calculate new output speed if getting close to position, constrain between min and max MotorSpeed
        if (outputa != outputaLast) moveMotora(outputa); //only update if speed has changed
      }
    }

    //what to do if desired direction is to the left
    else {
      if (pcount == counts - 1 && motorPosition < posa[1] + overShootVal) { //check for special case of encoder 360 rollover
        motorPosition = motorPosition + 360; //account for rollover of encoder
      }
      if (motorPosition < posa[pcount] + overShootVal) { //check to see of reached position yet
        outputa = 0;
        moveMotora(outputa);  //stop motor

        delay(1); //for stability
        motorPosition = readSensorValuea(); //verify at correct position
        dprintv("arrived at"); dprintv(motorPosition);
        if (motorPosition < (posa[pcount] - overShootVal)) { //check for overshoot, correct if inecessary
          dprintv("overshoot detected")
          signa = !signa; //reverse direction
          updatea();
        }
        else {
          #ifdef printSerial
                    printsa();
          #endif
        }
      }
      else {
        outputa = constrain(100 * abs(posa[pcount] - motorPosition), minMotorSpeed, maxMotorSpeed); //calculate new output speed if getting close to position, constrain between min and max MotorSpeed
        if (outputa != outputaLast) moveMotora(outputa); //only update if speed has changed
      }
    }

    //housekeeping routines to detect stalling or timeout conditions
    if (time2 - timeai > 1200) { //keep updating stall position and time every 1.2 seconds.
      motorPositioni = motorPositionLast;
      timeai = time2;
    }
    //check to see if motor is stalled
    if (abs(motorPositionLast - motorPositioni) < 3 && time2 - timeai > 1000 && outputa != 0) { //check if compare initial time and position to current.
      dprintv("Motor Stalled");
      #ifdef printSerial
            Serial.println("Motor Stalled");
      #endif
      #ifdef I2C
            err_status[0] = '9'; err_status[1] = '7'; //set device motor stall error flag to be sentto I2C
      #endif
      outputa = 0; //set speed to zero
      moveMotora(outputa); //stop device,
    }
  }

  dprintv("I am here");

  //Update the current injector position buffer to be sent back to the Pi
  #ifdef I2C
      convert2String (motorPosition, 6);

      for (int k = 0; k < 6; k++) // k iterates from 1 to 6 filling bit 1 to bit 6 into array
      {
        inj_pos[k] = stringValue[k];
      }

      inj_pos[6] = '\n';  // setting last bit to be '\n' to easily parse at Pi side.
    #ifdef printSerial
      Serial.println(inj_pos);
    #endif //printSerial
  #endif //I2C
}
#endif //device







#ifdef pump

  //check to see if syringe limits are exceeded or not
  void limitb() {
    dprintv("I am here");

    //if we have dipensed enough, or if we are at the limit
    if (abs(volDelta + volDeltaPrevious) > abs(volDis) || pos < 0 || pos > posLim) {
      //do nothing if correcting for overlimit
      if (Dir == HIGH && pos < 0) {
        #ifdef printSerial
              Serial.println("overlimit ok");
        #endif
      }
      //do nothing if correcting for overlimit
      else if (Dir == LOW && pos > posLim) {
        #ifdef printSerial
              Serial.println("overlimit ok");
        #endif
      }
      //stop motor from going beyond limit
      else {
        outputb = 0; moveMotorb(outputb); //set speed to zero, stop pump (limit reached)
        if (refillState == HIGH) {
          dprintv("done refilling");
          #ifdef I2C
                  err_status[0] = '0'; err_status[1] = '0'; //set pump overlimit error flag to be sent to I2C
          #endif
          getvol();
        }
        refillState = LOW;
        #ifdef printSerial
              Serial.println("overlimit, stopping motor");
              printsb();
        #endif
        #ifdef I2C
              err_status[0] = '5'; err_status[1] = '0'; //set pump overlimit error flag to be sent to I2C
        #endif
      }
    }

    //check to see if motor load has dramatically changed to avoid mechanical crashing of piston and harm to pump
    if (abs(outputi - integral * iGain) > outputError) { //check if average motor demand value significantly changed
      if (overloadFlag == LOW) {
        overloadFlag = HIGH;
        timeOverload = time;
      }
      else timeDeltaOverload = time - timeOverload;
      if (timeDelta > 25000) { //stop motor after 25ms of overload
        double voltage = analogRead(ADC1); //(A1 on motor B)  //check if pump has exceeded limit of travel
        #ifdef pump_484563
              float volcheck = volLim * voltage / 635; //
        #endif //pump_484563
        #ifdef pump_487923
              float volcheck = volLim * (voltage - 275) / 355; //
        #endif //pump_487923
        if (volcheck < 10 || volcheck > volLim - 10 || timeDeltaOverload > 250000) { //stop it if it has exceeded limit of travel
          outputb = 0; //set speed to zero
          moveMotorb(outputb); //stop pump,
          refillState = LOW; //reset refillState
          #ifdef printSerial
                  Serial.println("characterization not valid or motor overloaded");
                  Serial.print(volLim - volcheck), Serial.print ("volcheck, "); Serial.print(outputi), Serial.print(","), Serial.print(integral * iGain), Serial.print(","), Serial.println(abs(outputi - integral * iGain));
                  printsb();
          #endif
          #ifdef I2C
                  err_status[0] = '5'; err_status[1] = '6'; //set pump characterization error flag to be sent to I2C
          #endif
          getvol();
        }
      }
    }
    else overloadFlag = LOW; //set overload flag low  (integral*iGain>outputError, )
  }
#endif //pump





//MOVES MOTOR AT THE INPUT SPEED GIVEN BY motorSpeed IN THE DIRECTION GIVEN BY signa
#ifdef motorA
void moveMotora(double motorSpeed) {
  dprintv(motorSpeed);
  dprintv("-------------------------------------------------------------------");
  //0 is max speed, 255 is stop, invert here for simplicity elsewhere
  motorSpeed = constrain(255 - motorSpeed, 0, 255);
  if (signa == HIGH) { //positive dispense volume move forward
    digitalWrite(HBridgeAPin1, HIGH); //high for slow decay
    analogWrite(HBridgeAPin2, motorSpeed); //PWM sets speed
    dprintv("moving right");
  }
  else { //reverse direction if negative dispense volume
    analogWrite(HBridgeAPin1, motorSpeed);  //PWM sets speed
    digitalWrite(HBridgeAPin2, HIGH);  //high for slow decay
    dprintv("moving left");
  }
}
#endif //motorA





//MOVES MOTOR AT THE INPUT SPEED GIVEN BY motorSpeed IN THE DIRECTION GIVEN BY signb
#ifdef motorB
void moveMotorb(int motorSpeed) {
  //0 is max speed, 255 is stop, invert here for simplicity elsewhere
  motorSpeed = constrain(255 - motorSpeed, 0, 255); //constrain output to not exceed 8bit PWM output
  if (sign >= 0) { //positive dispense volume move forward
    digitalWrite(HBridgeBPin1, HIGH); //high for slow decay
    analogWrite(HBridgeBPin2, motorSpeed); //PWM sets speed
  }
  else { //reverse direction if negative dispense volume
    analogWrite(HBridgeBPin1, motorSpeed);  //PWM sets speed
    digitalWrite(HBridgeBPin2, HIGH);  //high for slow decay
  }
}
#endif //motorB





//Serial print commands for motor position while moving if printSerial is #defined
#ifdef printSerial
#ifdef device
  void printsa() {
    Serial.print("At position ");
    Serial.println(motorPositionLast); //not a negative value
  }
#endif //device
#endif //printSerial






#ifdef printSerial
#ifdef pump
  //print routine called for syringe pump if printSerial is #defined
  void printsb() {
    Serial.print(" pos: ");Serial.print(pos);Serial.print("  |  ");
    Serial.print(" demand:  ");Serial.print(demand); Serial.print("  |  ");
    Serial.print(" error : ");Serial.print(error); Serial.print("  |  ");
    Serial.print(" speed  :");Serial.print(outputb); Serial.print("  |  ");
    Serial.print(" volume  ");Serial.print(volLim - vol); Serial.print("  |  ");
    Serial.print(" volume :dispensed  ");Serial.print(volDelta + volDeltaPrevious); Serial.print("  |  ");
    Serial.print(" volDis : ");Serial.print(volDis); Serial.print("  |  ");
    Serial.print(" ul/min : ");Serial.print(rate); Serial.print("  |  ");
    Serial.print(" sec : ");Serial.print(timeDelta / 1.0e6); Serial.print("  |  ");
    Serial.print(" buffer : ");Serial.print(buf); Serial.print("  |  ");
    #ifdef pump_484563
      Serial.print(skip); Serial.print(" skip ");
    #endif
      Serial.print(integral * iGain - outputi); Serial.println(" interror");

  }
#endif //pump
#endif //printSerial







#ifdef device

//Function to read the value of the analog absolute encoder
double readSensorValuea() {

  degree = 0.0;

  Serial.println("checking for range of encoder. Comment me out after calibration...");
	//take sensorAvgCount readings from the sensor, serial print them out, and return the average of them
  for (int i = 1;  i <= sensorAvgCount ; i++) {
    // read the input on analog pin
    int sensorValue = analogRead(ADC0);
    Serial.println(sensorValue);//check for range of encoder, comment out after calibration
    double degreehold = 0.0, voltage = 0.0;
    // Map the input value to degree using two steps to account for the encoder voltage range
    voltage = fmap(sensorValue, 0, 1019 - fmapadjust, 0.015, 4.987);
    degreehold = fmap(voltage, 0.015, 4.987, 0.0, 360.0);

    degree += degreehold;

    delayMicroseconds(100);        // delay in between reads for stability
  }

  double degreeReturn = degree / sensorAvgCount;
  return degreeReturn;
}





// Function to do the raw voltage conversion
double fmap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif //device






#ifdef pump

  void refillb() {
    dprint("Refilling Pump");
    //move to home position until limit switch detected
    //re-initialize variables
    dprintv("I am Here")
    sign = -1.0; Dir = LOW;
    volDis = volLim;
    frate = refill_rateb;
    refillState = HIGH;
    #ifdef I2C
      err_status[0] = '5'; err_status[1] = '7'; //set flag to signify syringe moving to be sent to I2C
    #endif
    updateb();
  } //end refillb

#endif //pump





#ifdef motorA

  //get the motor position and move it. If it's asleep, wake it up.
  void updatea() {
    if (sleepState == LOW) {
      #ifdef printSerial
          Serial.println("---------------Waking up----------------");
      #endif
      sleep();	//toggles sleep state
      delay(1);
    }

    #ifdef device
      motorPosition = readSensorValuea();
      motorPositioni = motorPosition; //re-initialize position
      motorPositionLast = motorPosition;
	  #ifdef printSerial
	    printsa();  //serial print the last position
	  #endif
      timeai = time2; //re-intialize timeai
      outputa = constrain(100 * abs(posa[pcount] - motorPosition), minMotorSpeed, maxMotorSpeed); //calculate new output speed using the distance from the target and the constrain function
      Serial.println(outputa);
      moveMotora(outputa);
    #endif

    #ifdef peristaltic
      signa = HIGH;
      moveMotora(outputa);
    #endif //peristaltic

    #ifdef valve
      signa = HIGH;
      moveMotora(outputa);
    #endif //valve
  } //updatea
#endif //motorA






#ifdef pump

  //initialize dispense variables and begin dispensing syringe volume
  void updateb() {

    if (sleepState == LOW) {
		#ifdef printSerial
			Serial.println("---------------Waking up------------");
		#endif
        sleep();
        delay(1);
    }
    outputb = convert(frate); //calculate predetermined PWM output close to flow rate (will change with motor supply voltage)
    outputOld = outputb; //initialize motor update status
    outputi = outputb; //used to determine if slope & offset are valid or motor overtaxed
    demandi = pos, error = 0, //initialize demand value to be current position
    voli = vol; //initialize volume start position for dispense calculations
    volDeltaPrevious = 0; //initialize dispensed volume to zero
    integral = outputb / iGain; //initialize integral based on slope & offset output used in convert() routine
    timei = micros(); //initialize time for timeDelta calculations

    moveMotorb(outputb);  //tell motor to dispense

  }


  //initialize motor speed based on curve fit of PWM to desired flow rates
  double convert(double frate) {
    return frate * slope + offset; //current slope & offset values determined for 5V supply
  }

#endif //pump





#ifdef pump

  void getvol() {

    //average of three measurements to reduce error
    double voltage = analogRead(ADC1); //(A1 on motor B)
    voltage += analogRead(ADC1);
    voltage = (voltage + analogRead(ADC1)) / 3;

    #ifdef pump_484563
      vol = volLim * voltage / 460;
    #endif //pump_484563
    #ifdef pump_487923
      vol = volLim * (voltage - 275) / 355;
      vol = constrain(vol, 0.0, volLim);
    #endif //pump_487923
      pos = round(vol * factor);
    #ifdef printSerial
      Serial.print(" voltage"); Serial.println(voltage);
      printsb();
    #endif //printSerial
      update_rem_volb();

  }

#endif //pump






#ifdef I2C

  void update_rem_volb() {

    rem_volb = volLim - vol;
    convert2String (rem_volb, 7);
    for (int k = 0; k < 7; k++) // k iterates from 0 to 6 filling bit 0 to bit 6 into array
    {
      rem_vol[k] = stringValue[k];
    }

    rem_vol[7] = '\n';  // setting last bit to be '\n' to easily parse at Pi side.

    #ifdef valve

      if (valveSign == HIGH)
        rem_vol[7] = '0';
      if (valveSign == LOW)
        rem_vol[7] = '1';

    #endif

    rem_vol[8] = '\n';

  }

#endif //I2C
