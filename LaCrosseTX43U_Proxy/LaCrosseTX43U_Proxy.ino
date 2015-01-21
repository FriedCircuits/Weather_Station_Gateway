// Unknown 433Mhz weather sensor decoder. Untested in the real world.
// http://arduino.cc/forum/index.php/topic,142871.msg1106336.html#msg1106336

// __           ___       ___    ___
//   |         |  |      |  |   |  |
//   |_________|  |______|  |___|  |
//
//   |  Sync      |    1    |  0   |
//   |  8320us    | 4500us  | 2530us


//Adding RFM12B support to act as a gateway for Jeenode network
//FriedCrcuits - William Garrido - 12/19/2014
#include <JeeLib.h>
#include <avr/sleep.h>

#define SERIAL 0 //Enable Serial output


// Defines
#define DataBits0 4                                       // Number of data0 bits to expect
#define DataBits1 32                                      // Number of data1 bits to expect
#define allDataBits 36                                    // Number of data sum 0+1 bits to expect
// isrFlags bit numbers
#define F_HAVE_DATA 1                                     // 0=Nothing in read buffer, 1=Data in read buffer
#define F_GOOD_DATA 2                                     // 0=Unverified data, 1=Verified (2 consecutive matching reads)
#define F_CARRY_BIT 3                                     // Bit used to carry over bit shift from one long to the other
#define F_STATE 7                                         // 0=Sync mode, 1=Data mode

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

#define ACK_TIME        10  // number of milliseconds to wait for an ack

static byte myNodeID;       // node ID used for this unit

//For packing data to send to root jeenode
struct {
      byte light;
      byte reed :1;
      byte humi :7;
      byte temp :10;
      byte lobat :1;
      //byte moved :1;
      //int battery :13;
}payload;


// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}

void sendData(){
  // actual packet send: broadcast to all, current counter, 1 byte long
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &payload, sizeof payload);
  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_sleep(RF12_SLEEP);
  #if SERIAL
    Serial.print("SHA_Node:");
    Serial.print(rf12_hdr);
     Serial.print(":");
    Serial.print((int) payload.light);
    Serial.print(":");
    Serial.print((int) payload.reed);
    Serial.print(":");
    Serial.print((int) payload.humi);
    Serial.print(":");
    Serial.print((int) payload.temp);
    //Serial.print(":");
    //Serial.print((int) payload.moved);
    Serial.print(":");
    Serial.println((int) payload.lobat);
    //Serial.print(":");
    // Serial.println((int) payload.battery);
    serialFlush();
 #endif
}

static void serialFlush () {
    Serial.flush();
    delay(2); // make sure tx buf is empty before going back to sleep
}

// Constants
const unsigned long sync_MIN = 4300;                      // Minimum Sync time in micro seconds
const unsigned long sync_MAX = 4700;

const unsigned long bit1_MIN = 2300;
const unsigned long bit1_MAX = 2700;

const unsigned long bit0_MIN = 1330;
const unsigned long bit0_MAX = 1730;

const unsigned long glitch_Length = 300;                  // Anything below this value is a glitch and will be ignored.

// Interrupt variables
unsigned long fall_Time = 0;                              // Placeholder for microsecond time when last falling edge occured.
unsigned long rise_Time = 0;                              // Placeholder for microsecond time when last rising edge occured.
byte bit_Count = 0;                                       // Bit counter for received bits.
unsigned long build_Buffer[] = {
  0,0};                     // Placeholder last data packet being received.
volatile unsigned long read_Buffer[] = {
  0,0};             // Placeholder last full data packet read.
volatile byte isrFlags = 0;                               // Various flag bits

void PinChangeISR0(){                                     // Pin 2 (Interrupt 0) service routine
  unsigned long Time = micros();                          // Get current time
  if (digitalRead(3) == LOW) {
    // Falling edge
    if (Time > (rise_Time + glitch_Length)) {
      // Not a glitch
      Time = micros() - fall_Time;                        // Subtract last falling edge to get pulse time.
      if (bitRead(build_Buffer[1],31) == 1)
        bitSet(isrFlags, F_CARRY_BIT);
      else
        bitClear(isrFlags, F_CARRY_BIT);

      if (bitRead(isrFlags, F_STATE) == 1) {
        // Looking for Data
        if ((Time > bit0_MIN) && (Time < bit0_MAX)) {
          // 0 bit
          build_Buffer[1] = build_Buffer[1] << 1;
          build_Buffer[0] = build_Buffer[0] << 1;
          if (bitRead(isrFlags,F_CARRY_BIT) == 1)
            bitSet(build_Buffer[0],0);
          bit_Count++;
        }
        else if ((Time > bit1_MIN) && (Time < bit1_MAX)) {
          // 1 bit
          build_Buffer[1] = build_Buffer[1] << 1;
          bitSet(build_Buffer[1],0);
          build_Buffer[0] = build_Buffer[0] << 1;
          if (bitRead(isrFlags,F_CARRY_BIT) == 1)
            bitSet(build_Buffer[0],0);
          bit_Count++;
        }
        else {
          // Not a 0 or 1 bit so restart data build and check if it's a sync?
          bit_Count = 0;
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bitClear(isrFlags, F_GOOD_DATA);                // Signal data reads dont' match
          bitClear(isrFlags, F_STATE);                    // Set looking for Sync mode
          if ((Time > sync_MIN) && (Time < sync_MAX)) {
            // Sync length okay
            bitSet(isrFlags, F_STATE);                    // Set data mode
          }
        }
        if (bit_Count >= allDataBits) {
          // All bits arrived
          bitClear(isrFlags, F_GOOD_DATA);                // Assume data reads don't match
          if (build_Buffer[0] == read_Buffer[0]) {
            if (build_Buffer[1] == read_Buffer[1]) 
              bitSet(isrFlags, F_GOOD_DATA);              // Set data reads match
          }
          read_Buffer[0] = build_Buffer[0];
          read_Buffer[1] = build_Buffer[1];
          bitSet(isrFlags, F_HAVE_DATA);                  // Set data available
          bitClear(isrFlags, F_STATE);                    // Set looking for Sync mode
          digitalWrite(13,HIGH); // Used for debugging
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bit_Count = 0;
        }
      }
      else {
        // Looking for sync
        if ((Time > sync_MIN) && (Time < sync_MAX)) {
          // Sync length okay
          build_Buffer[0] = 0;
          build_Buffer[1] = 0;
          bit_Count = 0;
          bitSet(isrFlags, F_STATE);                      // Set data mode
          digitalWrite(13,LOW); // Used for debugging
        }
      }
      fall_Time = micros();                               // Store fall time
    }
  }
  else {
    // Rising edge
    if (Time > (fall_Time + glitch_Length)) {
      // Not a glitch
      rise_Time = Time;                                   // Store rise time
    }
  }
}


void setup() {
  pinMode(13,OUTPUT); // Used for debugging
  #if SERIAL
    Serial.begin(57600);
  #endif
  pinMode(3,INPUT);
  #if SERIAL
    Serial.println(F("ISR Pin 2 Configured For Input."));
  #endif
  attachInterrupt(1,PinChangeISR0,CHANGE);
  #if SERIAL
    Serial.println(F("Pin 2 ISR Function Attached. Here we go."));
    Serial.print("\n[roomNode.3]");
    serialFlush();
  #endif   
  myNodeID = rf12_config();
  myNodeID = rf12_config(0);
  rf12_sleep(RF12_SLEEP); 
  payload.reed = 0;
}

void loop() {
  unsigned long myData0 = 0;
  unsigned long myData1 = 0;
  if (bitRead(isrFlags,F_GOOD_DATA) == 1) 
  {
    // We have at least 2 consecutive matching reads
    myData0 = read_Buffer[0]; // Read the data spread over 2x 32 variables
    myData1 = read_Buffer[1];
    bitClear(isrFlags,F_HAVE_DATA); // Flag we have read the data
    dec2binLong(myData0,DataBits0);
    dec2binLong(myData1,DataBits1);

    #if SERIAL
      Serial.print(" - Battery=");
    #endif
    byte H = (myData1 >> 26) & 0x3;   // Get Battery
    payload.lobat = H;
    #if SERIAL
      Serial.print(H);
      Serial.print(" Channel=");
    #endif
    H = ((myData1 >> 24) & 0x3) + 1;        // Get Channel
    #if SERIAL
      Serial.print(H);
    #endif
    payload.light = H;
    #if SERIAL
      Serial.print(" Temperature=");
    #endif      
    byte ML = (myData1 >> 12) & 0xF0; // Get MMMM
    //     Serial.print(" (M=");
    //     Serial.print(ML);
    H = (myData1 >> 12) & 0xF;        // Get LLLL
    //     Serial.print(" L=");
    //     Serial.print(H);
    ML = ML | H;                      // OR MMMM & LLLL nibbles together
    H = (myData1 >> 20) & 0xF;        // Get HHHH
    //     Serial.print(" H=");
    //     Serial.print(H);
    //     Serial.print(" T= ");
    byte HH = 0;
    if((myData1 >> 23) & 0x1 == 1) //23 bit
      HH = 0xF;
    int Temperature = (H << 8)|(HH << 12) | ML;  // Combine HHHH MMMMLLLL
    //     Serial.print( Temperature);
    //     Serial.print(") ");
    // Temperature = Temperature*3; //F // Remove Constant offset
    #if SERIAL
      Serial.print(Temperature/10.0,1);   
      Serial.print("C Humidity=");
    #endif
    H = (myData1 >> 0) & 0xF0;        // Get HHHH
    //     Serial.print(" (H=");
    //     Serial.print(H);
    ML = (myData1 >> 0) & 0xF;       // Get LLLL
    //     Serial.print(" L=");
    //     Serial.print(ML);
    //     Serial.print(") ");
    ML = ML | H;                      // OR HHHH & LLLL nibbles together
    #if SERIAL
      Serial.print(ML);
      Serial.println("%");
      serialFlush();
    #endif

    payload.humi = ML;
    payload.temp = (1.8 * (Temperature/10.0)) + 32;
    //payload.lobat = rf12_lowbat(); //Can be enabled when running on batteries, otherwise we want to know the battery status of the remote sensor. When enabled will override sensor battery reading above
    sendData();
  }
  delay(100);
  //set_sleep_mode(SLEEP_MODE_IDLE);
        //sleep_mode();
  
}

void dec2binLong(unsigned long myNum, byte NumberOfBits) {
  if (NumberOfBits <= 32){
    myNum = myNum << (32 - NumberOfBits);
    for (int i=0; i<NumberOfBits; i++) {
      if (bitRead(myNum,31) == 1){ 
        #if SERIAL
          Serial.print("1");
        #endif
      }
      else{ 
        #if SERIAL
          Serial.print("0");
        #endif 
      }
      myNum = myNum << 1;
    }
  }
  #if SERIAL
   serialFlush();
  #endif
}


