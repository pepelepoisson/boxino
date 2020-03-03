/*
 * Pascal Prado, papasinventeurs@gmail.com, February 2020
 * ======================================================
 * A TOF10120 "Time Of Flight" distance detector is connected via I2C to the Arduino nano to detect passing boxes.
 * A filtration algorithm is used to count valid obstacles and discart false readings. 
 * Valid detections trigger a serial message sent by the Arduino to distance Pi for processing. A beep and a confirmation LED are also used.
 * Filtration algorithm can be reconfigured by sending serial messages from the Pi to the Arduino.
 * Button sends a serial message + bip + confirmation LED when pressed
 * 
 * TO BE DONE:
 * Read/Write filtration parameters in Arduino EEPROM memory to restore new setup automatically after a restart.
 * 
 */

#include <Wire.h>

#define BUZZER1  9  // Buzzer pin definitions (the other pin goes to ground)
#define LED1  5  // Buzzer pin definitions (the other pin goes to ground)
#define PUSH1  2  // Push button pin definitions (the other pin goes to ground)
#define BUTTON1_PUSH  (!digitalRead(PUSH1))  // Button is on when circuit is closed - used for standard push buttons
#define INPUT_SIZE 30  // Calculate based on max input size expected for one command received from Serial


unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
int distance_detection=500;  // Minimum distance to trigger detection (mm)
int minimum_duration_detection=100;  // Minimum duration of detection to validate true positive (ms)
int minimum_wait_time=1000;  // Minimum delay between two different detections, to avoid counting same object twice (ms)
bool detection_triggered=true, detection_disabled=false, object_detected=false;  // Status flags used to trigger or disable detection
long detection_timer=0;  // Timer used for time-based filters
int object_counter=0;  // Counter of total number of objects detected


int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}

int ReadDistance(){
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(50); 
    return lenth_val;
}

void buzz_sound(int buzz_length_ms, int buzz_delay_us){  // Toggle buzzer every buzz_delay_us, for a duration of buzz_length_ms.
  // Convert total play time from milliseconds to microseconds
  long buzz_length_us = buzz_length_ms * (long)1000;

  // Loop until the remaining play time is less than a single buzz_delay_us
  while (buzz_length_us > (buzz_delay_us * 2))
  {
    buzz_length_us -= buzz_delay_us * 2; //Decrease the remaining play time

    // Toggle the buzzer at various speeds
    digitalWrite(BUZZER1, LOW);
    delayMicroseconds(buzz_delay_us);


    digitalWrite(BUZZER1, HIGH);
    delayMicroseconds(buzz_delay_us);
  }
  digitalWrite(BUZZER1,LOW);
}

bool detection_filter(int distance){
  if (distance <=distance_detection){
    if (detection_triggered==false){
      // Detector is triggered - object is not yet confirmed (could be false alarm)
      detection_triggered=true;
      detection_timer=millis();  // Start counter to verify the duration of detection
      }
    if (detection_triggered==true && detection_disabled==false && millis()-detection_timer>=minimum_duration_detection){
      // Object is confirmed
      detection_timer=millis();  // Counter is resetted - will now be used to disable detection for some time
      detection_disabled=true;
      object_detected=true;
    }
    else {object_detected=false;}
  }
  else {
    detection_triggered=false;
    if (millis()-detection_timer>=minimum_wait_time){detection_disabled=false;}
    object_detected=false;
    }

  /*
  Serial.print(distance);Serial.print(",");
  Serial.print(detection_triggered);Serial.print(",");
  Serial.print(detection_disabled);Serial.print(",");
  Serial.println(object_detected);
  */
  
  return object_detected;
}

void setup_detection_from_serial(){
  // Parameters of the detection algorithm can be changed based on text messages received from serial. The format is as follows: X:YYYYY
  // X indicates the parameter to be changed and Y are the digits of the new value.
  // X=1 for distance_detection (mm)
  // X=2 for minimum_duration_detection (ms)
  // X=3 for minimum_wait_time (ms)
  // If one of the variables gets changed then the value of all parameters is sent back as serial confirmation message.
  
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  byte size = Serial.readBytes(input, INPUT_SIZE);
  // Add the final 0 to end the C string
  input[size] = 0;

  // Read each command pair 
  char* command = strtok(input, "&");
  while (command != 0)
  {
    // Split the command in two values
    char* separator = strchr(command, ':');
    if (separator != 0)
    {
        // Actually split the string in 2: replace ':' with 0
        *separator = 0;
        int CommandNumber = atoi(command);
        ++separator;
        int CommandValue = atoi(separator);

        if (CommandNumber==1){distance_detection=CommandValue;}
        if (CommandNumber==2){minimum_duration_detection=CommandValue;}
        if (CommandNumber==3){minimum_wait_time=CommandValue;}
        
        Serial.print(distance_detection);Serial.print(":");Serial.print(minimum_duration_detection);Serial.print(":");Serial.println(minimum_wait_time);
    }
    // Find the next command in input string
    command = strtok(0, "&");
}
}

void setup() {
  Wire.begin(); 
  Serial.begin(115200,SERIAL_8N1); 
  printf_begin();   
  
  pinMode(BUZZER1, OUTPUT);  //Setup outputs for buzzer
  pinMode(LED1, OUTPUT);  //Setup outputs for LED
  pinMode(PUSH1,INPUT);
  digitalWrite(PUSH1,HIGH);  // Configure built-in pullup resitor for push button 1
  
  digitalWrite(LED1,HIGH);  // Turn LED ON
  
  buzz_sound ( 150, 1136);
  //delay (1000);
  //buzz_sound ( 150, 851);
  //delay (1000);    
  //buzz_sound ( 150, 638);
  //delay (1000);
  //buzz_sound ( 150, 568);
  delay (1000);  
  
  digitalWrite(LED1,LOW);  // Turn LED OFF
}

void loop() {
  

   int x=ReadDistance();
   bool status=detection_filter(x);
   //Serial.println(status);
   //Serial.println(x);
   //Serial.println(" mm");
   if (status==true){
    object_counter++;
    Serial.print("Object "); Serial.print(object_counter); Serial.println(" detected!");
    digitalWrite(LED1,HIGH);  // Turn LED ON
    buzz_sound ( 150, 851);
    digitalWrite(LED1,LOW);  // Turn LED OFF
    }
    
    if (Serial.available() > 0) {  
      // If something arrives in the serial read buffer then wait a moment then run the routine to reconfigure detector.
      delay(100);
      setup_detection_from_serial();
    }

    if (BUTTON1_PUSH){
      digitalWrite(LED1,HIGH);  // Turn LED ON
      buzz_sound (300,200);
      Serial.println("Button pushed");
      while (BUTTON1_PUSH){
        //Just wait for button to be released to avoid sending multiple messages
        }
      digitalWrite(LED1,LOW);  // Turn LED OFF
    }
    
}
