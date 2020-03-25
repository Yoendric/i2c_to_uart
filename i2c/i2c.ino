#include <Wire.h>
#include <SoftwareSerial.h>

volatile byte RecvArray[8];
volatile byte PzemRead[25];

void setup()
{
 Wire.begin(10);                // join i2c bus with address #2
 Wire.onReceive(recvEvent); // register event
 Wire.onRequest(requestEvent);
 Serial.begin(9600);           // start serial for output
 Serial1.begin(9600);
}

void loop()
{
  delay(10);
  UART_read();
}

void recvEvent(int howMany)
{
  int Pointer = 0;
  while(1 < Wire.available()) // loop through all but the last
  {
    RecvArray[Pointer]=Wire.read();
    Pointer++;
  }
  RecvArray[Pointer] = Wire.read();    // receive byte as an integer
  if (RecvArray[Pointer] != 255){
    Serial.println("I was recived to write");         // print the character
    UART_write();
  }else{
    Serial.println("Scan address");
  }  
}
void requestEvent(int howMany)
{
  Serial.println("I was requested to send");
  for (int i=0;i<sizeof(PzemRead);i++){
    Wire.write(PzemRead[i]);
  } 
}
void UART_write(void)
{
  for(int i=0;i<sizeof(RecvArray);i++){
    Serial1.write(RecvArray[i]);
  }
}
void UART_read(void)
{
  if(Serial1.available() ){ // if new data is coming from the HW Serial
    delay(40);
    int Pointer = 0;
    while(Serial1.available())          // reading data into char array
    {
     PzemRead[Pointer]=Serial1.read();
     Pointer++;
    } 
  }
}
