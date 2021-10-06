int Sensor = 22;  
int LED = 3; 

void setup() {
  Serial.begin(9600);
  pinMode (Sensor, INPUT); 
  pinMode (LED, OUTPUT);   
  Serial.println("Waiting for motion");
}

void loop() {
     int val = digitalRead(Sensor); //Read Pin as input
     if((val > 0) && (flg==0))
     {
        digitalWrite(LED, HIGH);
        Serial.println("Motion Detected");
        flg = 1;
     }
     if(val == 0)
     {
        digitalWrite(LED, LOW);
        Serial.println("NO Motion"); 
        flg = 0;
     } 
