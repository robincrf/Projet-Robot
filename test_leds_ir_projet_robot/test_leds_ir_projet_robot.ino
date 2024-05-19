#define IR_TX 13
#define IR_RX A0

volatile int blanc = 0;
volatile int mesure = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_TX, OUTPUT);
  pinMode(IR_RX, INPUT);

  
  digitalWrite(IR_TX, LOW);
  Serial.begin(9600);
}

void loop() {
  //blanc
  digitalWrite(IR_TX, LOW);
  delay(10);
  
  blanc = analogRead(IR_RX);
 

  delay(10);


  //mesure
  digitalWrite(IR_TX, HIGH);
  delay(10);

  mesure = abs(analogRead(IR_RX) - blanc);
  Serial.println(mesure);
  
  if (mesure >= 2) Serial.println("Franchissement");

  
}
