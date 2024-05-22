#define IR_TX 13
#define IR_RX A1

volatile int blanc = 0;
volatile int mesure = 0;
volatile int ancienne_mesure = mesure - blanc;

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
  delay(100);
  blanc = analogRead(IR_RX);
  // delay(10);


  //mesure
  digitalWrite(IR_TX, HIGH);
  delay(100);
  mesure = analogRead(IR_RX) - blanc;
  // delay(10);

  Serial.println(mesure);// - ancienne_mesure);
  ancienne_mesure = mesure;
  // delay(500);
  
  // if (mesure < 15) Serial.println("Franchissement --------------------------------");
  // if (mesure > 22) Serial.println("Franchissement ********************************");


  
}
