// Pin assignments
const int stdby = 1;
const int a1 = 2;
const int a2 = 4;
const int pwma = 26;
const int PIN = 5;

// GLOBAL VARIABLES
float wall_position = 3350.0;
float control_signal = 0.0;
int Kp = 35;

void setup() {
  Serial.begin(9600);
  //while (!Serial) {
    ; // Wait for Serial to be ready
  //}

  Serial.println("System starting...");

  pinMode(PIN, OUTPUT);
  pinMode(stdby, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(pwma, OUTPUT);
  digitalWrite(stdby, HIGH);
}

void loop() {
  int current_position = analogRead(A1);

  // Print raw sensor reading
  Serial.print("Sensor reading: ");
  Serial.println(current_position);

  
  delay(200); // slow down prints
}
