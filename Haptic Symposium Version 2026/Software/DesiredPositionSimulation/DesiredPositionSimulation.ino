// Pin assignments
const int stdby = 1;
const int a1 = 2;
const int a2 = 4;
const int pwma = 26;   //pwm pin
const int sensorPin = A1;     //sensor pin output

// GLOBAL VARIABLES

// Time variables
long curr_time = 0; 
long prev_time = 0;
const int interval = 10; //loop rate in ms (100 Hz)

// Simulation Variables
float desired_angle = 0; //Where it is in reference to angle on the paddle
float angle_bound = 5; //bound to prevent noise at center

// Necessary Constants
float ang_velocity = 0.0;
float des_velocity = 0.0;
float error = 0; //initial angle rror
float vel_error = 0; //initialize velocity error
float current_angle = 0.0;
float past_angle = 0.0;

// Control Gains
float control_signal = 0.0;
int Kp = 7;
int Kd = 0.01;

// ---------- FUNCTIONS ----------

// Initialize motor driver
void init_driver() {
  pinMode(stdby, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  digitalWrite(stdby, HIGH);  // enable driver
  pinMode(pwma, OUTPUT);
}

// Drive motor
void drive(int speed) {
  if (speed > 0) {
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
  } else {
    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
  }
  speed = abs(speed);

  analogWrite(pwma, speed);

}

// Controller
float control(float kp, float kd, int desired_angle, float velocity, float position, float control_signal) {
  float maxspeed = 255.0;  // Arduino PWM max
  float max_error = 45; //max error in degrees

  if (abs(position) < angle_bound) { // if paddle is in bound, prevents jittering for a tight desired angle
    control_signal = 0.0;
  } else if (position > angle_bound) { // if positive angle
    error = position - desired_angle;
    vel_error = velocity - des_velocity;
    control_signal = (kp*error + kd*vel_error);
  } else { // if negative angle
    error = position - desired_angle;
    vel_error = velocity - des_velocity;
    control_signal = (kp*error + kd*vel_error);
  }

  // constraining control signal to max PWM input 
  if (control_signal > maxspeed) {
    control_signal = maxspeed;
  } else if ((control_signal < 0 ) && (abs(control_signal) > maxspeed)) {
    control_signal = -maxspeed;
  } else {
    control_signal = control_signal;
  }

  return control_signal;
}

// ---------- MAIN LOOP ----------
void setup() {
  Serial.begin(9600);

  // Print for debugging
  // Serial.println("System starting...");

  pinMode(sensorPin, OUTPUT); //init sensor
  digitalWrite(stdby, HIGH);

  init_driver();
}

void loop() {
  curr_time = millis();
  if ((curr_time - prev_time) >= interval) {

    int current_position_field = analogRead(sensorPin);  // ADC input (connect sensor to A1)
    float current_position_angle = 0.5321*current_position_field - 431.87; //INPUT CALIBRATION EQUATION, converting sensor reading to degrees

    current_angle = current_position_angle;
    ang_velocity = (current_angle - past_angle)/(interval/1000.0);
      
    control_signal = control(Kp, Kd, desired_angle, ang_velocity, current_position_angle, control_signal);

    drive((int)control_signal);

    // Print for debugging
      // Print raw sensor reading
      // Serial.print("Sensor reading: ");
      // Serial.println(current_position_angle);

      // Print control signal too
      // Serial.print("Control signal: ");
      // Serial.println(control_signal);
    // end debugging print
    
    past_angle = current_angle;

  }
  

}

