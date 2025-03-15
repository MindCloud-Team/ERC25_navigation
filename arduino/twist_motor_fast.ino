// Motor pins
#define r_pwm 3 
#define r_dir 2
#define l_pwm 9
#define l_dir 8

const int maxSpeed = 255; // maximum pwm for motor
const float wheel_base = 0.5; //distance between left and right wheels
double linear_x = 0;
double angular_z = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(r_pwm, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(l_dir, OUTPUT);
  pinMode(l_pwm, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    // Check if data contains both required parameters
    if (data.indexOf("linear_x") != -1 && data.indexOf("angular_z") != -1) {
      // Try to parse as proper JSON first
      if (data.startsWith("{") && data.endsWith("}")) {
        parseJsonFormat(data);
      } else {
        // Try to parse with more flexible approach
        parseSimpleFormat(data);
      }
      
      // Convert Twist to motor speeds
      int L_velocity = int((linear_x - (angular_z * wheel_base) / 2.0) * maxSpeed);
      int R_velocity = int((linear_x + (angular_z * wheel_base) / 2.0) * maxSpeed);
      
      // Limit the speed from range of (-255 --> 255)
      L_velocity = constrain(L_velocity, -maxSpeed, maxSpeed);
      R_velocity = constrain(R_velocity, -maxSpeed, maxSpeed);

      // Control the motors
      controlMotors(L_velocity, R_velocity);
    }
  }
}

// Parse proper JSON format: {"linear_x":0.5,"angular_z":0.0}
void parseJsonFormat(String data) {
  int linearStart = data.indexOf("\"linear_x\":");
  int angularStart = data.indexOf("\"angular_z\":");
  
  if (linearStart != -1 && angularStart != -1) {
    // Extract linear_x value
    int linearValueStart = linearStart + 10; // Length of "\"linear_x\":"
    int linearValueEnd = data.indexOf(",", linearValueStart);
    if (linearValueEnd == -1) linearValueEnd = data.indexOf("}", linearValueStart);
    String linearStr = data.substring(linearValueStart, linearValueEnd);
    linear_x = linearStr.toInt() / 1000.0;
    
    // Extract angular_z value
    int angularValueStart = angularStart + 11; // Length of "\"angular_z\":"
    int angularValueEnd = data.indexOf("}", angularValueStart);
    if (angularValueEnd == -1) angularValueEnd = data.length();
    String angularStr = data.substring(angularValueStart, angularValueEnd);
    angular_z = angularStr.toInt() / 1000.0;
  }
}

// Parse simple format: "linear_x":0.5"angular_z":0.0
void parseSimpleFormat(String data) {
  int linearStart = data.indexOf("linear_x");
  int angularStart = data.indexOf("angular_z");
  
  if (linearStart != -1) {
    // Find the colon after "linear_x"
    int colonPos = data.indexOf(":", linearStart);
    if (colonPos != -1) {
      // Find the next quote or the start of angular_z
      int endPos = data.indexOf("\"", colonPos);
      if (endPos == -1 || (angularStart != -1 && endPos > angularStart)) {
        endPos = angularStart;
      }
      if (endPos == -1) endPos = data.length();
      
      // Extract the value
      String linearStr = data.substring(colonPos + 1, endPos);
      linearStr.trim();
      linear_x = linearStr.toInt() / 1000.0;
    }
  }
  
  if (angularStart != -1) {
    // Find the colon after "angular_z"
    int colonPos = data.indexOf(":", angularStart);
    if (colonPos != -1) {
      // Extract until the end or next quote
      int endPos = data.indexOf("\"", colonPos);
      if (endPos == -1) endPos = data.length();
      
      // Extract the value
      String angularStr = data.substring(colonPos + 1, endPos);
      angularStr.trim();
      angular_z = angularStr.toInt() / 1000.0;
    }
  }
}

void controlMotors(int L_pwm, int R_pwm) {
  if (L_pwm > 0) {
    // Move left motors forward with specified pwm
    analogWrite(l_pwm, L_pwm);
    digitalWrite(l_dir, LOW);
  } else {
    // Move left motors backward with specified pwm
    analogWrite(l_pwm, abs(L_pwm));
    digitalWrite(l_dir, HIGH);
  }
  
  if (R_pwm > 0) {
    // Move right motors forward with specified pwm
    analogWrite(r_pwm, R_pwm);
    digitalWrite(r_dir, LOW);
  } else {
    // Move right motors backward with specified pwm
    analogWrite(r_pwm, abs(R_pwm));
    digitalWrite(r_dir, HIGH);
  }
}