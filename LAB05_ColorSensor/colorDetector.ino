const int PIN_RED   = 9;   
const int PIN_BLUE  = 10;  
const int PIN_GREEN = 11;  
const int PIN_LDR   = A0;
const bool COMMON_ANODE = true;  

void setColor(bool rOn, bool gOn, bool bOn) {
  int rVal = rOn ? 255 : 0;
  int gVal = gOn ? 255 : 0;
  int bVal = bOn ? 255 : 0;

  if (COMMON_ANODE) {      //common anode
    rVal = 255 - rVal;
    gVal = 255 - gVal;
    bVal = 255 - bVal;
  }

  analogWrite(PIN_RED,   rVal);
  analogWrite(PIN_GREEN, gVal);
  analogWrite(PIN_BLUE,  bVal);
}


int measureOne(bool rOn, bool gOn, bool bOn) {
  setColor(rOn, gOn, bOn);
  delay(200);              

  long sum = 0;
  const int N = 10;
  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_LDR);
    delay(5);
  }
  return (int)(sum / N);
}


String detectColor(int r, int g, int b) {
  // Check for RED
  if (r >= 260) {
    return "RED";
  }

  // Check for GREEN
  if (r >= 185 && r <= 200 &&
      g >= 170 && g <= 210 &&
      b >= 100 && b <= 130) {
    return "GREEN";
  }

  // Check for BLUE
  if (r >= 80 && r <= 200 &&
      g >= 70 && g <= 150 &&
      b >= 58 && b <= 120) {
    return "BLUE";
  }
  return "UNKNOWN";
}



void setup() {
  Serial.begin(9600);
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);
}

void loop() {
  // Measure each channel with only that LED on
  int r = measureOne(true,  false, false);  // red LED ON
  int g = measureOne(false, true,  false);  // green LED ON
  int b = measureOne(false, false, true);   // blue LED ON

  setColor(false, false, false);            // all off

  String col = detectColor(r, g, b);

  Serial.print(r);
  Serial.print(",");
  Serial.print(g);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.println(col);

  delay(300);
}
