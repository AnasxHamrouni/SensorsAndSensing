const int emgPin = A0;

const unsigned long RELAX1_MS   = 5000;
const unsigned long PREPARE_MS  = 3000;
const unsigned long SQUEEZE_MS  = 5000;
const unsigned long RELAX2_MS   = 5000;

const int samplePeriodMs = 5;

const int maWindow = 20;
int   maBuffer[maWindow];
int   maIndex = 0;
long  maSum = 0;

long offsetSum = 0;
int  offsetCount = 0;
int  baseline = 0;

enum Phase { PHASE_RELAX1, PHASE_PREPARE, PHASE_SQUEEZE, PHASE_RELAX2, PHASE_DONE };
Phase phase = PHASE_RELAX1;

unsigned long phaseStart = 0;

void printPhaseInstruction(Phase p) {
  switch (p) {
    case PHASE_RELAX1:
      Serial.println("=== RELAX: keep muscle relaxed, calibration starts ===");
      break;
    case PHASE_PREPARE:
      Serial.println("=== GET READY: you will squeeze in 3 seconds ===");
      break;
    case PHASE_SQUEEZE:
      Serial.println("=== SQUEEZE NOW: contract muscle for 5 seconds ===");
      break;
    case PHASE_RELAX2:
      Serial.println("=== RELAX AGAIN: release muscle, keep relaxed ===");
      break;
    case PHASE_DONE:
      Serial.println("=== DONE: experiment finished, stop logging ===");
      break;
  }
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_MICRO) || defined(ARDUINO_SAMD_ZERO)
  while (!Serial) { ; }
#endif

  for (int i = 0; i < maWindow; i++) maBuffer[i] = 0;

  phaseStart = millis();
  printPhaseInstruction(phase);

  Serial.println("time_ms,phase,raw,ac,rect,envelope");
}

void loop() {
  static unsigned long lastSampleTime = 0;
  unsigned long now = millis();

  unsigned long elapsed = now - phaseStart;
  if (phase == PHASE_RELAX1 && elapsed >= RELAX1_MS) {
    phase = PHASE_PREPARE;
    phaseStart = now;
    printPhaseInstruction(phase);
  } else if (phase == PHASE_PREPARE && elapsed >= PREPARE_MS) {
    phase = PHASE_SQUEEZE;
    phaseStart = now;
    printPhaseInstruction(phase);
  } else if (phase == PHASE_SQUEEZE && elapsed >= SQUEEZE_MS) {
    phase = PHASE_RELAX2;
    phaseStart = now;
    printPhaseInstruction(phase);
  } else if (phase == PHASE_RELAX2 && elapsed >= RELAX2_MS) {
    phase = PHASE_DONE;
    phaseStart = now;
    printPhaseInstruction(phase);
  }

  if (phase == PHASE_DONE) {
    return;
  }

  if (now - lastSampleTime < samplePeriodMs) return;
  lastSampleTime = now;

  int raw = analogRead(emgPin);

  if (phase == PHASE_RELAX1) {
    offsetSum += raw;
    offsetCount++;
  }

  if (phase == PHASE_PREPARE && baseline == 0 && offsetCount > 0) {
    baseline = offsetSum / offsetCount;
    Serial.print("### Baseline computed: ");
    Serial.println(baseline);
  }

  int ac = baseline != 0 ? (raw - baseline) : 0;
  int rect = ac >= 0 ? ac : -ac;

  maSum -= maBuffer[maIndex];
  maBuffer[maIndex] = rect;
  maSum += rect;
  maIndex++;
  if (maIndex >= maWindow) maIndex = 0;

  float envelope = (float)maSum / maWindow;

  const char* phaseLabel = "";
  if      (phase == PHASE_RELAX1)  phaseLabel = "RELAX1";
  else if (phase == PHASE_PREPARE) phaseLabel = "PREPARE";
  else if (phase == PHASE_SQUEEZE) phaseLabel = "SQUEEZE";
  else if (phase == PHASE_RELAX2)  phaseLabel = "RELAX2";

  Serial.print(now);
  Serial.print(",");
  Serial.print(phaseLabel);
  Serial.print(",");
  Serial.print(raw);
  Serial.print(",");
  Serial.print(ac);
  Serial.print(",");
  Serial.print(rect);
  Serial.print(",");
  Serial.println(envelope);
}
