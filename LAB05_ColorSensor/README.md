# LAB05 Color Sensor

This lab uses an RGB LED and an LDR (photoresistor) to detect object color by reflectance.
The Arduino turns on one LED channel at a time (R, G, B), reads reflected light with the LDR, and classifies the color using threshold rules.

## Script

File: `colorDetector.ino`

What it does:
- Drives RGB LED on PWM pins `9` (R), `10` (B), `11` (G).
- Reads LDR value from analog pin `A0`.
- For each channel, turns on only one LED and takes 10 averaged ADC samples.
- Prints serial output as `r,g,b,label` at `9600` baud.
- Classifies into `RED`, `GREEN`, `BLUE`, or `UNKNOWN`.

Note:
- `COMMON_ANODE = true` in the code, so PWM values are inverted for a common-anode RGB LED.

## Experiment Setup

Hardware:
- Arduino board (e.g., Uno)
- RGB LED
- LDR + resistor (voltage divider)
- Jumper wires and breadboard

Wiring (according to the script):
- RGB LED red pin -> `D9`
- RGB LED blue pin -> `D10`
- RGB LED green pin -> `D11`
- LDR divider output -> `A0`
- Common grounds connected

Procedure:
1. Upload `colorDetector.ino`.
2. Open Serial Monitor at `9600` baud.
3. Place colored objects in front of the sensor/LED at fixed distance.
4. Observe lines like: `192,184,116,GREEN`.
5. If needed, tune thresholds in `detectColor()` for your lighting conditions.

## Output Format

Each line in Serial Monitor:

`<R_reading>,<G_reading>,<B_reading>,<DetectedLabel>`

Example:

`278,153,95,RED`
