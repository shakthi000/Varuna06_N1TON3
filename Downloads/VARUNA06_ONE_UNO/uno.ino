#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ============================================================
// VARUNA06 — SINGLE ARDUINO UNO VERSION
// ============================================================
//
// N1 = SIMULATED MAGNETOMETER
// N2 = REAL TX COIL
// N3 = REAL RX COIL
// ADS1115 = RX ADC
//
// UNO USB SERIAL -> FLASK -> DASHBOARD
//
// No ESP32
// No I/Q
// No phase
// ============================================================


// ============================================================
// ADS1115
// ============================================================

Adafruit_ADS1115 ads;

bool adsAvailable = false;


// ============================================================
// PINS
// ============================================================

// TX coil driver control
const int TX_PIN = 9;

// Optional status LED
const int LED_PIN = 13;


// ============================================================
// FREQUENCIES
// ============================================================

const float frequencies[] = {
  250.0,
  750.0,
  1750.0
};

const int NUM_FREQ = 3;

int frequencyIndex = 0;

float currentFrequency = 250.0;


// ============================================================
// TX STATE
// ============================================================

bool txEnabled = true;


// ============================================================
// TIMING
// ============================================================

unsigned long lastTelemetry = 0;

unsigned long sampleNumber = 0;

const unsigned long TELEMETRY_INTERVAL = 250;


// ============================================================
// BASELINE COMMAND
// ============================================================

bool baselineRequested = false;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(TX_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  // ----------------------------------------------------------
  // ADS1115
  // ----------------------------------------------------------

  if (ads.begin()) {

    adsAvailable = true;

    // Gain = +/-4.096V
    //
    // This gives a useful measurement range
    // for a conditioned RX signal.

    ads.setGain(GAIN_ONE);

  }
  else {

    adsAvailable = false;
  }


  // ----------------------------------------------------------
  // START TX
  // ----------------------------------------------------------

  startTX();


  // ----------------------------------------------------------
  // READY EVENT
  // ----------------------------------------------------------

  sendEvent("UNO_READY");


  // ----------------------------------------------------------
  // ADS STATUS
  // ----------------------------------------------------------

  if (adsAvailable) {

    sendEvent("ADS1115_READY");

  }
  else {

    sendEvent("ADS1115_ERROR");
  }
}


// ============================================================
// START TX
// ============================================================

void startTX() {

  txEnabled = true;

  tone(
    TX_PIN,
    (unsigned int)currentFrequency
  );

  digitalWrite(
    LED_PIN,
    HIGH
  );
}


// ============================================================
// STOP TX
// ============================================================

void stopTX() {

  txEnabled = false;

  noTone(TX_PIN);

  digitalWrite(
    LED_PIN,
    LOW
  );
}


// ============================================================
// CHANGE FREQUENCY
// ============================================================

void setFrequency(float f) {

  currentFrequency = f;

  if (txEnabled) {

    tone(
      TX_PIN,
      (unsigned int)currentFrequency
    );
  }
}


// ============================================================
// READ ADS1115 RX
// ============================================================

float readRXVoltage() {

  if (!adsAvailable) {

    return 0.0;
  }


  int16_t raw =
      ads.readADC_SingleEnded(0);


  float voltage =
      ads.computeVolts(raw);


  return voltage;
}


// ============================================================
// OPTIONAL REFERENCE CHANNEL
// ============================================================

float readReferenceVoltage() {

  if (!adsAvailable) {

    return 0.0;
  }


  int16_t raw =
      ads.readADC_SingleEnded(1);


  return ads.computeVolts(raw);
}


// ============================================================
// SIMULATED N1 MAGNETOMETER
// ============================================================
//
// N1 is intentionally simulated.
// Later this function can be replaced with
// the actual magnetometer reading.
//

float readSimulatedMagnetometer() {

  float t =
      millis() / 1000.0;


  // Slowly changing environmental field

  float magnetic =
      48.0
      + sin(t * 0.35) * 1.5
      + sin(t * 0.11) * 0.7;


  // Occasional synthetic anomaly

  int cycle =
      ((int)t) % 40;


  if (
    cycle >= 25 &&
    cycle <= 28
  ) {

    magnetic += 12.0;
  }


  return magnetic;
}


// ============================================================
// NOISE ESTIMATION
// ============================================================

float estimateNoise(
    float rx
) {

  static float previous = 0.0;

  float noise =
      fabs(rx - previous);

  previous = rx;

  return noise;
}


// ============================================================
// EVENT
// ============================================================

void sendEvent(
    const char* eventName
) {

  Serial.print(
    "{\"system\":\"VARUNA06\",\"event\":\""
  );

  Serial.print(eventName);

  Serial.println(
    "\"}"
  );
}


// ============================================================
// TELEMETRY
// ============================================================

void sendTelemetry() {

  // ----------------------------------------------------------
  // READ N1
  // ----------------------------------------------------------

  float magnetic =
      readSimulatedMagnetometer();


  // ----------------------------------------------------------
  // READ N3
  // ----------------------------------------------------------

  float rx =
      readRXVoltage();


  float reference =
      readReferenceVoltage();


  float noise =
      estimateNoise(rx);


  // ----------------------------------------------------------
  // SIMPLE EM CHANGE
  // ----------------------------------------------------------
  //
  // The Flask server performs the main baseline calculation.
  // Arduino sends raw measurements here.
  //

  float emChange =
      0.0;


  if (reference > 0.000001) {

    emChange =
      ((rx - reference) /
       reference) * 100.0;
  }


  // ----------------------------------------------------------
  // N2 FREQUENCY
  // ----------------------------------------------------------

  currentFrequency =
      frequencies[frequencyIndex];


  // ----------------------------------------------------------
  // JSON
  // ----------------------------------------------------------

  Serial.print("{");

  Serial.print(
    "\"system\":\"VARUNA06\","
  );

  Serial.print(
    "\"timestamp\":"
  );

  Serial.print(
    millis()
  );

  Serial.print(",");

  Serial.print(
    "\"sample\":"
  );

  Serial.print(
    sampleNumber
  );

  Serial.print(",");


  // ----------------------------------------------------------
  // N1
  // ----------------------------------------------------------

  Serial.print(
    "\"N1\":{"
  );

  Serial.print(
    "\"available\":true,"
  );

  Serial.print(
    "\"simulated\":true,"
  );

  Serial.print(
    "\"magnitude\":"
  );

  Serial.print(
    magnetic,
    3
  );

  Serial.print(
    "},"
  );


  // ----------------------------------------------------------
  // N2
  // ----------------------------------------------------------

  Serial.print(
    "\"N2\":{"
  );

  Serial.print(
    "\"tx\":"
  );

  Serial.print(
    txEnabled ? 1 : 0
  );

  Serial.print(",");

  Serial.print(
    "\"frequency\":"
  );

  Serial.print(
    currentFrequency,
    1
  );

  Serial.print(
    "},"
  );


  // ----------------------------------------------------------
  // N3
  // ----------------------------------------------------------

  Serial.print(
    "\"N3\":{"
  );

  Serial.print(
    "\"rx_v\":"
  );

  Serial.print(
    rx,
    6
  );

  Serial.print(",");

  Serial.print(
    "\"reference_v\":"
  );

  Serial.print(
    reference,
    6
  );

  Serial.print(",");

  Serial.print(
    "\"noise_v\":"
  );

  Serial.print(
    noise,
    6
  );

  Serial.print(
    "},"
  );


  // ----------------------------------------------------------
  // ANALYTICS
  // ----------------------------------------------------------

  Serial.print(
    "\"analytics\":{"
  );

  Serial.print(
    "\"em_change_percent\":"
  );

  Serial.print(
    emChange,
    2
  );

  Serial.print(
    "}"
  );


  Serial.println(
    "}"
  );


  sampleNumber++;
}


// ============================================================
// HANDLE COMMAND
// ============================================================

void handleCommand(
    String command
) {

  command.trim();


  // ----------------------------------------------------------
  // RESET BASELINE
  // ----------------------------------------------------------

  if (
    command ==
    "RESET_BASELINE"
  ) {

    sendEvent(
      "BASELINE_START"
    );


    delay(100);


    sendEvent(
      "BASELINE_READY"
    );

    return;
  }


  // ----------------------------------------------------------
  // TX ON
  // ----------------------------------------------------------

  if (
    command ==
    "TX_ON"
  ) {

    startTX();

    return;
  }


  // ----------------------------------------------------------
  // TX OFF
  // ----------------------------------------------------------

  if (
    command ==
    "TX_OFF"
  ) {

    stopTX();

    return;
  }
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // RECEIVE COMMANDS FROM PYTHON
  // ----------------------------------------------------------

  if (
    Serial.available()
  ) {

    String command =
        Serial.readStringUntil(
          '\n'
        );

    handleCommand(
      command
    );
  }


  // ----------------------------------------------------------
  // MULTI-FREQUENCY SCAN
  // ----------------------------------------------------------

  if (
    millis() -
    lastTelemetry >=
    TELEMETRY_INTERVAL
  ) {

    lastTelemetry =
        millis();


    // Change frequency every telemetry cycle

    frequencyIndex++;

    if (
      frequencyIndex >=
      NUM_FREQ
    ) {

      frequencyIndex = 0;
    }


    setFrequency(
      frequencies[
        frequencyIndex
      ]
    );


    sendTelemetry();
  }
}