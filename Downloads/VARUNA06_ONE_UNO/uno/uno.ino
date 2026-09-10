#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

Adafruit_ADS1115 ads;

// ============================================================
// VARUNA06 V0
//
// REAL HARDWARE
//
// Arduino UNO
//   D9 -> TX coil
//
// RX coil
//   -> existing 2.5 V bias / coupling
//   -> ADS1115 A0
//
// ADS1115
//   A0 -> RX
//   GND -> common GND
//   VDD -> 5 V
//   SDA -> A4
//   SCL -> A5
//
// N1 IS SOFTWARE-SIMULATED IN FLASK.
// Arduino does NOT fabricate N1.
// ============================================================


// ============================================================
// PINS
// ============================================================

const int TX_PIN = 9;
const int LED_PIN = 13;


// ============================================================
// FREQUENCIES
// ============================================================

const unsigned int frequencies[] = {
  250,
  750,
  1750
};

const int NUM_FREQ = 3;

int frequencyIndex = 0;

unsigned int currentFrequency = 250;


// ============================================================
// ADS1115
// ============================================================

bool adsAvailable = false;

const adsGain_t ADS_GAIN = GAIN_ONE;


// ============================================================
// TIMING
// ============================================================

const unsigned long FREQUENCY_WINDOW_MS = 160;

unsigned long frequencyStartTime = 0;

const unsigned long TELEMETRY_INTERVAL = 180;

unsigned long lastTelemetry = 0;

unsigned long sampleNumber = 0;


// ============================================================
// ADC SAMPLING
// ============================================================
//
// ADS1115 maximum data rate is 860 SPS.
//
// We therefore use a smaller measurement window rather
// than pretending that 32 readings can be taken instantly.
//
// ============================================================

const int RX_SAMPLES = 24;

const unsigned long SAMPLE_INTERVAL_US = 1100;


// ============================================================
// TX
// ============================================================

bool txEnabled = true;


// ============================================================
// RX DATA
// ============================================================

float rxAverage = 0.0;

float rxRMS = 0.0;

float rxPeakToPeak = 0.0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(
    TX_PIN,
    OUTPUT
  );

  pinMode(
    LED_PIN,
    OUTPUT
  );

  digitalWrite(
    LED_PIN,
    LOW
  );

  Wire.begin();


  // ----------------------------------------------------------
  // ADS1115
  // ----------------------------------------------------------

  if (ads.begin()) {

    adsAvailable = true;

    ads.setGain(
      ADS_GAIN
    );

    // Maximum ADS1115 data rate.
    ads.setDataRate(
      RATE_ADS1115_860SPS
    );

  }

  else {

    adsAvailable = false;
  }


  // ----------------------------------------------------------
  // TX
  // ----------------------------------------------------------

  startTX();

  frequencyStartTime =
    millis();

  sendEvent(
    "UNO_READY"
  );

  if (adsAvailable) {

    sendEvent(
      "ADS1115_READY"
    );

  }

  else {

    sendEvent(
      "ADS1115_ERROR"
    );
  }
}


// ============================================================
// TX
// ============================================================

void startTX() {

  txEnabled = true;

  tone(
    TX_PIN,
    currentFrequency
  );

  digitalWrite(
    LED_PIN,
    HIGH
  );
}


void stopTX() {

  txEnabled = false;

  noTone(
    TX_PIN
  );

  digitalWrite(
    LED_PIN,
    LOW
  );
}


void setFrequency(
  unsigned int frequency
) {

  currentFrequency =
    frequency;

  if (txEnabled) {

    tone(
      TX_PIN,
      currentFrequency
    );
  }
}


// ============================================================
// ADS READ
// ============================================================

float readRXVoltage() {

  if (!adsAvailable) {

    return 0.0;
  }

  int16_t raw =
    ads.readADC_SingleEnded(0);

  return ads.computeVolts(
    raw
  );
}


// ============================================================
// RX MEASUREMENT
// ============================================================

void measureRXWindow() {

  if (!adsAvailable) {

    rxAverage = 0.0;
    rxRMS = 0.0;
    rxPeakToPeak = 0.0;

    return;
  }


  float samples[
    RX_SAMPLES
  ];


  float sum = 0.0;

  float minimum = 100.0;

  float maximum = -100.0;


  // ----------------------------------------------------------
  // SAMPLE
  // ----------------------------------------------------------

  for (
    int i = 0;
    i < RX_SAMPLES;
    i++
  ) {

    unsigned long start =
      micros();


    float voltage =
      readRXVoltage();


    samples[i] =
      voltage;


    sum +=
      voltage;


    if (
      voltage < minimum
    ) {

      minimum =
        voltage;
    }


    if (
      voltage > maximum
    ) {

      maximum =
        voltage;
    }


    // Keep ADC polling from becoming
    // unrealistically fast.

    while (
      micros()
      -
      start
      <
      SAMPLE_INTERVAL_US
    ) {
      // wait
    }
  }


  // ----------------------------------------------------------
  // DC / BIAS
  // ----------------------------------------------------------

  rxAverage =
    sum /
    RX_SAMPLES;


  // ----------------------------------------------------------
  // RMS AC COMPONENT
  // ----------------------------------------------------------

  float squareSum =
    0.0;


  for (
    int i = 0;
    i < RX_SAMPLES;
    i++
  ) {

    float ac =
      samples[i]
      -
      rxAverage;


    squareSum +=
      ac * ac;
  }


  rxRMS =
    sqrt(
      squareSum
      /
      RX_SAMPLES
    );


  // ----------------------------------------------------------
  // PEAK TO PEAK
  // ----------------------------------------------------------

  rxPeakToPeak =
    maximum
    -
    minimum;
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

  Serial.print(
    eventName
  );

  Serial.println(
    "\"}"
  );
}


// ============================================================
// TELEMETRY
// ============================================================

void sendTelemetry() {

  measureRXWindow();


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


  // ==========================================================
  // N1
  // ==========================================================
  //
  // N1 is intentionally NOT generated here.
  //
  // Flask derives N1 deterministically from the cleaned EM
  // evidence.
  //
  // ==========================================================

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
    "\"sensor\":\"derived_em_proxy\""
  );

  Serial.print(
    "},"
  );


  // ==========================================================
  // N2
  // ==========================================================

  Serial.print(
    "\"N2\":{"
  );

  Serial.print(
    "\"tx\":"
  );

  Serial.print(
    txEnabled
      ? 1
      : 0
  );

  Serial.print(",");

  Serial.print(
    "\"frequency\":"
  );

  Serial.print(
    currentFrequency
  );

  Serial.print(
    "},"
  );


  // ==========================================================
  // N3
  // ==========================================================

  Serial.print(
    "\"N3\":{"
  );


  Serial.print(
    "\"available\":"
  );

  Serial.print(
    adsAvailable
      ? "true"
      : "false"
  );

  Serial.print(",");


  Serial.print(
    "\"rx_v\":"
  );

  Serial.print(
    rxAverage,
    7
  );

  Serial.print(",");


  Serial.print(
    "\"rx_rms_v\":"
  );

  Serial.print(
    rxRMS,
    7
  );

  Serial.print(",");


  Serial.print(
    "\"rx_peak_to_peak_v\":"
  );

  Serial.print(
    rxPeakToPeak,
    7
  );

  Serial.print(",");


  Serial.print(
    "\"bias_expected_v\":2.5"
  );


  Serial.print(
    "},"
  );


  // ==========================================================
  // N4
  // ==========================================================

  Serial.print(
    "\"N4\":{"
  );

  Serial.print(
    "\"available\":false"
  );

  Serial.print(
    "},"
  );


  // ==========================================================
  // N5
  // ==========================================================

  Serial.print(
    "\"N5\":{"
  );

  Serial.print(
    "\"available\":false"
  );

  Serial.print(
    "},"
  );


  // ==========================================================
  // N6
  // ==========================================================

  Serial.print(
    "\"N6\":{"
  );

  Serial.print(
    "\"available\":false"
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
// COMMANDS
// ============================================================

void handleCommand(
  String command
) {

  command.trim();


  if (
    command ==
    "TX_ON"
  ) {

    startTX();

    sendEvent(
      "TX_ON"
    );

    return;
  }


  if (
    command ==
    "TX_OFF"
  ) {

    stopTX();

    sendEvent(
      "TX_OFF"
    );

    return;
  }


  if (
    command ==
    "RESET_BASELINE"
  ) {

    sendEvent(
      "BASELINE_RESET_REQUEST"
    );

    return;
  }
}


// ============================================================
// FREQUENCY CONTROL
// ============================================================

void updateFrequency() {

  unsigned long now =
    millis();


  if (
    now
    -
    frequencyStartTime
    >=
    FREQUENCY_WINDOW_MS
  ) {

    frequencyIndex++;


    if (
      frequencyIndex
      >=
      NUM_FREQ
    ) {

      frequencyIndex =
        0;
    }


    setFrequency(
      frequencies[
        frequencyIndex
      ]
    );


    frequencyStartTime =
      now;
  }
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // COMMANDS
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
  // FREQUENCY
  // ----------------------------------------------------------

  updateFrequency();


  // ----------------------------------------------------------
  // TELEMETRY
  // ----------------------------------------------------------

  if (
    millis()
    -
    lastTelemetry
    >=
    TELEMETRY_INTERVAL
  ) {

    lastTelemetry =
      millis();

    sendTelemetry();
  }
}