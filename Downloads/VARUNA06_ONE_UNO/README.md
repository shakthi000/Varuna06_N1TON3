# VARUNA06 — ONE UNO Architecture

## What changed

The ESP32/Wi-Fi layer is removed.

The final software path is:

Arduino Uno
  -> USB serial
  -> Laptop
  -> Flask
  -> Browser dashboard

The single Uno now handles:

- N1 HMC5883L magnetic sensing
- N2 TX coil frequency generation
- N3 RX coil + ADS1115
- multi-frequency scanning: 40 / 80 / 160 Hz
- baseline calibration
- I/Q synchronous demodulation
- phase signature
- magnetic + EM fusion score
- adaptive confirmation rescan
- JSON telemetry

## Files

- `arduino/varuna06_one_uno.ino` — upload to the Arduino Uno.
- `app.py` — Flask server + direct COM3 serial reader.
- `templates/dashboard.html` — animated dashboard.
- `requirements.txt` — Python dependencies.

## Run

1. Close Arduino Serial Monitor before starting Flask. Only one program can normally own COM3 at a time.
2. Install dependencies:

   `pip install -r requirements.txt`

3. Upload the Arduino sketch.
4. Keep the target away during startup so the Uno can collect the baseline.
5. Run:

   `python app.py`

6. Open:

   `http://127.0.0.1:5000`

## Important hardware signal condition

The RX coil must not be connected directly to an out-of-range ADS1115 input.

The receiver front end should provide a safe, biased signal within the ADS1115 input range. The I/Q algorithm assumes the measured waveform is the AC response from the RX coil.

## Judge-facing features

The dashboard visibly demonstrates:

1. Multi-frequency fingerprinting
2. I/Q components
3. Phase signature
4. Magnetic + EM fusion
5. Baseline correction
6. Adaptive confirmation rescan

Do not claim that one phase angle universally identifies a specific metal. Present phase as one feature in a multi-sensor classification/fusion system.
