from flask import Flask, jsonify, render_template, Response
import serial
import threading
import time
import json
import csv
import io
import math

try:
    import numpy as np
    from sklearn.ensemble import IsolationForest
    from sklearn.preprocessing import StandardScaler

    ML_AVAILABLE = True

except Exception:
    ML_AVAILABLE = False


# ============================================================
# APP
# ============================================================

app = Flask(__name__)


# ============================================================
# SERIAL CONFIG
# ============================================================

SERIAL_PORT = "COM3"

BAUD_RATE = 115200


# ============================================================
# SETTINGS
# ============================================================

WINDOW_SIZE = 180

EVENT_LIMIT = 60

ML_MIN_SAMPLES = 50


# ============================================================
# STATE
# ============================================================

serial_connection = None

serial_lock = threading.Lock()

last_packet_time = 0


history = []

telemetry_history = []

events = []


# ============================================================
# BASELINE
# ============================================================

baseline_values = []

baseline_v = None

baseline_magnetic = None

BASELINE_SAMPLES = 30


# ============================================================
# LATEST
# ============================================================

latest = {
    "system": "VARUNA06",

    "timestamp": 0,

    "sample": 0,

    "N1": {
        "available": True,
        "simulated": True,
        "magnitude": 0
    },

    "N2": {
        "tx": 0,
        "frequency": 0
    },

    "N3": {
        "rx_v": 0,
        "reference_v": 0,
        "noise_v": 0
    },

    "analytics": {
        "baseline_v": 0,
        "em_change_percent": 0,
        "magnetic_change_percent": 0,
        "fusion_score": 0,
        "anomaly": False,
        "confirmed": False,
        "ml_score": 0,
        "ml_anomaly": False
    }
}


# ============================================================
# ML
# ============================================================

ml_model = None

ml_scaler = None

ml_training_data = []


# ============================================================
# HELPERS
# ============================================================

def safe_float(value, default=0):

    try:
        return float(value)

    except Exception:
        return default


def add_event(
    event_type,
    message
):

    events.insert(
        0,
        {
            "time": time.strftime("%H:%M:%S"),
            "type": event_type,
            "message": message
        }
    )

    del events[EVENT_LIMIT:]


# ============================================================
# FEATURES
# ============================================================

def make_features(packet):

    n1 = packet.get(
        "N1",
        {}
    )

    n2 = packet.get(
        "N2",
        {}
    )

    n3 = packet.get(
        "N3",
        {}
    )

    a = packet.get(
        "analytics",
        {}
    )


    return [

        safe_float(
            n2.get("frequency")
        ),

        safe_float(
            n3.get("rx_v")
        ),

        safe_float(
            n3.get("reference_v")
        ),

        safe_float(
            n3.get("noise_v")
        ),

        safe_float(
            n1.get("magnitude")
        ),

        safe_float(
            a.get("em_change_percent")
        ),

        safe_float(
            a.get(
                "magnetic_change_percent"
            )
        )

    ]


# ============================================================
# TRAIN ML
# ============================================================

def train_ml():

    global ml_model
    global ml_scaler

    if not ML_AVAILABLE:
        return False

    if len(
        ml_training_data
    ) < ML_MIN_SAMPLES:
        return False


    try:

        X = np.array(
            ml_training_data,
            dtype=float
        )


        ml_scaler = StandardScaler()

        X_scaled = (
            ml_scaler.fit_transform(X)
        )


        ml_model = IsolationForest(
            n_estimators=120,
            contamination=0.05,
            random_state=42
        )


        ml_model.fit(
            X_scaled
        )


        return True


    except Exception as e:

        print(
            "ML training error:",
            e
        )

        ml_model = None

        ml_scaler = None

        return False


# ============================================================
# RUN ML
# ============================================================

def run_ml(packet):

    if not ML_AVAILABLE:

        return {
            "score": 0,
            "anomaly": False
        }


    features = make_features(
        packet
    )


    if (
        ml_model is None
        and
        len(
            ml_training_data
        ) < ML_MIN_SAMPLES
    ):

        ml_training_data.append(
            features
        )


    if (
        ml_model is None
        and
        len(
            ml_training_data
        ) >= ML_MIN_SAMPLES
    ):

        train_ml()


    if (
        ml_model is None
        or
        ml_scaler is None
    ):

        return {
            "score": 0,
            "anomaly": False
        }


    try:

        X = np.array(
            [features],
            dtype=float
        )


        X_scaled = (
            ml_scaler.transform(X)
        )


        decision = float(
            ml_model
            .decision_function(
                X_scaled
            )[0]
        )


        prediction = int(
            ml_model
            .predict(
                X_scaled
            )[0]
        )


        score = max(
            0,
            min(
                100,
                50 -
                decision * 100
            )
        )


        return {
            "score": score,
            "anomaly":
                prediction == -1
        }


    except Exception:

        return {
            "score": 0,
            "anomaly": False
        }


# ============================================================
# PROCESS PACKET
# ============================================================

def process_packet(packet):

    global latest

    global last_packet_time

    global baseline_v

    global baseline_magnetic


    if (
        packet.get("system")
        != "VARUNA06"
    ):

        return


    last_packet_time = time.time()


    # ========================================================
    # EVENTS
    # ========================================================

    event = packet.get(
        "event"
    )


    if event:

        if event == "UNO_READY":

            add_event(
                "SYSTEM",
                "Arduino UNO sensor engine ready"
            )


        elif event == "ADS1115_READY":

            add_event(
                "SYSTEM",
                "ADS1115 receiver ADC detected"
            )


        elif event == "ADS1115_ERROR":

            add_event(
                "SYSTEM",
                "ADS1115 not detected"
            )


        elif event == "BASELINE_START":

            add_event(
                "BASELINE",
                "Baseline calibration started"
            )


        elif event == "BASELINE_READY":

            add_event(
                "BASELINE",
                "Baseline calibration complete"
            )


        return


    # ========================================================
    # DATA
    # ========================================================

    n1 = packet.get(
        "N1",
        {}
    )

    n2 = packet.get(
        "N2",
        {}
    )

    n3 = packet.get(
        "N3",
        {}
    )


    rx = safe_float(
        n3.get("rx_v")
    )


    reference = safe_float(
        n3.get("reference_v")
    )


    noise = safe_float(
        n3.get("noise_v")
    )


    magnetic = safe_float(
        n1.get("magnitude")
    )


    frequency = safe_float(
        n2.get("frequency")
    )


    # ========================================================
    # BASELINE
    # ========================================================

    if baseline_v is None:

        baseline_values.append(
            rx
        )


        if len(
            baseline_values
        ) >= BASELINE_SAMPLES:

            baseline_v = sum(
                baseline_values
            ) / len(
                baseline_values
            )


            baseline_magnetic = magnetic

            add_event(
                "BASELINE",
                "Automatic RX baseline established"
            )


    # ========================================================
    # EM CHANGE
    # ========================================================

    if (
        baseline_v is not None
        and
        abs(baseline_v) > 0.000001
    ):

        em_change = (
            abs(
                rx -
                baseline_v
            )
            /
            abs(baseline_v)
        ) * 100


    else:

        em_change = 0


    # ========================================================
    # MAGNETIC CHANGE
    # ========================================================

    if (
        baseline_magnetic is not None
        and
        abs(baseline_magnetic) > 0.000001
    ):

        magnetic_change = (
            abs(
                magnetic -
                baseline_magnetic
            )
            /
            abs(baseline_magnetic)
        ) * 100


    else:

        magnetic_change = 0


    # ========================================================
    # HARDWARE FUSION
    # ========================================================

    em_component = min(
        100,
        em_change * 2
    )


    magnetic_component = min(
        100,
        magnetic_change * 2
    )


    fusion_score = (
        em_component * 0.70
        +
        magnetic_component * 0.30
    )


    hardware_anomaly = (
        fusion_score >= 55
    )


    # ========================================================
    # TEMP PACKET
    # ========================================================

    packet.setdefault(
        "analytics",
        {}
    )


    packet["analytics"][
        "baseline_v"
    ] = (
        baseline_v
        if baseline_v is not None
        else 0
    )


    packet["analytics"][
        "em_change_percent"
    ] = em_change


    packet["analytics"][
        "magnetic_change_percent"
    ] = magnetic_change


    packet["analytics"][
        "fusion_score"
    ] = fusion_score


    # ========================================================
    # ML
    # ========================================================

    ml = run_ml(
        packet
    )


    packet["analytics"][
        "ml_score"
    ] = ml["score"]


    packet["analytics"][
        "ml_anomaly"
    ] = ml["anomaly"]


    # ========================================================
    # FINAL ANOMALY
    # ========================================================

    final_anomaly = (
        hardware_anomaly
        or
        ml["anomaly"]
    )


    packet["analytics"][
        "anomaly"
    ] = final_anomaly


    packet["analytics"][
        "confirmed"
    ] = (
        hardware_anomaly
        and
        ml["anomaly"]
    )


    # ========================================================
    # UPDATE LATEST
    # ========================================================

    with serial_lock:

        latest = packet


        history.append(
            {
                "frequency":
                    frequency,

                "rx":
                    rx,

                "fusion":
                    fusion_score,

                "ml":
                    ml["score"],

                "magnetic":
                    magnetic
            }
        )


        if len(history) > WINDOW_SIZE:

            del history[
                :-WINDOW_SIZE
            ]


        telemetry_history.append(
            {
                "time":
                    packet.get(
                        "timestamp",
                        0
                    ),

                "sample":
                    packet.get(
                        "sample",
                        0
                    ),

                "frequency":
                    frequency,

                "rx_v":
                    rx,

                "reference_v":
                    reference,

                "noise_v":
                    noise,

                "baseline_v":
                    baseline_v
                    if baseline_v is not None
                    else 0,

                "em_change_percent":
                    em_change,

                "magnetic":
                    magnetic,

                "magnetic_change_percent":
                    magnetic_change,

                "fusion_score":
                    fusion_score,

                "ml_score":
                    ml["score"],

                "anomaly":
                    final_anomaly,

                "confirmed":
                    packet["analytics"][
                        "confirmed"
                    ]
            }
        )


        if len(
            telemetry_history
        ) > 3000:

            del telemetry_history[
                :-3000
            ]


        # ====================================================
        # EVENT
        # ====================================================

        if final_anomaly:

            add_event(
                "ANOMALY",
                (
                    f"{frequency:.0f} Hz • "
                    f"RX {rx:.4f} V • "
                    f"EM Δ {em_change:.1f}% • "
                    f"Fusion {fusion_score:.0f}%"
                )
            )


# ============================================================
# SERIAL THREAD
# ============================================================

def serial_reader():

    global serial_connection


    while True:

        try:

            if (
                serial_connection
                is None
            ):

                print(
                    "Opening",
                    SERIAL_PORT
                )


                serial_connection = serial.Serial(
                    SERIAL_PORT,
                    BAUD_RATE,
                    timeout=1
                )


                time.sleep(2)


                print(
                    "UNO connected"
                )


            line = (
                serial_connection
                .readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )


            if not line:
                continue


            if not line.startswith("{"):
                continue


            try:

                packet = json.loads(
                    line
                )


                process_packet(
                    packet
                )


            except Exception as e:

                print(
                    "JSON error:",
                    e,
                    line
                )


        except Exception as e:

            print(
                "Serial error:",
                e
            )


            try:

                if serial_connection:
                    serial_connection.close()

            except Exception:
                pass


            serial_connection = None


            time.sleep(2)


# ============================================================
# DASHBOARD
# ============================================================

@app.route("/")
def dashboard():

    return render_template(
        "dashboard.html"
    )


# ============================================================
# DATA API
# ============================================================

@app.route("/data")
def data():

    connected = (
        time.time() -
        last_packet_time
    ) < 3


    with serial_lock:

        return jsonify(
            {
                "raw":
                    latest,

                "history":
                    history,

                "events":
                    events,

                "connection":
                    connected,

                "gateway":
                    connected,

                "ml_available":
                    ML_AVAILABLE,

                "ml_trained":
                    ml_model is not None
            }
        )


# ============================================================
# RESET BASELINE
# ============================================================

@app.route(
    "/baseline/reset",
    methods=["POST"]
)
def reset_baseline():

    global baseline_v

    global baseline_magnetic

    baseline_v = None

    baseline_magnetic = None

    baseline_values.clear()


    add_event(
        "BASELINE",
        "Manual baseline reset requested"
    )


    return jsonify(
        {
            "success": True,
            "message":
                "Baseline reset. Collecting new baseline..."
        }
    )


# ============================================================
# EXPORT
# ============================================================

@app.route("/export")
def export():

    output = io.StringIO()

    writer = csv.writer(
        output
    )


    writer.writerow(
        [
            "time",
            "sample",
            "frequency_hz",
            "rx_v",
            "reference_v",
            "noise_v",
            "baseline_v",
            "em_change_percent",
            "magnetic",
            "magnetic_change_percent",
            "fusion_score",
            "ml_score",
            "anomaly",
            "confirmed"
        ]
    )


    with serial_lock:

        for row in telemetry_history:

            writer.writerow(
                [
                    row["time"],
                    row["sample"],
                    row["frequency"],
                    row["rx_v"],
                    row["reference_v"],
                    row["noise_v"],
                    row["baseline_v"],
                    row["em_change_percent"],
                    row["magnetic"],
                    row["magnetic_change_percent"],
                    row["fusion_score"],
                    row["ml_score"],
                    row["anomaly"],
                    row["confirmed"]
                ]
            )


    return Response(
        output.getvalue(),
        mimetype="text/csv",
        headers={
            "Content-Disposition":
                "attachment; filename=varuna06_data.csv"
        }
    )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("=" * 60)

    print(
        "VARUNA06 — SINGLE UNO + ADS1115"
    )

    print("=" * 60)

    print(
        f"Serial port: {SERIAL_PORT}"
    )

    print(
        f"ML available: {ML_AVAILABLE}"
    )

    print(
        "Dashboard: http://127.0.0.1:5000"
    )

    print("=" * 60)


    thread = threading.Thread(
        target=serial_reader,
        daemon=True
    )

    thread.start()


    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=True
    )