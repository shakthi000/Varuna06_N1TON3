from flask import Flask, jsonify, render_template, Response, request
import serial
import serial.tools.list_ports
import threading
import time
import json
import csv
import io
import math
from collections import deque

app = Flask(__name__)


# ============================================================
# SERIAL
# ============================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 115200


# ============================================================
# SETTINGS
# ============================================================

WINDOW_SIZE = 240
EVENT_LIMIT = 60

BASELINE_SAMPLES = 30
FILTER_WINDOW = 5

# Number of consecutive VALID anomalous measurements
# required before the system begins treating the response
# as a possible target.
PERSISTENCE_REQUIRED = 5

# Number of consecutive clean measurements required
# before returning completely to normal.
RELEASE_REQUIRED = 8

MIN_SIGNAL = 1e-9

# ------------------------------------------------------------
# NOISE / INTERFERENCE REJECTION
# ------------------------------------------------------------

# Minimum number of recent samples needed before evaluating
# a persistent response.
EVIDENCE_WINDOW = 5

# A response must exceed the estimated baseline noise
# by this factor.
NOISE_SIGMA_GATE = 4.0

# Minimum relative response before it can contribute.
MIN_RESPONSE_RATIO = 0.025

# Maximum amount of score allowed from a single
# frequency before cross-frequency evidence exists.
SINGLE_FREQUENCY_CAP = 35.0

# A very large isolated jump is treated as interference
# unless it persists.
TRANSIENT_MULTIPLIER = 3.0

# Number of frequency channels that must show meaningful
# repeatable response.
MIN_SUPPORTING_FREQUENCIES = 2


# ============================================================
# STATE
# ============================================================

serial_connection = None
serial_lock = threading.Lock()

last_packet_time = 0

history = []
telemetry_history = []
events = []


latest = {

    "system": "VARUNA06",

    "timestamp": 0,

    "sample": 0,

    "N1": {
        "available": True,
        "simulated": True,
        "magnitude": 0,
        "magnetic_change_percent": 0
    },

    "N2": {
        "tx": 0,
        "frequency": 0
    },

    "N3": {
        "available": False,
        "rx_v": 0,
        "rx_rms_v": 0,
        "rx_peak_to_peak_v": 0,
        "reference_v": 0,
        "bias_expected_v": 2.5
    },

    "N4": {
        "available": False
    },

    "N5": {
        "available": False
    },

    "N6": {
        "available": False
    },

    "analytics": {

        "baseline_v": 0,
        "filtered_v": 0,

        "rx_rms_v": 0,
        "rx_peak_to_peak_v": 0,

        "signal_change_percent": 0,
        "em_change_percent": 0,

        "noise_percent": 0,
        "signal_to_noise": 0,

        "frequency_consistency": 0,
        "frequency_support": 0,

        "persistence": 0,

        "signal_score": 0,
        "fusion_score": 0,
        "final_score": 0,

        "ml_score": 0,

        "anomaly": False,
        "confirmed": False
    }
}


# ============================================================
# BASELINE
# ============================================================

baseline_values = deque(
    maxlen=BASELINE_SAMPLES
)

baseline_v = None
baseline_rms = None
baseline_noise = None


# ============================================================
# RX FILTER
# ============================================================

rx_history = deque(
    maxlen=FILTER_WINDOW
)


# ============================================================
# RECENT RESPONSE
# ============================================================

recent_responses = deque(
    maxlen=EVIDENCE_WINDOW
)


# ============================================================
# FREQUENCY RESPONSE
# ============================================================

frequency_responses = {

    250.0: deque(maxlen=8),

    750.0: deque(maxlen=8),

    1750.0: deque(maxlen=8)
}


# ============================================================
# DETECTION STATE
# ============================================================

anomaly_streak = 0
normal_streak = 0


# ============================================================
# HELPERS
# ============================================================

def safe_float(value, default=0.0):

    try:

        x = float(value)

        if math.isfinite(x):
            return x

        return default

    except Exception:

        return default


def clamp(x, low, high):

    return max(
        low,
        min(high, x)
    )


def median(values):

    if not values:
        return 0.0

    values = sorted(values)

    n = len(values)

    middle = n // 2

    if n % 2:

        return values[middle]

    return (
        values[middle - 1]
        +
        values[middle]
    ) / 2.0


def mad(values, center=None):

    if not values:
        return 0.0

    if center is None:
        center = median(values)

    deviations = [

        abs(x - center)

        for x in values
    ]

    return median(deviations)


def add_event(event_type, message):

    events.insert(
        0,
        {
            "time":
                time.strftime("%H:%M:%S"),

            "type":
                event_type,

            "message":
                message
        }
    )

    del events[EVENT_LIMIT:]


# ============================================================
# RX FILTER
# ============================================================

def filtered_rx(raw):

    rx_history.append(raw)

    return median(
        list(rx_history)
    )


# ============================================================
# BASELINE
# ============================================================

def establish_baseline(rx):

    global baseline_v
    global baseline_rms
    global baseline_noise

    if baseline_v is not None:
        return False

    baseline_values.append(rx)

    if len(baseline_values) < BASELINE_SAMPLES:
        return False

    values = list(
        baseline_values
    )

    baseline_v = median(values)

    deviation = mad(
        values,
        baseline_v
    )

    # Robust noise estimate.
    baseline_noise = max(
        deviation * 1.4826,
        MIN_SIGNAL
    )

    baseline_rms = baseline_v

    add_event(
        "BASELINE",
        (
            "RX baseline established: "
            f"{baseline_v:.7f} V • "
            f"noise {baseline_noise:.7f} V"
        )
    )

    return True


# ============================================================
# BASELINE RESPONSE ANALYSIS
# ============================================================

def calculate_signal_score(rx):

    if baseline_v is None:

        return {
            "score": 0.0,
            "change_percent": 0.0,
            "noise_percent": 0.0,
            "snr": 0.0,
            "valid": False,
            "interference": False
        }


    difference = abs(
        rx - baseline_v
    )


    denominator = max(
        abs(baseline_v),
        MIN_SIGNAL
    )


    deviation_ratio = (
        difference /
        denominator
    )


    change_percent = (
        deviation_ratio *
        100.0
    )


    noise = max(
        baseline_noise or 0.0,
        MIN_SIGNAL
    )


    noise_ratio = (
        noise /
        denominator
    )


    snr = (
        difference /
        noise
    )


    noise_percent = (
        noise_ratio *
        100.0
    )


    # --------------------------------------------------------
    # HARD NOISE GATE
    # --------------------------------------------------------
    #
    # If the change is smaller than BOTH:
    #
    #   relative minimum
    #
    # and
    #
    #   measured baseline noise threshold
    #
    # it contributes ZERO anomaly score.
    #

    noise_threshold_ratio = max(

        MIN_RESPONSE_RATIO,

        noise_ratio *
        NOISE_SIGMA_GATE

    )


    if deviation_ratio <= noise_threshold_ratio:

        return {

            "score": 0.0,

            "change_percent":
                change_percent,

            "noise_percent":
                noise_percent,

            "snr":
                snr,

            "valid": False,

            "interference": False
        }


    # --------------------------------------------------------
    # RESPONSE ABOVE NOISE FLOOR
    # --------------------------------------------------------

    excess_ratio = (

        deviation_ratio
        -
        noise_threshold_ratio
    )


    # Normalize excess response.
    #
    # 10% above the noise gate -> approximately full scale.
    #

    raw_score = (
        excess_ratio /
        0.10
    ) * 100.0


    raw_score = clamp(
        raw_score,
        0,
        100
    )


    # --------------------------------------------------------
    # TRANSIENT PROTECTION
    # --------------------------------------------------------
    #
    # A huge instantaneous jump is NOT automatically a target.
    #
    # If it is dramatically larger than the recent response,
    # classify it as a transient candidate.
    #

    previous = list(
        recent_responses
    )


    interference = False


    if len(previous) >= 3:

        previous_values = [

            x["deviation_ratio"]

            for x in previous
        ]


        previous_median = median(
            previous_values
        )


        if (

            previous_median > 0

            and

            deviation_ratio >
            previous_median *
            TRANSIENT_MULTIPLIER

        ):

            interference = True


    return {

        "score":
            raw_score,

        "change_percent":
            change_percent,

        "noise_percent":
            noise_percent,

        "snr":
            snr,

        "valid":
            True,

        "interference":
            interference
    }


# ============================================================
# FREQUENCY TRACKING
# ============================================================

def update_frequency_response(
    frequency,
    score,
    valid
):

    if frequency not in frequency_responses:

        frequency_responses[
            frequency
        ] = deque(maxlen=8)


    frequency_responses[
        frequency
    ].append({

        "score": score,

        "valid": valid,

        "time": time.time()
    })


# ============================================================
# FREQUENCY SUPPORT
# ============================================================

def frequency_support():

    supported = 0
    total = 0

    frequency_scores = []


    for frequency, values in \
        frequency_responses.items():

        if not values:
            continue

        total += 1

        recent = list(values)[-3:]


        valid_scores = [

            x["score"]

            for x in recent

            if x["valid"]
        ]


        if not valid_scores:
            continue


        mean_score = (

            sum(valid_scores)
            /
            len(valid_scores)

        )


        frequency_scores.append(
            mean_score
        )


        # A frequency only counts as
        # supporting evidence if it has
        # repeated valid response.
        #

        if (

            len(valid_scores) >= 2

            and

            mean_score >= 20

        ):

            supported += 1


    if total == 0:

        return 0, 0.0


    consistency = (

        supported /
        total
    ) * 100.0


    return (
        supported,
        clamp(
            consistency,
            0,
            100
        )
    )


# ============================================================
# CROSS-FREQUENCY EVIDENCE
# ============================================================

def cross_frequency_evidence():

    supported, consistency = \
        frequency_support()


    # --------------------------------------------------------
    # No cross-frequency support:
    #
    # Do NOT allow a single frequency to generate a high
    # anomaly score.
    # --------------------------------------------------------

    if supported < MIN_SUPPORTING_FREQUENCIES:

        return {

            "supported":
                supported,

            "consistency":
                consistency,

            "evidence":
                False
        }


    return {

        "supported":
            supported,

        "consistency":
            consistency,

        "evidence":
            True
    }


# ============================================================
# DETECTION
# ============================================================

def detect(
    rx,
    frequency
):

    global anomaly_streak
    global normal_streak


    signal = calculate_signal_score(
        rx
    )


    # Store current response for
    # transient analysis.
    #

    deviation_ratio = 0.0

    if baseline_v is not None:

        deviation_ratio = (

            abs(
                rx - baseline_v
            )
            /
            max(
                abs(baseline_v),
                MIN_SIGNAL
            )
        )


    recent_responses.append({

        "deviation_ratio":
            deviation_ratio,

        "score":
            signal["score"],

        "valid":
            signal["valid"],

        "interference":
            signal["interference"],

        "frequency":
            frequency
    })


    # --------------------------------------------------------
    # Interference is NOT target evidence.
    # --------------------------------------------------------

    valid_response = (

        signal["valid"]

        and

        not signal["interference"]

    )


    update_frequency_response(

        frequency,

        signal["score"]
        if valid_response
        else 0.0,

        valid_response

    )


    cross = \
        cross_frequency_evidence()


    supported = \
        cross["supported"]


    consistency = \
        cross["consistency"]


    # --------------------------------------------------------
    # IMPORTANT:
    #
    # Before cross-frequency evidence exists,
    # cap the instantaneous score.
    #
    # This prevents:
    #
    #   random interference
    #        ↓
    #   one huge reading
    #        ↓
    #   90% anomaly
    #
    # --------------------------------------------------------

    if not cross["evidence"]:

        gated_signal_score = min(

            signal["score"],

            SINGLE_FREQUENCY_CAP

        )

    else:

        gated_signal_score = \
            signal["score"]


    # --------------------------------------------------------
    # Interference contributes ZERO to target score.
    # --------------------------------------------------------

    if not valid_response:

        gated_signal_score = 0.0


    # --------------------------------------------------------
    # TARGET CANDIDATE
    # --------------------------------------------------------

    candidate = (

        valid_response

        and

        gated_signal_score >= 20

        and

        cross["evidence"]

    )


    if candidate:

        anomaly_streak += 1

        normal_streak = 0

    else:

        normal_streak += 1

        anomaly_streak = max(
            0,
            anomaly_streak - 1
        )


    persistence = clamp(

        (
            anomaly_streak
            /
            PERSISTENCE_REQUIRED
        ) * 100.0,

        0,

        100

    )


    # --------------------------------------------------------
    # FINAL SCORE
    # --------------------------------------------------------
    #
    # No valid cross-frequency evidence:
    #     score remains low.
    #
    # Valid cross-frequency evidence:
    #     signal + consistency + persistence.
    #

    if not cross["evidence"]:

        final_score = min(

            gated_signal_score,

            SINGLE_FREQUENCY_CAP

        )

    else:

        final_score = (

            gated_signal_score * 0.45

            +

            consistency * 0.25

            +

            persistence * 0.30

        )


    final_score = clamp(
        final_score,
        0,
        100
    )


    # --------------------------------------------------------
    # CONFIRMATION
    # --------------------------------------------------------

    confirmed = (

        anomaly_streak
        >=
        PERSISTENCE_REQUIRED

        and

        supported
        >=
        MIN_SUPPORTING_FREQUENCIES

        and

        gated_signal_score
        >=
        45

        and

        consistency
        >=
        50

    )


    # --------------------------------------------------------
    # ANOMALY STATE
    # --------------------------------------------------------

    if confirmed:

        anomaly = True

    elif normal_streak >= RELEASE_REQUIRED:

        anomaly = False

    else:

        anomaly = latest[
            "analytics"
        ].get(
            "anomaly",
            False
        )


    # --------------------------------------------------------
    # N1 SIMULATION
    # --------------------------------------------------------
    #
    # N1 does NOT generate independent random anomalies.
    #
    # It is derived ONLY from accepted EM evidence.
    #
    # No target evidence -> magnetic reference stays normal.
    #

    if cross["evidence"]:

        simulated_magnetic_change = clamp(

            (
                gated_signal_score
                *
                0.65
            )

            +

            (
                consistency
                *
                0.20
            )

            +

            (
                persistence
                *
                0.15
            ),

            0,
            100

        )

    else:

        simulated_magnetic_change = 0.0


    return {

        "signal_score":
            gated_signal_score,

        "signal_change":
            signal["change_percent"],

        "noise_percent":
            signal["noise_percent"],

        "signal_to_noise":
            signal["snr"],

        "frequency_consistency":
            consistency,

        "frequency_support":
            supported,

        "persistence":
            persistence,

        "fusion_score":
            final_score,

        "final_score":
            final_score,

        "anomaly":
            anomaly,

        "confirmed":
            confirmed,

        "interference":
            signal["interference"],

        "n1_simulated_change":
            simulated_magnetic_change
    }


# ============================================================
# PACKET PROCESSING
# ============================================================

def process_packet(packet):

    global latest
    global last_packet_time


    if packet.get("system") != "VARUNA06":

        return


    last_packet_time = time.time()


    event = packet.get("event")


    if event:

        if event == "UNO_READY":

            add_event(
                "SYSTEM",
                "Arduino UNO ready"
            )


        elif event == "ADS1115_READY":

            add_event(
                "SYSTEM",
                "ADS1115 receiver detected"
            )


        elif event == "ADS1115_ERROR":

            add_event(
                "SYSTEM",
                "ADS1115 not detected"
            )


        elif event == "TX_ON":

            add_event(
                "SYSTEM",
                "TX enabled"
            )


        elif event == "TX_OFF":

            add_event(
                "SYSTEM",
                "TX disabled"
            )


        elif event == "BASELINE_RESET_REQUEST":

            add_event(
                "BASELINE",
                "Arduino baseline reset requested"
            )


        return


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


    rx_rms = safe_float(
        n3.get("rx_rms_v")
    )


    rx_pp = safe_float(
        n3.get("rx_peak_to_peak_v")
    )


    frequency = safe_float(
        n2.get("frequency")
    )


    # --------------------------------------------------------
    # FILTER
    # --------------------------------------------------------

    rx_filtered = filtered_rx(
        rx
    )


    # --------------------------------------------------------
    # BASELINE
    # --------------------------------------------------------

    establish_baseline(
        rx_filtered
    )


    # --------------------------------------------------------
    # CALIBRATION PERIOD
    # --------------------------------------------------------

    if baseline_v is None:

        analytics = {

            "baseline_v": 0,

            "filtered_v":
                rx_filtered,

            "rx_rms_v":
                rx_rms,

            "rx_peak_to_peak_v":
                rx_pp,

            "signal_change_percent":
                0,

            "em_change_percent":
                0,

            "noise_percent":
                0,

            "signal_to_noise":
                0,

            "frequency_consistency":
                0,

            "frequency_support":
                0,

            "persistence":
                0,

            "signal_score":
                0,

            "fusion_score":
                0,

            "final_score":
                0,

            "ml_score":
                0,

            "anomaly":
                False,

            "confirmed":
                False,

            "interference":
                False
        }


        # N1 remains simulated but neutral.
        packet["N1"] = {

            "available": True,

            "simulated": True,

            "sensor":
                "simulated_from_EM",

            "magnitude":
                0,

            "magnetic_change_percent":
                0
        }


    else:

        result = detect(

            rx_filtered,

            frequency

        )


        analytics = {

            "baseline_v":
                baseline_v,

            "filtered_v":
                rx_filtered,

            "rx_rms_v":
                rx_rms,

            "rx_peak_to_peak_v":
                rx_pp,

            "signal_change_percent":
                result[
                    "signal_change"
                ],

            "em_change_percent":
                result[
                    "signal_change"
                ],

            "noise_percent":
                result[
                    "noise_percent"
                ],

            "signal_to_noise":
                result[
                    "signal_to_noise"
                ],

            "frequency_consistency":
                result[
                    "frequency_consistency"
                ],

            "frequency_support":
                result[
                    "frequency_support"
                ],

            "persistence":
                result[
                    "persistence"
                ],

            "signal_score":
                result[
                    "signal_score"
                ],

            "fusion_score":
                result[
                    "fusion_score"
                ],

            "final_score":
                result[
                    "final_score"
                ],

            "ml_score":
                0,

            "anomaly":
                result[
                    "anomaly"
                ],

            "confirmed":
                result[
                    "confirmed"
                ],

            "interference":
                result[
                    "interference"
                ]
        }


        # ----------------------------------------------------
        # SIMULATED N1
        # ----------------------------------------------------
        #
        # IMPORTANT:
        # N1 never independently creates an anomaly.
        #
        # It mirrors accepted EM evidence only.
        #

        n1_change = result[
            "n1_simulated_change"
        ]


        simulated_magnitude = (

            100.0

            +

            (
                n1_change
                *
                0.25
            )

        )


        packet["N1"] = {

            "available": True,

            "simulated": True,

            "sensor":
                "simulated_from_EM",

            "magnitude":
                simulated_magnitude,

            "magnetic_change_percent":
                n1_change
        }


        # ----------------------------------------------------
        # EVENTS
        # ----------------------------------------------------

        if result["confirmed"]:

            add_event(

                "ANOMALY",

                (
                    f"{frequency:.0f} Hz • "
                    f"RX {rx_filtered:.7f} V • "
                    f"Δ "
                    f"{result['signal_change']:.2f}% • "
                    f"Support "
                    f"{result['frequency_support']} freq • "
                    f"Score "
                    f"{result['fusion_score']:.0f}%"
                )

            )


        elif result["interference"]:

            add_event(

                "SYSTEM",

                (
                    f"Transient interference rejected • "
                    f"{frequency:.0f} Hz • "
                    f"RX {rx_filtered:.7f} V"
                )

            )


        elif result["anomaly"]:

            add_event(

                "RESCAN",

                (
                    f"Persistent EM response • "
                    f"Support "
                    f"{result['frequency_support']} freq • "
                    f"Score "
                    f"{result['fusion_score']:.0f}%"
                )

            )


    # --------------------------------------------------------
    # UPDATE PACKET
    # --------------------------------------------------------

    packet["analytics"] = analytics


    with serial_lock:

        latest = packet


        history.append({

            "frequency":
                frequency,

            "rx":
                rx_filtered,

            "rx_rms":
                rx_rms,

            "rx_peak_to_peak":
                rx_pp,

            "fusion":
                analytics[
                    "fusion_score"
                ],

            "score":
                analytics[
                    "final_score"
                ]
        })


        if len(history) > WINDOW_SIZE:

            del history[
                :-WINDOW_SIZE
            ]


        telemetry_history.append({

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

            "rx_rms_v":
                rx_rms,

            "rx_peak_to_peak_v":
                rx_pp,

            "baseline_v":
                baseline_v or 0,

            "signal_change_percent":
                analytics[
                    "signal_change_percent"
                ],

            "noise_percent":
                analytics[
                    "noise_percent"
                ],

            "signal_to_noise":
                analytics[
                    "signal_to_noise"
                ],

            "frequency_consistency":
                analytics[
                    "frequency_consistency"
                ],

            "frequency_support":
                analytics[
                    "frequency_support"
                ],

            "fusion_score":
                analytics[
                    "fusion_score"
                ],

            "anomaly":
                analytics[
                    "anomaly"
                ],

            "confirmed":
                analytics[
                    "confirmed"
                ]
        })


        if len(
            telemetry_history
        ) > 5000:

            del telemetry_history[
                :-5000
            ]


# ============================================================
# SERIAL THREAD
# ============================================================

def serial_reader():

    global serial_connection


    while True:

        try:

            if serial_connection is None:

                print(
                    "Opening",
                    SERIAL_PORT
                )


                serial_connection = \
                    serial.Serial(
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
                    e
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
# DATA
# ============================================================

@app.route("/data")
def data():

    connected = (

        time.time()
        -
        last_packet_time

    ) < 3


    with serial_lock:

        return jsonify({

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
                False,

            "ml_trained":
                False

        })


# ============================================================
# RESET BASELINE
# ============================================================

@app.route(
    "/baseline/reset",
    methods=["POST"]
)
def reset_baseline():

    global baseline_v
    global baseline_rms
    global baseline_noise

    global anomaly_streak
    global normal_streak


    baseline_v = None
    baseline_rms = None
    baseline_noise = None


    baseline_values.clear()
    rx_history.clear()
    recent_responses.clear()


    for values in \
        frequency_responses.values():

        values.clear()


    anomaly_streak = 0
    normal_streak = 0


    add_event(

        "BASELINE",

        (
            "Baseline reset. "
            "Keep target away during calibration."
        )

    )


    return jsonify({

        "success":
            True,

        "message":
            (
                "Baseline reset. "
                "Keep the sensor in a clean "
                "no-target environment while "
                "calibration runs."
            )

    })


# ============================================================
# EXPORT
# ============================================================

@app.route("/export")
def export():

    output = io.StringIO()

    writer = csv.writer(
        output
    )


    writer.writerow([

        "time",
        "sample",
        "frequency_hz",
        "rx_v",
        "rx_rms_v",
        "rx_peak_to_peak_v",
        "baseline_v",
        "signal_change_percent",
        "noise_percent",
        "signal_to_noise",
        "frequency_consistency",
        "frequency_support",
        "fusion_score",
        "anomaly",
        "confirmed"

    ])


    with serial_lock:

        for row in telemetry_history:

            writer.writerow([

                row["time"],

                row["sample"],

                row["frequency"],

                row["rx_v"],

                row["rx_rms_v"],

                row["rx_peak_to_peak_v"],

                row["baseline_v"],

                row[
                    "signal_change_percent"
                ],

                row[
                    "noise_percent"
                ],

                row[
                    "signal_to_noise"
                ],

                row[
                    "frequency_consistency"
                ],

                row[
                    "frequency_support"
                ],

                row[
                    "fusion_score"
                ],

                row[
                    "anomaly"
                ],

                row[
                    "confirmed"
                ]

            ])


    return Response(

        output.getvalue(),

        mimetype="text/csv",

        headers={

            "Content-Disposition":
                (
                    "attachment; "
                    "filename=varuna06_data.csv"
                )

        }

    )


# ============================================================
# SETTINGS & PORTS API
# ============================================================

@app.route("/api/ports")
def get_ports():
    try:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["COM1", "COM2", "COM3", "COM4", "COM5"]
    except Exception:
        ports = ["COM1", "COM2", "COM3", "COM4", "COM5"]
    return jsonify({"ports": ports, "current": SERIAL_PORT, "baud": BAUD_RATE})


@app.route("/api/settings", methods=["GET"])
def get_settings():
    return jsonify({
        "serial_port": SERIAL_PORT,
        "baud_rate": BAUD_RATE,
        "noise_sigma": NOISE_SIGMA_GATE,
        "persistence": PERSISTENCE_REQUIRED
    })


@app.route("/settings/update", methods=["POST"])
def update_settings():
    global SERIAL_PORT, BAUD_RATE, NOISE_SIGMA_GATE, PERSISTENCE_REQUIRED, serial_connection
    data = request.json or {}
    
    if "serial_port" in data:
        SERIAL_PORT = str(data["serial_port"])
    if "baud_rate" in data:
        try:
            BAUD_RATE = int(data["baud_rate"])
        except ValueError:
            pass
    if "noise_sigma" in data:
        try:
            NOISE_SIGMA_GATE = float(data["noise_sigma"])
        except ValueError:
            pass
    if "persistence" in data:
        try:
            PERSISTENCE_REQUIRED = int(data["persistence"])
        except ValueError:
            pass

    # Force reconnect with new port
    with serial_lock:
        if serial_connection:
            try:
                serial_connection.close()
            except Exception:
                pass
            serial_connection = None

    add_event("SYSTEM", f"Settings updated: Port={SERIAL_PORT}, Baud={BAUD_RATE}")
    return jsonify({"success": True, "message": "Settings updated and serial reconnection scheduled."})


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print(
        "=" * 60
    )

    print(
        "VARUNA06 — V0 EM RECEIVER"
    )

    print(
        "=" * 60
    )

    print(
        f"Serial port: {SERIAL_PORT}"
    )

    print(
        f"Baud rate: {BAUD_RATE}"
    )

    print(
        "TX: Arduino D9"
    )

    print(
        "RX: ADS1115 A0"
    )

    print(
        "N1: SIMULATED FROM ACCEPTED EM EVIDENCE"
    )

    print(
        "ML: disabled"
    )

    print(
        "Detection: noise-gated EM + "
        "cross-frequency evidence + persistence"
    )

    print(
        "Dashboard: http://127.0.0.1:5000"
    )

    print(
        "=" * 60
    )


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