from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
import atexit
import time
import socket
from threading import Lock, Thread, Event
import subprocess
import random
import os

app = Flask(__name__)

# ============================================================
# ===== DRIVE MOTORS (BTS7960 x2) =====
# ============================================================
LPWM_L = 18
RPWM_L = 23
LPWM_R = 19
RPWM_R = 20

# ============================================================
# ===== DOME MOTOR (BTS7960) =====
# ============================================================
LPWM_DOME = 24
RPWM_DOME = 25

# ============================================================
# ===== 2-3-2 LEG ACTUATORS (BTS7960 x2 - one per actuator) =====
# Two mirrored 12V linear actuators, each on its OWN BTS7960
# driver - electrically independent, but commanded together in
# software. Hold-to-move: press R to extend, F to retract, and
# they move for as long as you hold the key/button. Each
# actuator's built-in limit switch protects it automatically if
# you hold past the true end of travel.
# ============================================================
LPWM_ACT_A = 5    # Actuator A - Retract
RPWM_ACT_A = 6    # Actuator A - Extend
LPWM_ACT_B = 16   # Actuator B - Retract
RPWM_ACT_B = 21   # Actuator B - Extend

ACTUATOR_SPEED = 100  # actuators run at full speed for reliable limit-switch contact

# ============================================================
# ===== CENTER FOOT LEAD SCREW MOTOR (BTS7960, 24V) =====
# Hold-to-move control, no limit switches assumed.
# ============================================================
LPWM_SCREW = 13   # Foot up
RPWM_SCREW = 12   # Foot down

DEFAULT_SCREW_SPEED = 60

# ============================================================
# ===== GENERAL CONFIG =====
# ============================================================
PWM_FREQ = 1000
DEFAULT_SPEED = 25       # Drive motors default (slider default)
DEFAULT_DOME_SPEED = 30  # Dome motor default
MIN_SPEED = 0
MAX_SPEED = 100

# Drive motor ramping (smooth acceleration/deceleration)
RAMP_START_SPEED = 35    # motors always kick off at this duty cycle for reliable start
RAMP_STEP = 5             # duty cycle change per ramp tick
RAMP_DELAY = 0.03         # seconds between ramp ticks (~30ms = fast, smooth ramp)

# Audio configuration
AUDIO_DEVICE = "hw:3,0"  # USB speaker (card 3)
AUDIO_CARD = "3"
STATIC_PATH = os.path.join(os.path.dirname(__file__), 'static')

# Thread safety
motor_lock = Lock()
audio_lock = Lock()

# Track current state
current_state = {
    "direction": "stop",
    "speed": 0,
    "dome_direction": "stop",
    "dome_speed": 0,
    "actuator_state": "stopped",   # stopped / extending / retracting
    "screw_state": "stop",         # stop / up / down
    "volume": 80
}

# Audio state
audio_state = {
    "is_moving": False,
    "current_process": None,
    "idle_thread": None,
    "movement_thread": None,
    "stop_threads": False
}

# Drive ramp state
drive_ramp_stop_event = Event()
drive_ramp_thread = None
current_drive_speed = 0
current_drive_setter = None

# Audio file categories
audio_files = {
    'movement': [
        'scanning.mp3', 'excited-1.mp3', 'excited-2.mp3',
        'uh-huh.mp3', 'acknowledged-1.mp3', 'acknowledged-2.mp3'
    ],
    'idle': [
        'chat-1.mp3', 'chat-2.mp3', 'chat-long.mp3',
        'curious.mp3', 'sing-sound-effect.mp3'
    ],
    'emergency': [
        'alarm.mp3', 'worried.mp3', 'overload.mp3',
        'overload-screaming.mp3', '1-screaming.mp3', 'no.mp3'
    ],
    'special': [
        'excited-1.mp3', 'excited-2.mp3', 'shout.mp3', 'over-there.mp3'
    ]
}

# ============================================================
# ===== OLED DISPLAY (shows WiFi network + IP address) =====
# Requires: pip install adafruit-blinka adafruit-circuitpython-ssd1306 pillow
# Wiring: I2C bus (SDA -> GPIO2 / pin3, SCL -> GPIO3 / pin5)
# Shares the I2C bus with the Adafruit PWM/Servo HAT (different address)
# ============================================================
oled = None
oled_available = False
try:
    import board
    import busio
    import adafruit_ssd1306
    from PIL import Image, ImageDraw, ImageFont

    i2c = busio.I2C(board.SCL, board.SDA)
    oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)  # change to 128,64 if that's your screen
    oled_font = ImageFont.load_default()
    oled_available = True
    print("✅ OLED display initialized")
except Exception as e:
    print(f"⚠️  OLED not available: {e}")
    print("   Install with: pip install adafruit-blinka adafruit-circuitpython-ssd1306 pillow")

oled_stop_flag = False

def get_ip_address():
    """Get the Pi's current LAN IP address"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "No network"
    finally:
        s.close()
    return ip

def get_wifi_ssid():
    """Get the currently connected WiFi network name"""
    try:
        result = subprocess.run(['iwgetid', '-r'], capture_output=True, text=True, timeout=2)
        ssid = result.stdout.strip()
        return ssid if ssid else "Not connected"
    except Exception:
        return "Unknown"

def _oled_draw(lines):
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    y = 0
    for line in lines:
        draw.text((0, y), line, font=oled_font, fill=255)
        y += 11
    oled.image(image)
    oled.show()

def oled_loop():
    """Continuously display the Pi's WiFi network and IP address on the OLED"""
    if not oled_available:
        return

    while not oled_stop_flag:
        ip = get_ip_address()
        ssid = get_wifi_ssid()
        if ip == "No network":
            _oled_draw(["R2-D2 BOOTING...", "Waiting for network"])
        else:
            _oled_draw(["R2-D2 ONLINE", f"WiFi: {ssid}", f"IP: {ip}"])
        time.sleep(5)

# ============================================================
# ===== GPIO SETUP =====
# ============================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([
    RPWM_L, LPWM_L, RPWM_R, LPWM_R,
    RPWM_DOME, LPWM_DOME,
    RPWM_ACT_A, LPWM_ACT_A,
    RPWM_ACT_B, LPWM_ACT_B,
    RPWM_SCREW, LPWM_SCREW
], GPIO.OUT)

# Drive motor PWM
pwm_l_fwd = GPIO.PWM(RPWM_L, PWM_FREQ)
pwm_l_rev = GPIO.PWM(LPWM_L, PWM_FREQ)
pwm_r_fwd = GPIO.PWM(RPWM_R, PWM_FREQ)
pwm_r_rev = GPIO.PWM(LPWM_R, PWM_FREQ)
pwm_l_fwd.start(0); pwm_l_rev.start(0)
pwm_r_fwd.start(0); pwm_r_rev.start(0)

# Dome motor PWM
pwm_dome_cw = GPIO.PWM(RPWM_DOME, PWM_FREQ)
pwm_dome_ccw = GPIO.PWM(LPWM_DOME, PWM_FREQ)
pwm_dome_cw.start(0); pwm_dome_ccw.start(0)

# Actuator PWM (2-3-2 legs - two independent drivers)
pwm_act_a_extend = GPIO.PWM(RPWM_ACT_A, PWM_FREQ)
pwm_act_a_retract = GPIO.PWM(LPWM_ACT_A, PWM_FREQ)
pwm_act_a_extend.start(0); pwm_act_a_retract.start(0)

pwm_act_b_extend = GPIO.PWM(RPWM_ACT_B, PWM_FREQ)
pwm_act_b_retract = GPIO.PWM(LPWM_ACT_B, PWM_FREQ)
pwm_act_b_extend.start(0); pwm_act_b_retract.start(0)

# Lead screw PWM (center foot)
pwm_screw_up = GPIO.PWM(LPWM_SCREW, PWM_FREQ)
pwm_screw_down = GPIO.PWM(RPWM_SCREW, PWM_FREQ)
pwm_screw_up.start(0); pwm_screw_down.start(0)

print("✅ BTS7960 drive motors initialized (Left & Right)")
print("✅ BTS7960 dome motor initialized")
print("✅ BTS7960 2-3-2 leg actuators initialized (Actuator A & B, independent)")
print("✅ BTS7960 lead screw motor initialized")
print("🔊 Audio system initialized (USB speaker)")

# ============================================================
# ===== AUDIO FUNCTIONS =====
# ============================================================
def set_volume(volume):
    """Set system volume (0-100) on the USB speaker"""
    try:
        volume = max(0, min(100, volume))
        subprocess.run(['amixer', '-c', AUDIO_CARD, 'set', 'PCM', f'{volume}%'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        current_state["volume"] = volume
        print(f"🔊 Volume set to {volume}%")
    except Exception as e:
        print(f"❌ Volume control error: {e}")

def get_current_volume():
    """Read current volume from the USB speaker"""
    try:
        result = subprocess.run(['amixer', '-c', AUDIO_CARD, 'get', 'PCM'],
                                 capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'Playback' in line and '%' in line:
                start = line.find('[') + 1
                end = line.find('%]')
                if start > 0 and end > 0:
                    vol = int(line[start:end])
                    current_state["volume"] = vol
                    return vol
    except Exception:
        pass
    return 80

def play_audio_file(filename):
    """Play an audio file on the Pi through the USB speaker"""
    filepath = os.path.join(STATIC_PATH, filename)
    if not os.path.exists(filepath):
        print(f"⚠️  Audio file not found: {filepath}")
        return

    with audio_lock:
        if audio_state["current_process"]:
            try:
                audio_state["current_process"].terminate()
                audio_state["current_process"].wait(timeout=0.5)
            except Exception:
                pass
        try:
            audio_state["current_process"] = subprocess.Popen(
                ['mpg123', '-q', '-a', AUDIO_DEVICE, filepath],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            print(f"🔊 Playing: {filename}")
        except Exception as e:
            print(f"❌ Audio error: {e}")

def get_random_sound(category):
    sounds = audio_files.get(category, [])
    return random.choice(sounds) if sounds else None

def movement_sound_loop():
    while not audio_state["stop_threads"]:
        if audio_state["is_moving"]:
            sound = get_random_sound('movement')
            if sound:
                play_audio_file(sound)
            time.sleep(random.uniform(3, 5))
        else:
            time.sleep(0.5)

def idle_sound_loop():
    while not audio_state["stop_threads"]:
        if not audio_state["is_moving"]:
            sound = get_random_sound('idle')
            if sound:
                play_audio_file(sound)
            time.sleep(random.uniform(4, 10))
        else:
            time.sleep(0.5)

def start_audio_system():
    if not audio_state["movement_thread"] or not audio_state["movement_thread"].is_alive():
        audio_state["stop_threads"] = False
        audio_state["movement_thread"] = Thread(target=movement_sound_loop, daemon=True)
        audio_state["movement_thread"].start()
    if not audio_state["idle_thread"] or not audio_state["idle_thread"].is_alive():
        audio_state["idle_thread"] = Thread(target=idle_sound_loop, daemon=True)
        audio_state["idle_thread"].start()

def stop_audio_system():
    audio_state["stop_threads"] = True
    with audio_lock:
        if audio_state["current_process"]:
            try:
                audio_state["current_process"].terminate()
                audio_state["current_process"] = None
            except Exception:
                pass

# ============================================================
# ===== HELPERS =====
# ============================================================
def constrain_speed(speed):
    return max(0, min(MAX_SPEED, speed))

def set_moving(flag):
    audio_state["is_moving"] = flag

# ============================================================
# ===== DRIVE MOTOR RAMPING =====
# Motors always kick off at RAMP_START_SPEED (35%) for reliable
# torque, then smoothly ramp up to the requested speed. Stopping
# smoothly ramps back down to 0 instead of cutting instantly.
# ============================================================
def set_drive_motors(left_fwd, left_rev, right_fwd, right_rev):
    with motor_lock:
        pwm_l_fwd.ChangeDutyCycle(constrain_speed(left_fwd))
        pwm_l_rev.ChangeDutyCycle(constrain_speed(left_rev))
        pwm_r_fwd.ChangeDutyCycle(constrain_speed(right_fwd))
        pwm_r_rev.ChangeDutyCycle(constrain_speed(right_rev))

def _drive_ramp_worker(setter, target):
    global current_drive_speed
    speed = current_drive_speed
    if target > 0 and speed < RAMP_START_SPEED:
        speed = RAMP_START_SPEED

    setter(speed)
    current_drive_speed = speed

    while not drive_ramp_stop_event.is_set() and speed != target:
        if speed < target:
            speed = min(target, speed + RAMP_STEP)
        else:
            speed = max(target, speed - RAMP_STEP)
        setter(speed)
        current_drive_speed = speed
        time.sleep(RAMP_DELAY)

def _start_drive_ramp(setter, target):
    global drive_ramp_thread, current_drive_setter
    drive_ramp_stop_event.set()
    if drive_ramp_thread and drive_ramp_thread.is_alive():
        drive_ramp_thread.join(timeout=0.2)
    drive_ramp_stop_event.clear()
    current_drive_setter = setter
    drive_ramp_thread = Thread(target=_drive_ramp_worker, args=(setter, target), daemon=True)
    drive_ramp_thread.start()

# ============================================================
# ===== DRIVE MOTOR FUNCTIONS =====
# ============================================================
def stop_drive():
    global current_drive_setter
    if current_drive_setter:
        _start_drive_ramp(current_drive_setter, 0)
    else:
        set_drive_motors(0, 0, 0, 0)
    current_state.update({"direction": "stop", "speed": 0})
    set_moving(False)

def brake_drive():
    global current_drive_setter, current_drive_speed
    drive_ramp_stop_event.set()
    with motor_lock:
        pwm_l_fwd.ChangeDutyCycle(100)
        pwm_l_rev.ChangeDutyCycle(100)
        pwm_r_fwd.ChangeDutyCycle(100)
        pwm_r_rev.ChangeDutyCycle(100)
        time.sleep(0.05)
    set_drive_motors(0, 0, 0, 0)
    current_drive_setter = None
    current_drive_speed = 0
    current_state.update({"direction": "stop", "speed": 0})
    set_moving(False)

def forward(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(s, 0, s, 0)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "forward", "speed": speed})
    set_moving(True)

def backward(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(0, s, 0, s)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "backward", "speed": speed})
    set_moving(True)

def left(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(0, s, s, 0)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "left", "speed": speed})
    set_moving(True)

def right(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(s, 0, 0, s)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "right", "speed": speed})
    set_moving(True)

def pivot_left(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(s * 0.3, 0, s, 0)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "pivot_left", "speed": speed})
    set_moving(True)

def pivot_right(speed):
    speed = constrain_speed(speed)
    setter = lambda s: set_drive_motors(s, 0, s * 0.3, 0)
    _start_drive_ramp(setter, speed)
    current_state.update({"direction": "pivot_right", "speed": speed})
    set_moving(True)

# ============================================================
# ===== DOME MOTOR FUNCTIONS =====
# ============================================================
def stop_dome():
    with motor_lock:
        pwm_dome_cw.ChangeDutyCycle(0)
        pwm_dome_ccw.ChangeDutyCycle(0)
        current_state.update({"dome_direction": "stop", "dome_speed": 0})

def rotate_dome_left(speed):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_dome_ccw.ChangeDutyCycle(0)
        pwm_dome_cw.ChangeDutyCycle(speed)
        current_state.update({"dome_direction": "left", "dome_speed": speed})
    set_moving(True)

def rotate_dome_right(speed):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_dome_cw.ChangeDutyCycle(0)
        pwm_dome_ccw.ChangeDutyCycle(speed)
        current_state.update({"dome_direction": "right", "dome_speed": speed})
    set_moving(True)

# ============================================================
# ===== 2-3-2 LEG ACTUATOR FUNCTIONS =====
# Hold-to-move: extend/retract run for as long as the button or
# key is held, giving direct manual control over how far the
# legs pivot. Actuator A and B are on separate BTS7960 drivers
# but are always commanded at the exact same instant, so they
# move in sync. Each actuator's built-in limit switch will stop
# it safely if you hold past the true end of travel.
# ============================================================
def stop_actuators():
    with motor_lock:
        pwm_act_a_extend.ChangeDutyCycle(0)
        pwm_act_a_retract.ChangeDutyCycle(0)
        pwm_act_b_extend.ChangeDutyCycle(0)
        pwm_act_b_retract.ChangeDutyCycle(0)
        current_state["actuator_state"] = "stopped"

def extend_actuators(speed=ACTUATOR_SPEED):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_act_a_retract.ChangeDutyCycle(0)
        pwm_act_b_retract.ChangeDutyCycle(0)
        pwm_act_a_extend.ChangeDutyCycle(speed)
        pwm_act_b_extend.ChangeDutyCycle(speed)
        current_state["actuator_state"] = "extending"
    set_moving(True)

def retract_actuators(speed=ACTUATOR_SPEED):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_act_a_extend.ChangeDutyCycle(0)
        pwm_act_b_extend.ChangeDutyCycle(0)
        pwm_act_a_retract.ChangeDutyCycle(speed)
        pwm_act_b_retract.ChangeDutyCycle(speed)
        current_state["actuator_state"] = "retracting"
    set_moving(True)

# ============================================================
# ===== LEAD SCREW (CENTER FOOT) FUNCTIONS =====
# Simple hold-to-move control (press = move, release = stop)
# ============================================================
def stop_screw():
    with motor_lock:
        pwm_screw_up.ChangeDutyCycle(0)
        pwm_screw_down.ChangeDutyCycle(0)
        current_state["screw_state"] = "stop"

def screw_up(speed=DEFAULT_SCREW_SPEED):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_screw_down.ChangeDutyCycle(0)
        pwm_screw_up.ChangeDutyCycle(speed)
        current_state["screw_state"] = "up"

def screw_down(speed=DEFAULT_SCREW_SPEED):
    speed = constrain_speed(speed)
    with motor_lock:
        pwm_screw_up.ChangeDutyCycle(0)
        pwm_screw_down.ChangeDutyCycle(speed)
        current_state["screw_state"] = "down"

# ============================================================
# ===== EMERGENCY STOP =====
# ============================================================
def emergency_stop():
    brake_drive()
    stop_dome()
    stop_actuators()
    stop_screw()
    sound = get_random_sound('emergency')
    if sound:
        play_audio_file(sound)
    print("⚠️  EMERGENCY STOP ACTIVATED")

def cleanup():
    global oled_stop_flag
    oled_stop_flag = True
    stop_audio_system()
    emergency_stop()
    GPIO.cleanup()

atexit.register(cleanup)

# ============================================================
# ===== ROUTES =====
# ============================================================
@app.route("/")
def home():
    return render_template("index.html")

@app.route("/move", methods=["POST"])
def move():
    try:
        data = request.get_json()
        direction = data.get("direction")
        speed = int(data.get("speed", DEFAULT_SPEED))

        if direction == "forward":
            forward(speed)
        elif direction == "backward":
            backward(speed)
        elif direction == "left":
            left(speed)
        elif direction == "right":
            right(speed)
        elif direction == "pivot_left":
            pivot_left(speed)
        elif direction == "pivot_right":
            pivot_right(speed)
        elif direction == "stop":
            stop_drive()
        elif direction == "brake":
            brake_drive()
        else:
            return jsonify(status="error", message="Invalid direction"), 400

        return jsonify(status="ok", state=current_state)
    except Exception as e:
        print(f"Error in move endpoint: {e}")
        stop_drive()
        return jsonify(status="error", message=str(e)), 500

@app.route("/dome", methods=["POST"])
def dome():
    try:
        data = request.get_json()
        direction = data.get("direction")
        speed = int(data.get("speed", DEFAULT_DOME_SPEED))

        if direction == "left":
            rotate_dome_left(speed)
        elif direction == "right":
            rotate_dome_right(speed)
        elif direction == "stop":
            stop_dome()
        else:
            return jsonify(status="error", message="Invalid dome direction"), 400

        return jsonify(status="ok", state=current_state)
    except Exception as e:
        print(f"Error in dome endpoint: {e}")
        stop_dome()
        return jsonify(status="error", message=str(e)), 500

@app.route("/actuators", methods=["POST"])
def actuators():
    """2-3-2 leg transition control - hold to move"""
    try:
        data = request.get_json()
        direction = data.get("direction")
        speed = int(data.get("speed", ACTUATOR_SPEED))

        if direction == "extend":
            extend_actuators(speed)
        elif direction == "retract":
            retract_actuators(speed)
        elif direction == "stop":
            stop_actuators()
        else:
            return jsonify(status="error", message="Invalid actuator direction"), 400

        return jsonify(status="ok", state=current_state)
    except Exception as e:
        print(f"Error in actuators endpoint: {e}")
        stop_actuators()
        return jsonify(status="error", message=str(e)), 500

@app.route("/screw", methods=["POST"])
def screw():
    """Center foot lead screw control (hold-to-move)"""
    try:
        data = request.get_json()
        direction = data.get("direction")
        speed = int(data.get("speed", DEFAULT_SCREW_SPEED))

        if direction == "up":
            screw_up(speed)
        elif direction == "down":
            screw_down(speed)
        elif direction == "stop":
            stop_screw()
        else:
            return jsonify(status="error", message="Invalid screw direction"), 400

        return jsonify(status="ok", state=current_state)
    except Exception as e:
        print(f"Error in screw endpoint: {e}")
        stop_screw()
        return jsonify(status="error", message=str(e)), 500

@app.route("/status", methods=["GET"])
def status():
    return jsonify(state=current_state)

@app.route("/volume", methods=["POST"])
def volume():
    try:
        data = request.get_json()
        vol = int(data.get("volume", 80))
        set_volume(vol)
        return jsonify(status="ok", volume=current_state["volume"])
    except Exception as e:
        print(f"Error in volume endpoint: {e}")
        return jsonify(status="error", message=str(e)), 500

@app.route("/emergency_stop", methods=["POST"])
def emergency_stop_endpoint():
    emergency_stop()
    return jsonify(status="ok", message="Emergency stop activated")

# ============================================================
# ===== MAIN =====
# ============================================================
if __name__ == "__main__":
    try:
        start_audio_system()

        if oled_available:
            oled_thread = Thread(target=oled_loop, daemon=True)
            oled_thread.start()

        initial_volume = get_current_volume()
        print(f"Current volume: {initial_volume}%")
        print(f"Pi IP address: {get_ip_address()}")
        print("R2-D2 control system ready")
        print("R2-D2 Motor Control Server starting on port 5000...")
        print("Access at: http://<your-pi-ip>:5000")

        app.run(host="0.0.0.0", port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n⚠️  Shutting down...")
    finally:
        cleanup()
        print("✅ Cleanup complete")