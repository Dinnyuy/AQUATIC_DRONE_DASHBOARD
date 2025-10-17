from flask import Flask, render_template, Response, jsonify
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from datetime import datetime, timedelta
import random
import time
import serial
import threading
import logging
import os
import serial.tools.list_ports
import json
import cv2
import numpy as np

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///sensors.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)
migrate = Migrate(app, db)

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('ArduinoDashboard')

# Ballast Water Thresholds (Kribi waters specific)
TURBIDITY_THRESHOLD = 6.0  # NTU (ballast water indicator)
TEMPERATURE_THRESHOLD = 31.0  # °C (thermal pollution)
CONDUCTIVITY_THRESHOLD = 8.0  # mS/cm (salinity change)
PH_THRESHOLD_LOW = 6.5  # Minimum pH
PH_THRESHOLD_HIGH = 8.5  # Maximum pH
DO_THRESHOLD = 5.0  # mg/L (minimum dissolved oxygen)
BALLAST_ALERT_THRESHOLD = 3  # Minimum sensors that must exceed thresholds

# Serial Setup
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = None
arduino_connected = False
reconnect_attempts = 0
MAX_RECONNECT_ATTEMPTS = 50
RECONNECT_DELAY = 5  # seconds

# Globals - Initialize with None
latest_turbidity = None
latest_temperature = None
latest_conductivity = None
latest_ph = None
latest_do = None
latest_latitude = None
latest_longitude = None

last_update = None
first_connection_time = None
data_lock = threading.Lock()

# GPS Configuration
BASE_LATITUDE = 4.2105
BASE_LONGITUDE = 6.4375
last_real_gps_time = None
gps_fix_acquired = False
gps_mode = "simulated"

# Peak tracking variables
current_peak_temp = -float('inf')
current_peak_turbidity = -float('inf')
current_peak_ec = -float('inf')
current_peak_ph = -float('inf')
current_peak_do = -float('inf')
last_peak_log_time = datetime.now()

# Camera Setup - Improved for both Raspberry Pi Camera and USB Webcam
camera = None
camera_lock = threading.Lock()
camera_initialized = False
camera_type = "none"

# Database models
class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    turbidity = db.Column(db.Float)
    temperature = db.Column(db.Float)
    conductivity = db.Column(db.Float)
    ph = db.Column(db.Float)  # pH sensor
    do = db.Column(db.Float)  # Dissolved Oxygen sensor
    latitude = db.Column(db.Float)
    longitude = db.Column(db.Float)
    gps_type = db.Column(db.String(20))  # 'simulated' or 'real'
    above_threshold = db.Column(db.Boolean, default=False)

class PeakLog(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    peak_temperature = db.Column(db.Float)
    peak_turbidity = db.Column(db.Float)
    peak_ec = db.Column(db.Float)
    peak_ph = db.Column(db.Float)  # pH peak
    peak_do = db.Column(db.Float)  # DO peak
    latitude = db.Column(db.Float)
    longitude = db.Column(db.Float)
    gps_type = db.Column(db.String(20))

# Initialize from database on startup
def initialize_from_database():
    global latest_turbidity, latest_temperature, latest_conductivity, last_update
    global latest_ph, latest_do
    global current_peak_temp, current_peak_turbidity, current_peak_ec, current_peak_ph, current_peak_do, last_peak_log_time
    global gps_fix_acquired, last_real_gps_time, gps_mode
    
    try:
        # Get last sensor data
        last_entry = SensorData.query.order_by(SensorData.timestamp.desc()).first()
        if last_entry:
            latest_turbidity = last_entry.turbidity
            latest_temperature = last_entry.temperature
            latest_conductivity = last_entry.conductivity
            latest_ph = last_entry.ph
            latest_do = last_entry.do
            last_update = last_entry.timestamp
            
            if last_entry.gps_type == 'real':
                gps_fix_acquired = True
                last_real_gps_time = last_entry.timestamp
                gps_mode = "real"
            
            logger.info(f"Initialized from DB: Turbidity={latest_turbidity}, Temp={latest_temperature}, EC={latest_conductivity}, pH={latest_ph}, DO={latest_do}")
        
        # Get last peak log
        last_peak = PeakLog.query.order_by(PeakLog.timestamp.desc()).first()
        if last_peak:
            last_peak_log_time = last_peak.timestamp
        
        # Initialize current peaks
        if last_peak_log_time:
            max_values = db.session.query(
                db.func.max(SensorData.temperature),
                db.func.max(SensorData.turbidity),
                db.func.max(SensorData.conductivity),
                db.func.max(SensorData.ph),
                db.func.max(SensorData.do)
            ).filter(SensorData.timestamp >= last_peak_log_time).first()
            
            if max_values[0] is not None: current_peak_temp = max_values[0]
            if max_values[1] is not None: current_peak_turbidity = max_values[1]
            if max_values[2] is not None: current_peak_ec = max_values[2]
            if max_values[3] is not None: current_peak_ph = max_values[3]
            if max_values[4] is not None: current_peak_do = max_values[4]
        
        if latest_temperature: current_peak_temp = max(current_peak_temp, latest_temperature)
        if latest_turbidity: current_peak_turbidity = max(current_peak_turbidity, latest_turbidity)
        if latest_conductivity: current_peak_ec = max(current_peak_ec, latest_conductivity)
        if latest_ph: current_peak_ph = max(current_peak_ph, latest_ph)
        if latest_do: current_peak_do = max(current_peak_do, latest_do)
        
        logger.info(f"Initialized peaks: Temp={current_peak_temp}, Turbidity={current_peak_turbidity}, EC={current_peak_ec}, pH={current_peak_ph}, DO={current_peak_do}")
        logger.info(f"GPS mode: {gps_mode}")
    except Exception as e:
        logger.error(f"Database init error: {str(e)}")
        db.drop_all()
        db.create_all()

# Function to connect to Arduino
def connect_to_arduino():
    global ser, arduino_connected, reconnect_attempts, first_connection_time
    
    try:
        if ser and ser.is_open: ser.close()
            
        ports = serial.tools.list_ports.comports()
        logger.info(f"Available ports: {[p.device for p in ports]}")
            
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        arduino_connected = True
        reconnect_attempts = 0
        logger.info(f"Connected to Arduino on {SERIAL_PORT}")
        ser.flushInput()
        
        if not first_connection_time:
            first_connection_time = datetime.now()
            logger.info(f"First connection: {first_connection_time}")
            
        return True
    except Exception as e:
        logger.error(f"Connection error: {str(e)}")
        ser = None
        arduino_connected = False
        reconnect_attempts += 1
        return False

def serial_reader():
    global latest_turbidity, latest_temperature, latest_conductivity, last_update
    global latest_ph, latest_do
    global arduino_connected, ser, gps_fix_acquired, last_real_gps_time, gps_mode
    global latest_latitude, latest_longitude

    while True:
        if not arduino_connected:
            if reconnect_attempts < MAX_RECONNECT_ATTEMPTS:
                logger.info(f"Reconnect attempt {reconnect_attempts+1}/{MAX_RECONNECT_ATTEMPTS}")
                if connect_to_arduino():
                    continue
                else:
                    time.sleep(RECONNECT_DELAY)
                    continue
            else:
                time.sleep(RECONNECT_DELAY * 2)
                continue

        if ser and ser.is_open:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                logger.info(f"Raw: {line}")

                if line and "|" in line:
                    parts = line.split("|")
                    values = {}
                    gps_data_received = False

                    for part in parts:
                        key_val = part.strip().split(":")
                        if len(key_val) >= 2:
                            key = key_val[0].strip()
                            val = ":".join(key_val[1:]).strip()

                            clean_val = ''.join([c for c in val if c in '0123456789.-'])

                            if key == "TURB" and clean_val:
                                values["turbidity"] = float(clean_val)
                            elif key == "TEMP" and clean_val:
                                values["temperature"] = float(clean_val)
                            elif key == "EC" and clean_val:
                                values["conductivity"] = float(clean_val)
                            elif key == "PH" and clean_val:
                                values["ph"] = float(clean_val)
                            elif key == "DO" and clean_val:
                                values["do"] = float(clean_val)
                            elif key == "LAT" and clean_val:
                                values["latitude"] = float(clean_val)
                                gps_data_received = True
                            elif key == "LON" and clean_val:
                                values["longitude"] = float(clean_val)
                                gps_data_received = True

                    # Improved GPS detection and logging
                    valid_coords = (
                        "latitude" in values and 
                        "longitude" in values and
                        values["latitude"] != 0.0 and 
                        values["longitude"] != 0.0
                    )

                    gps_source = ""  # Track GPS source for this reading

                    if valid_coords:
                        # Always prioritize valid coordinates
                        gps_fix_acquired = True
                        last_real_gps_time = datetime.now()
                        gps_mode = "real"
                        gps_source = "real (current)"
                    elif not gps_fix_acquired:
                        # Only simulate if we've never had a real fix
                        lat_offset = (random.random() - 0.5) * 0.01
                        lon_offset = (random.random() - 0.5) * 0.01
                        values["latitude"] = BASE_LATITUDE + lat_offset
                        values["longitude"] = BASE_LONGITUDE + lon_offset
                        gps_mode = "simulated"
                        gps_source = "simulated"
                    else:
                        # Use cached coordinates when we have fix but no new data
                        gps_source = "real (cached)"

                    if values:
                        with data_lock:
                            if "turbidity" in values: latest_turbidity = values["turbidity"]
                            if "temperature" in values: latest_temperature = values["temperature"]
                            if "conductivity" in values: latest_conductivity = values["conductivity"]
                            if "ph" in values: latest_ph = values["ph"]
                            if "do" in values: latest_do = values["do"]
                            if "latitude" in values: latest_latitude = values["latitude"]
                            if "longitude" in values: latest_longitude = values["longitude"]
                            last_update = datetime.now()

                        # Log threshold alerts with new sensors
                        threshold_count = 0
                        if "turbidity" in values and values["turbidity"] > TURBIDITY_THRESHOLD:
                            threshold_count += 1
                        if "temperature" in values and values["temperature"] > TEMPERATURE_THRESHOLD:
                            threshold_count += 1
                        if "conductivity" in values and values["conductivity"] > CONDUCTIVITY_THRESHOLD:
                            threshold_count += 1
                        if "ph" in values and (values["ph"] < PH_THRESHOLD_LOW or values["ph"] > PH_THRESHOLD_HIGH):
                            threshold_count += 1
                        if "do" in values and values["do"] < DO_THRESHOLD:
                            threshold_count += 1
                            
                        if threshold_count >= BALLAST_ALERT_THRESHOLD:
                            alert_msg = "BALLAST WATER ALERT! Parameters exceeded:"
                            if "turbidity" in values and values["turbidity"] > TURBIDITY_THRESHOLD:
                                alert_msg += f" Turbidity ({values['turbidity']} > {TURBIDITY_THRESHOLD} NTU)"
                            if "temperature" in values and values["temperature"] > TEMPERATURE_THRESHOLD:
                                alert_msg += f" Temperature ({values['temperature']} > {TEMPERATURE_THRESHOLD} °C)"
                            if "conductivity" in values and values["conductivity"] > CONDUCTIVITY_THRESHOLD:
                                alert_msg += f" Conductivity ({values['conductivity']} > {CONDUCTIVITY_THRESHOLD} mS/cm)"
                            if "ph" in values and (values["ph"] < PH_THRESHOLD_LOW or values["ph"] > PH_THRESHOLD_HIGH):
                                alert_msg += f" pH ({values['ph']} outside {PH_THRESHOLD_LOW}-{PH_THRESHOLD_HIGH})"
                            if "do" in values and values["do"] < DO_THRESHOLD:
                                alert_msg += f" Dissolved Oxygen ({values['do']} < {DO_THRESHOLD} mg/L)"
                            logger.warning(alert_msg)

                        # Updated log shows all sensors and GPS source
                        logger.info(f"Processed -> Turbidity: {latest_turbidity}, Temp: {latest_temperature}, EC: {latest_conductivity}, pH: {latest_ph}, DO: {latest_do}, GPS: {gps_source}")

            except Exception as e:
                logger.error(f"Serial error: {str(e)}")
                arduino_connected = False
                ser = None
        else:
            arduino_connected = False

        time.sleep(0.1)

def generate_peak_log():
    with data_lock:
        if gps_fix_acquired and latest_latitude and latest_longitude:
            latitude = latest_latitude
            longitude = latest_longitude
            gps_type = "real"
        else:
            latitude = BASE_LATITUDE + (random.random() - 0.5) * 0.01
            longitude = BASE_LONGITUDE + (random.random() - 0.5) * 0.01
            gps_type = "simulated"

        return {
            'peak_temperature': round(current_peak_temp, 2),
            'peak_turbidity': round(current_peak_turbidity, 2),
            'peak_ec': round(current_peak_ec, 2),
            'peak_ph': round(current_peak_ph, 2),
            'peak_do': round(current_peak_do, 2),
            'latitude': round(latitude, 6),
            'longitude': round(longitude, 6),
            'gps_type': gps_type
        }

# Logger every 3 seconds
def sensor_logger():
    global current_peak_temp, current_peak_turbidity, current_peak_ec, current_peak_ph, current_peak_do, last_peak_log_time
    global gps_fix_acquired, last_real_gps_time, gps_mode
    global latest_latitude, latest_longitude

    # Check GPS timeout (5 minutes without real GPS)
    def check_gps_timeout():
        global gps_fix_acquired, gps_mode
        if gps_fix_acquired and last_real_gps_time:
            time_since_last_fix = (datetime.now() - last_real_gps_time).total_seconds()
            if time_since_last_fix > 300:
                logger.warning(f"⏱ GPS timeout: {int(time_since_last_fix)} seconds")
                gps_fix_acquired = False
                gps_mode = "simulated"
                logger.warning("⚠️ GPS fix lost, switching to simulated")

    while True:
        time.sleep(3)
        check_gps_timeout()
        
        # Skip logging if Arduino is disconnected
        if not arduino_connected:
            logger.warning("Arduino disconnected - skipping logging")
            time.sleep(10)  # wait longer before next check
            continue
            
        with app.app_context():
            if (latest_turbidity is not None and 
                latest_temperature is not None and 
                latest_conductivity is not None and
                latest_ph is not None and
                latest_do is not None):
                
                # Use real coordinates if available
                if gps_fix_acquired and latest_latitude and latest_longitude:
                    latitude = latest_latitude
                    longitude = latest_longitude
                    gps_type = "real"
                else:
                    latitude = BASE_LATITUDE + (random.random() - 0.5) * 0.01
                    longitude = BASE_LONGITUDE + (random.random() - 0.5) * 0.01
                    gps_type = "simulated"
                
                # Check if at least two sensors exceed thresholds
                threshold_count = 0
                if latest_turbidity > TURBIDITY_THRESHOLD:
                    threshold_count += 1
                if latest_temperature > TEMPERATURE_THRESHOLD:
                    threshold_count += 1
                if latest_conductivity > CONDUCTIVITY_THRESHOLD:
                    threshold_count += 1
                if latest_ph < PH_THRESHOLD_LOW or latest_ph > PH_THRESHOLD_HIGH:
                    threshold_count += 1
                if latest_do < DO_THRESHOLD:
                    threshold_count += 1
                    
                above_threshold = threshold_count >= BALLAST_ALERT_THRESHOLD
                
                new_entry = SensorData(
                    turbidity=round(latest_turbidity, 2),
                    temperature=round(latest_temperature, 2),
                    conductivity=round(latest_conductivity, 2),
                    ph=round(latest_ph, 2),
                    do=round(latest_do, 2),
                    latitude=round(latitude, 6),
                    longitude=round(longitude, 6),
                    gps_type=gps_type,
                    timestamp=datetime.now(),
                    above_threshold=above_threshold
                )
                db.session.add(new_entry)
                
                # Update peak values
                current_peak_temp = max(current_peak_temp, latest_temperature)
                current_peak_turbidity = max(current_peak_turbidity, latest_turbidity)
                current_peak_ec = max(current_peak_ec, latest_conductivity)
                current_peak_ph = max(current_peak_ph, latest_ph)
                current_peak_do = max(current_peak_do, latest_do)
                
                # Log peak values every 5 minutes
                now = datetime.now()
                if (now - last_peak_log_time).total_seconds() >= 300:
                    peak_data = generate_peak_log()
                    new_peak_entry = PeakLog(
                        peak_temperature=peak_data['peak_temperature'],
                        peak_turbidity=peak_data['peak_turbidity'],
                        peak_ec=peak_data['peak_ec'],
                        peak_ph=peak_data['peak_ph'],
                        peak_do=peak_data['peak_do'],
                        latitude=peak_data['latitude'],
                        longitude=peak_data['longitude'],
                        gps_type=peak_data['gps_type'],
                        timestamp=now
                    )
                    db.session.add(new_peak_entry)
                    logger.info(f"Logged peaks: Temp={peak_data['peak_temperature']}, Turbidity={peak_data['peak_turbidity']}, EC={peak_data['peak_ec']}, pH={peak_data['peak_ph']}, DO={peak_data['peak_do']}")
                    
                    # Reset peaks
                    current_peak_temp = -float('inf')
                    current_peak_turbidity = -float('inf')
                    current_peak_ec = -float('inf')
                    current_peak_ph = -float('inf')
                    current_peak_do = -float('inf')
                    last_peak_log_time = now
                
                db.session.commit()
            else:
                time.sleep(10)

def cleanup_scheduler():
    while True:
        cleanup_old_data()
        time.sleep(86400)

# Cleanup old data
def cleanup_old_data():
    with app.app_context():
        cutoff_time = datetime.utcnow() - timedelta(days=31)
        deleted_sensor = SensorData.query.filter(SensorData.timestamp < cutoff_time).delete()
        deleted_peaks = PeakLog.query.filter(PeakLog.timestamp < cutoff_time).delete()
        db.session.commit()
        logger.info(f"Deleted {deleted_sensor} SensorData and {deleted_peaks} PeakLog entries.")

# Improved Camera Functions for both Raspberry Pi Camera and USB Webcam
def init_camera():
    """Initialize camera with support for both Raspberry Pi Camera and USB Webcam"""
    global camera, camera_initialized, camera_type
    
    # Reset camera state
    camera_initialized = False
    camera = None
    camera_type = "none"
    
    # Method 1: Try Raspberry Pi Camera (picamera2)
    try:
        from picamera2 import Picamera2
        camera = Picamera2()
        config = camera.create_video_configuration(main={"size": (640, 480)})
        camera.configure(config)
        camera.start()
        time.sleep(2)  # Camera warm-up
        camera_initialized = True
        camera_type = "picamera2"
        logger.info("✅ Raspberry Pi Camera (picamera2) initialized successfully")
        return True
    except ImportError:
        logger.info("picamera2 not available in virtual environment")
    except Exception as e:
        logger.warning(f"picamera2 failed: {str(e)}")
    
    # Method 2: Try Legacy Raspberry Pi Camera (picamera)
    try:
        import picamera
        camera = picamera.PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 20
        time.sleep(2)  # Camera warm-up
        camera_initialized = True
        camera_type = "picamera"
        logger.info("✅ Legacy Raspberry Pi Camera (picamera) initialized successfully")
        return True
    except ImportError:
        logger.info("picamera not available in virtual environment")
    except Exception as e:
        logger.warning(f"picamera failed: {str(e)}")
    
    # Method 3: Try USB Webcam with OpenCV (Primary fallback)
    logger.info("Attempting to initialize USB Webcam...")
    
    # Try different camera indices with better detection
    camera_indices = [0, 1, 2, 3, 4]  # Try more indices
    backend_preferences = [cv2.CAP_V4L2, cv2.CAP_ANY]  # Prefer V4L2 backend on Raspberry Pi
    
    for backend in backend_preferences:
        for i in camera_indices:
            try:
                logger.info(f"Trying camera index {i} with backend {backend}")
                camera = cv2.VideoCapture(i, backend)
                
                if camera.isOpened():
                    # Set camera properties
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    camera.set(cv2.CAP_PROP_FPS, 20)
                    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to minimize latency
                    
                    # Test if we can actually read from this camera
                    ret, frame = camera.read()
                    if ret and frame is not None:
                        logger.info(f"✅ USB Webcam (index {i}, backend {backend}) initialized successfully")
                        logger.info(f"Frame dimensions: {frame.shape}")
                        camera_initialized = True
                        camera_type = f"usb_webcam_{i}"
                        return True
                    else:
                        logger.warning(f"Camera index {i} opened but cannot read frames")
                        camera.release()
                        camera = None
                else:
                    logger.info(f"Camera index {i} not available with backend {backend}")
                    
            except Exception as e:
                logger.warning(f"USB Webcam index {i} failed: {str(e)}")
                if camera:
                    camera.release()
                    camera = None
    
    # If all methods fail
    logger.warning("❌ All camera initialization methods failed")
    camera = None
    camera_initialized = False
    camera_type = "none"
    return False

def generate_placeholder_frame(error_msg=None):
    """Generate a placeholder frame when camera is unavailable"""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Add background gradient (blue water theme)
    for y in range(480):
        color = int(50 + (y / 480) * 50)
        cv2.line(frame, (0, y), (640, y), (color, color, 100), 1)
    
    # Add text
    cv2.putText(frame, "AQUATIC DRONE CAMERA", (120, 150), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    if error_msg:
        cv2.putText(frame, "Camera Feed Unavailable", (180, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"Error: {error_msg[:40]}...", (150, 230), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    else:
        cv2.putText(frame, "Camera Initializing...", (200, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Add timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(frame, timestamp, (220, 250), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    
    # Add Raspberry Pi logo/indicator
    cv2.putText(frame, "Raspberry Pi", (260, 280), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
    
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buffer.tobytes()

def generate_frames():
    """Generate camera frames with robust error handling"""
    global camera, camera_initialized, camera_type
    
    frame_count = 0
    last_time = time.time()
    consecutive_failures = 0
    max_consecutive_failures = 5
    
    while True:
        try:
            frame = None
            
            # USB Webcam method (OpenCV)
            if "usb_webcam" in camera_type and hasattr(camera, 'read'):
                try:
                    with camera_lock:
                        # Sometimes we need to grab multiple times to clear buffer
                        for _ in range(2):
                            camera.grab()
                        success, frame = camera.retrieve()
                        
                    if not success or frame is None:
                        consecutive_failures += 1
                        logger.warning(f"Failed to read frame from USB webcam (attempt {consecutive_failures})")
                        if consecutive_failures >= max_consecutive_failures:
                            logger.error("Too many consecutive failures, reinitializing camera")
                            init_camera()  # Try to reinitialize
                            consecutive_failures = 0
                        frame = None
                    else:
                        consecutive_failures = 0  # Reset on success
                        
                except Exception as e:
                    logger.error(f"USB Webcam read error: {str(e)}")
                    frame = None
                    consecutive_failures += 1
            
            # Process frame if available
            if frame is not None:
                # Add timestamp and info to frame
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, f"Aquatic Drone - {timestamp}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Add camera type info
                cv2.putText(frame, f"Camera: {camera_type}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Add status info
                cv2.putText(frame, "Status: LIVE", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Encode as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [
                    cv2.IMWRITE_JPEG_QUALITY, 80  # Good quality for webcam
                ])
                
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    
                    # Calculate FPS (for logging)
                    frame_count += 1
                    current_time = time.time()
                    if current_time - last_time >= 10.0:  # Log every 10 seconds
                        fps = frame_count / (current_time - last_time)
                        logger.info(f"Camera streaming at {fps:.1f} FPS - {camera_type}")
                        frame_count = 0
                        last_time = current_time
                else:
                    raise Exception("Frame encoding failed")
                    
            else:
                # Create placeholder frame when no camera available
                frame_bytes = generate_placeholder_frame("No frame received from camera")
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
        except Exception as e:
            logger.error(f"Frame generation error: {str(e)}")
            frame_bytes = generate_placeholder_frame(error_msg=str(e))
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        # Control frame rate
        time.sleep(0.033)  # ~30 FPS

# Routes
@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/api/real-time-data')
def get_real_time_data():
    with data_lock:
        return jsonify({
            'turbidity': latest_turbidity,
            'temperature': latest_temperature,
            'conductivity': latest_conductivity,
            'ph': latest_ph,
            'do': latest_do,
            'timestamp': last_update.isoformat() if last_update else None,
            'connected': arduino_connected,
            'gps_mode': gps_mode,
            'gps_fix_acquired': gps_fix_acquired,
            'last_real_gps_time': last_real_gps_time.isoformat() if last_real_gps_time else None,
            'latitude': latest_latitude,
            'longitude': latest_longitude,
            'thresholds': {
                'turbidity': TURBIDITY_THRESHOLD,
                'temperature': TEMPERATURE_THRESHOLD,
                'conductivity': CONDUCTIVITY_THRESHOLD,
                'ph_low': PH_THRESHOLD_LOW,
                'ph_high': PH_THRESHOLD_HIGH,
                'dissolved_oxygen': DO_THRESHOLD
            }
        })

@app.route('/api/historical-data')
def get_historical_data():
    global arduino_connected, last_update
    
    # If disconnected, get data only up to the last update time
    if not arduino_connected and last_update:
        entries = SensorData.query.filter(
            SensorData.timestamp <= last_update
        ).order_by(SensorData.timestamp.desc()).limit(20).all()
    else:
        entries = SensorData.query.order_by(SensorData.timestamp.desc()).limit(20).all()
        
    entries.reverse()
    data = [{
        'time': entry.timestamp.isoformat(),
        'turbidity': entry.turbidity,
        'temperature': entry.temperature,
        'conductivity': entry.conductivity,
        'ph': entry.ph,
        'do': entry.do,
        'lat': entry.latitude,
        'lon': entry.longitude,
        'gps_type': entry.gps_type,
        'above_threshold': entry.above_threshold
        
    } for entry in entries]
    return jsonify(data)

# API endpoint for peak logs
@app.route('/api/peak-logs')
def get_peak_logs():
    one_month_ago = datetime.utcnow() - timedelta(days=30)
    logs = PeakLog.query.filter(PeakLog.timestamp >= one_month_ago).order_by(PeakLog.timestamp.desc()).all()
    data = [{
        'time': log.timestamp.isoformat(),
        'temperature': log.peak_temperature,
        'turbidity': log.peak_turbidity,
        'ec': log.peak_ec,
        'ph': log.peak_ph,
        'do': log.peak_do,
        'latitude': log.latitude,
        'longitude': log.longitude,
        'gps_type': log.gps_type
    } for log in logs]
    return jsonify(data)

# New endpoint for 3D peak data
@app.route('/api/3d-peak-data')
def get_3d_peak_data():
    logs = PeakLog.query.order_by(PeakLog.timestamp.asc()).all()
    timestamps = []
    turbidity = []
    temperature = []
    conductivity = []
    ph = []
    do = []
    latitudes = []
    longitudes = []
    gps_types = []
    
    for log in logs:
        timestamps.append(log.timestamp.timestamp())
        turbidity.append(log.peak_turbidity)
        temperature.append(log.peak_temperature)
        conductivity.append(log.peak_ec)
        ph.append(log.peak_ph)
        do.append(log.peak_do)
        latitudes.append(log.latitude)
        longitudes.append(log.longitude)
        gps_types.append(log.gps_type)
    
    return jsonify({
        'timestamps': timestamps,
        'turbidity': turbidity,
        'temperature': temperature,
        'conductivity': conductivity,
        'ph': ph,
        'do': do,
        'latitudes': latitudes,
        'longitudes': longitudes,
        'gps_types': gps_types
    })

# API to get last data timestamp
@app.route('/api/last-data-timestamp')
def get_last_data_timestamp():
    last_entry = SensorData.query.order_by(SensorData.timestamp.desc()).first()
    if last_entry:
        return jsonify({
            'timestamp': last_entry.timestamp.isoformat(),
            'gps_type': last_entry.gps_type
        })
    else:
        return jsonify({'timestamp': None, 'gps_type': None})

# Serial debug endpoint
@app.route('/api/serial-debug')
def serial_debug():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return jsonify({
        'port': SERIAL_PORT,
        'connected': arduino_connected,
        'reconnect_attempts': reconnect_attempts,
        'last_update': last_update.isoformat() if last_update else None,
        'available_ports': ports,
        'gps_mode': gps_mode,
        'gps_fix_acquired': gps_fix_acquired,
        'last_real_gps_time': last_real_gps_time.isoformat() if last_real_gps_time else None
    })

# Manual reconnect endpoint
@app.route('/api/reconnect-arduino')
def manual_reconnect():
    global reconnect_attempts
    reconnect_attempts = 0
    if connect_to_arduino():
        return jsonify({'status': 'success', 'message': 'Reconnected to Arduino'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to reconnect'})

# Camera status endpoint
@app.route('/api/camera-status')
def camera_status():
    """API endpoint to check camera status"""
    global camera, camera_initialized, camera_type
    
    status = "unknown"
    details = ""
    
    if not camera_initialized:
        status = "not_initialized"
        details = "Camera not initialized"
    elif camera_type == "picamera2" and hasattr(camera, 'started') and camera.started:
        status = "running"
        details = "Using Raspberry Pi Camera (picamera2)"
    elif camera_type == "picamera" and hasattr(camera, '_camera'):
        status = "running" 
        details = "Using Legacy Raspberry Pi Camera (picamera)"
    elif "usb_webcam" in camera_type and hasattr(camera, 'isOpened') and camera.isOpened():
        status = "running"
        details = f"Using USB Webcam ({camera_type})"
    else:
        status = "error"
        details = "Camera initialized but not functioning properly"
    
    return jsonify({
        'status': status,
        'type': camera_type,
        'initialized': camera_initialized,
        'details': details
    })

# Manual camera reconnect endpoint
@app.route('/api/reconnect-camera')
def manual_camera_reconnect():
    """Manual camera reconnection endpoint"""
    global camera_initialized
    success = init_camera()
    if success:
        return jsonify({'status': 'success', 'message': f'Camera reconnected: {camera_type}'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to reconnect camera'})

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Run
if __name__ == '__main__':
    with app.app_context():
        if not os.path.exists('sensors.db'):
            logger.info("Creating new database")
            db.create_all()
        else:
            logger.info("Using existing database")
        initialize_from_database()
    
    # Initialize camera with improved error handling
    camera_init_success = init_camera()
    if camera_init_success:
        logger.info(f"✅ Camera initialized successfully: {camera_type}")
    else:
        logger.warning("⚠️ Camera initialization failed - video feed will show placeholder")
    
    connect_to_arduino()
    threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=sensor_logger, daemon=True).start()
    threading.Thread(target=cleanup_scheduler, daemon=True).start()

    ports = serial.tools.list_ports.comports()
    logger.info(f"Available COM ports: {[p.device for p in ports]}")
    
    app.run(debug=True, use_reloader=False, host='0.0.0.0', port=5000)