from flask import Flask, render_template, Response, jsonify
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import random
import time
import serial
import threading
import logging

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///sensors.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('ArduinoDashboard')

# Serial config
SERIAL_PORT = 'COM8'
BAUD_RATE = 9600
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    logger.info(f"Connected to Arduino on {SERIAL_PORT}")
except Exception as e:
    logger.error(f"Serial connection error: {str(e)}")
    ser = None

# Globals
latest_turbidity = 0.0
latest_temperature = 0.0
last_update = None
first_connection_time = None
data_lock = threading.Lock()

# DB model
class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    turbidity = db.Column(db.Float)
    temperature = db.Column(db.Float)
    conductivity = db.Column(db.Float)
    latitude = db.Column(db.Float)
    longitude = db.Column(db.Float)

# Serial reading thread
def serial_reader():
    global latest_turbidity, latest_temperature, last_update, first_connection_time
    while True:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("TURBIDITY:") and "TEMPERATURE:" in line:
                    parts = line.split(";")
                    turbidity_val = float(parts[0].split(":")[1])
                    temperature_val = float(parts[1].split(":")[1])

                    with data_lock:
                        latest_turbidity = turbidity_val
                        latest_temperature = temperature_val
                        now = datetime.now()
                        last_update = now
                        if not first_connection_time:
                            first_connection_time = now
                            logger.info(f"Arduino first connected at: {first_connection_time}")
            except Exception as e:
                logger.error(f"Serial read error: {str(e)}")
        time.sleep(1)

# Generate mixed data (real + simulated)
def generate_sensor_data():
    with data_lock:
        turbidity_val = latest_turbidity
        temperature_val = latest_temperature
    return {
        'turbidity': round(turbidity_val, 2),
        'temperature': round(temperature_val, 2),
        'conductivity': round(random.uniform(52, 58), 2),  # Only simulated
        'latitude': round(4.2105 + random.uniform(-0.01, 0.01), 6),
        'longitude': round(6.4375 + random.uniform(-0.01, 0.01), 6)
    }

# Log every 3 seconds
def sensor_logger():
    while True:
        time.sleep(3)
        with app.app_context():
            if first_connection_time:
                data = generate_sensor_data()
                new_entry = SensorData(
                    turbidity=data['turbidity'],
                    temperature=data['temperature'],
                    conductivity=data['conductivity'],
                    latitude=data['latitude'],
                    longitude=data['longitude'],
                    timestamp=datetime.now()
                )
                db.session.add(new_entry)
                db.session.commit()
                logger.info(f"Logged: {data}")

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
            'timestamp': last_update.isoformat() if last_update else None,
            'connected': ser is not None and ser.is_open
        })

@app.route('/api/historical-data')
def get_historical_data():
    if first_connection_time:
        entries = SensorData.query.filter(SensorData.timestamp >= first_connection_time).order_by(SensorData.timestamp).all()
    else:
        last_10 = SensorData.query.order_by(SensorData.timestamp.desc()).limit(10).all()
        entries = reversed(last_10)

    data = [{
        'time': entry.timestamp.isoformat(),
        'turbidity': entry.turbidity,
        'temperature': entry.temperature,
        'conductivity': entry.conductivity,
        'lat': entry.latitude,
        'lon': entry.longitude
    } for entry in entries]

    return jsonify(data)

@app.route('/video_feed')
def video_feed():
    def generate_camera_frames():
        while True:
            try:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       open("static/placeholder.jpg", "rb").read() + b'\r\n')
                time.sleep(1)
            except FileNotFoundError:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n\r\n')
                time.sleep(1)
    return Response(generate_camera_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Startup
if __name__ == '__main__':
    with app.app_context():
        db.create_all()
    if ser:
        threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=sensor_logger, daemon=True).start()
    app.run(debug=True, use_reloader=False)
