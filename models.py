from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class SensorData(db.Model):
    # Same as in app.py (redundant but shows separation)
    pass