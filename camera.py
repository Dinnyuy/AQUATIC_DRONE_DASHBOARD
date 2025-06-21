#from picamera import PiCamera
#import time

#camera = PiCamera()
#camera.resolution = (640, 480)

#def generate_frames():
    #while True:
        #camera.capture('static/frame.jpg', use_video_port=True)
        #yield (b'--frame\r\n'
               #b'Content-Type: image/jpeg\r\n\r\n' + open('static/frame.jpg', 'rb').read() + b'\r\n')
        #time.sleep(0.1)