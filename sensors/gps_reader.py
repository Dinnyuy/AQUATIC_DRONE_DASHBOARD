#import serial

#def get_gps_data():
    #gps = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)
    #while True:
        #line = gps.readline().decode('utf-8', errors='ignore')
        #if line.startswith('$GPGGA'):
            #data = line.split(',')
            #return {
                #'lat': float(data[2][:2]) + float(data[2][2:])/60,
                #'lon': float(data[4][:3]) + float(data[4][3:])/60
            #}