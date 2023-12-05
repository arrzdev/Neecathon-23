from flask import Flask, jsonify
from serial import Serial
from time import sleep
from threading import Thread

app = Flask(__name__)

class AccelSensor:
  def __init__(self):
    self.serial_port = '/dev/ttyACM0'
    self.baud_rate = 115200

    self.data = []
    self.sensor_thread = Thread(target=self.read_sensor_data)

  def get_data(self):
    return self.data

  def read_sensor_data(self):
    with Serial(self.serial_port, self.baud_rate, timeout=1) as serial:
      while True:
        if len(self.data) > 100:
          return

        if (not serial.readable()):
          continue

        line_data = serial.readline().decode('utf-8').strip()
        if (line_data == ''):
          continue

        unpiped_data = line_data.split('|')
        if (len(unpiped_data) != 2):
          continue

        coords, sound_value = unpiped_data[0], unpiped_data[1]
        args = coords.split(',')

        if (len(args) != 6):
          continue
        
        print(args)

        self.data.append({
          'acc': {
            'x': float(args[0]),
            'y': float(args[1]),
            'z': float(args[2])
          },
          'gyro': {
            'x': float(args[3]),
            'y': float(args[4]),
            'z': float(args[5])
          },
        })

sensor = AccelSensor()
@app.route('/', methods=['GET'])
def get_data():
  return jsonify({"data": sensor.get_data()})

if __name__ == '__main__':
  sensor.sensor_thread.start() #start the background thread   
  app.run(debug=True, host="0.0.0.0")
