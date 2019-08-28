from rpisteppermotor.py import MotorControl
import socketio

DIR_PIN = 17
STEP_PIN = 24

#init 
sio = socketio.Client()
motor = MotorControl(DIR_PIN, STEP_PIN)

#events
@sio.event
def connect():
    print("connected o server...")
    
@sio.on('on_rotate_platform')
def rotate(steps) :
    print("rotating..")
    motor.move(int(steps), 100, 10)

#connect
sio.connect('http://localhost:5000')