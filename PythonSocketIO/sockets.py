import socketio  # pip install "python-socketio[client]"
import json
# standard Python
sio = socketio.Client()


@sio.event
def connect():
    print("I'm connected!")
    sio.emit('onLidarStateChanged', "payload")


@sio.event
def onMoveByStepsTriggered(payload):
    print("move by steps triggered")
    print(payload["steps"])
    print(payload["acceleration"])


sio.connect('https://capstone-application-server.herokuapp.com')
