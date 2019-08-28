import RPi.GPIO as GPIO

# http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
# http://blog.templaro.com/reading-a-rotary-encoder-demo-code-for-atmega328/

class Encoder:
    count = 0
    degrees = 0

    def __init__(self, pinAin, pinBin):
        print("Setting up Rotary Encoder...")
        self.pinA = pinAin
        self.pinB = pinBin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinA, GPIO.IN)
        GPIO.setup(self.pinB, GPIO.IN)



    def run(self):
        lastAB = 0b00
        outcome = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]

        while True:
            currentA = GPIO.input(self.pinA)
            currentB = GPIO.input(self.pinB)

            currentAB = (currentA << 1) | currentB
            position = (lastAB << 2) | currentAB
            Encoder.count += outcome[position]
            lastAB = currentAB
            # encoder is 200 counts/turn *2 for AB phases * 1.8 ratio = 720
            Encoder.degrees = (Encoder.count/1440.0)*360.0
            print(Encoder.count)
            print(Encoder.degrees)




test = Encoder(3,4)
test.run()
GPIO.cleanup()
