import time
import math
import mpu6050
import RPi.GPIO as GPIO
from time import sleep
import cv2
from picamera2 import Picamera2



# servo

servoPin          = 12 
SERVO_MAX_DUTY    = 12 
SERVO_MIN_DUTY    = 3   

GPIO.setmode(GPIO.BOARD)  
GPIO.setup(12, GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)


servo = GPIO.PWM(12, 50)
servo.start(0) 
servox = GPIO.PWM(11,50)
servox.start(0)
servoCatch = GPIO.PWM(15,50)
servoCatch.start(0)

# Sensor initialization
mpu = mpu6050.MPU6050()    
mpu.dmpInitialize()      
mpu.setDMPEnabled(True)   

# get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize()   

count = 0                 
past = time.time()        


sample_rate = mpu.getRate()   
dlp_mode = mpu.getDLPFMode()  
gyro_fs = mpu.getFullScaleGyroRange()     
accel_fs = mpu.getFullScaleAccelRange()   
print('sample_rate = ', sample_rate,
      ', dlp_mode = ', dlp_mode,
      ', gyro_fs = ', gyro_fs,
      ', accel_fs = ', accel_fs)
      
def mapping(x,input_min,input_max,output_min,output_max):
  return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

pastx=0
pastz=0

def setServoPos(degreez,degreex):
    global pastx, pastz
    if degreez > 180:
      degreez = 180
    if degreex >180:
      degreex=180
      
    xb=True
    zb=True
    if pastx-7<=degreex and pastx+7>=degreex:
      xb=False
    if pastz-4<=degreez and pastz+4>=degreez:
      xz=False
    
    print("pastx, pastz",pastx, degreex)

  
    duty = SERVO_MIN_DUTY+(degreez*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    dutyx = SERVO_MIN_DUTY+(degreex*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    print("Degree: {} to {}(Duty)".format(degreez, duty))
    print("Degreex : {} Degreez: {}".format(duty,dutyx))
    if xb:
      servox.ChangeDutyCycle(dutyx)
      pastx=degreex
    if zb:
      servo.ChangeDutyCycle(duty)
      pastz=degreez


# button
GPIO.setwarnings(False)

z=0
x=0
saveAction=[]

face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

def face_detect():

    
    if len(saveAction) == 0:
      return

    servoCatch.ChangeDutyCycle(7)

    while True:
        print("detecting..")
        im = picam2.capture_array()

        grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        faces = face_detector.detectMultiScale(grey, 1.1, 5)
        if len(faces)>0:
            for (x, y, w, h) in faces:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0))

            cv2.imshow("Camera", im)
            cv2.waitKey(1)
            
            for z,x in saveAction:
              setServoPos(z,x)
              sleep(0.05)
            
            servoCatch.ChangeDutyCycle(12)
            break

def button_callback(channel):
  global saveAction
  i=0
  saveAction=[]
  while i<200:
    print("saving....")
    saveAction.append((z,x))
    i+=1
    sleep(0.05)


GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(16,GPIO.RISING,callback=button_callback)





while True:
    if GPIO.input(18) == GPIO.HIGH:
        face_detect()

    mpuIntStatus = mpu.getIntStatus()

    if mpuIntStatus >= 2:

        fifoCount = mpu.getFIFOCount()

        if (fifoCount == 1024) or (mpuIntStatus & 0x10):
            mpu.resetFIFO()
            print('overflow!', fifoCount, mpuIntStatus & 0x10)

        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()

        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        ypr = mpu.dmpGetYawPitchRoll(q, g)

        new = time.time()

        print("z축",ypr['yaw'] * 180 / math.pi, ',',
                    "y축",ypr['pitch'] * 180 / math.pi, ',',
                    "x축",ypr['roll'] * 180 / math.pi, ',',
                    new - past)
  #            print("z축",ypr['yaw'] * 180 / math.pi)
  #            print("y축",ypr['pitch'] * 180 / math.pi, ',')
  #            print("x축",ypr['roll'] * 180 / math.pi, ',')

        z = int(mapping(ypr['yaw']*180/math.pi,-180,180,0,180))
        x= int(mapping(ypr['roll']*180/math.pi,-80,80,0,180))
        setServoPos(z,x)
        sleep(0.001)

        fifoCount -= packetSize

        past = new
        count += 1

