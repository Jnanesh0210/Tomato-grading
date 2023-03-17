import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner
import RPi.GPIO as GPIO
runner = None
show_camera = False
framee_count=0
servo_pin = 25
servo1 = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
GPIO.setup(servo1, GPIO.OUT)
# setup PWM process
pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
pwm1 = GPIO.PWM(servo1,50)
pwm.start(7) # start PWM by rotating to 90 degrees
pwm1.start(7)
pwm1.ChangeDutyCycle(2.0)
pwm.ChangeDutyCycle(2.0) #close
def now():
    return round(time.time() * 1000)
def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" %port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()
            if ret:
                backendName =camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) found in port %s " %(backendName,h,w, port))
                port_ids.append(port)
            camera.release()
    return port_ids
def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)
def help():
    print('python classify.py <path_to_model.eim> <Camera port ID, only required when more than 1 camera is present>')
def main(argv):
    framee_count=0
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)
    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()
    if len(args) == 0:
        help()
        sys.exit(2)
    model = args[0]
    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)
    print('MODEL: ' + modelfile)
    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            if len(args)>= 2:
                videoCaptureDeviceId = int(args[1])
            else:
                port_ids = get_webcams()
                if len(port_ids) == 0:
                    raise Exception('Cannot find any webcams')
                if len(args)<= 1 and len(port_ids)> 1:
                    raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
                videoCaptureDeviceId = int(port_ids[0])
            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) in port %s selected." %(backendName,h,w, videoCaptureDeviceId))
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")
            next_frame = 0 # limit to ~10 fps here
            for res, img in runner.classifier(videoCaptureDeviceId):
                if (next_frame > now()):
                    time.sleep((next_frame - now()) / 1000)
                # print('classification runner response', res)
                data = []
                framee_count  = framee_count +1
                print("Frames: ", framee_count)
                if "classification" in res["result"].keys():
                    print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                    for label in labels:
                        score = res['result']['classification'][label]
                    #    print(score)
                        print('%s: %.2f\t' % (label, score), end='')
                        data.append(score)
                    print('', flush=True)
                    Green = round(data[0],2)
                    Red = round(data[1],2)
                    Uncertain = round(data[2],2)
                    print(Green, Red, Uncertain)                    
                    if (Green >=0.25 and framee_count%10 ==0):
                        while(Green >=0.25):
                            pwm1.ChangeDutyCycle(12.0)
                            print("Green Tomato Detected")
                            time.sleep(0.500)
                            pwm1.ChangeDutyCycle(2.0) #close
                            time.sleep(0.250)
                            pwm.ChangeDutyCycle(7.0)
                            time.sleep(0.450)
                            pwm.ChangeDutyCycle(2.0)
                            Green=0.01
#                            time.sleep(2)                            
                    if (Red >=0.50 and framee_count%10 ==0):
                        while(Red >=0.50):
                            pwm1.ChangeDutyCycle(7.0)
                            print("Red Tomato Detected")
                            time.sleep(0.500)
                            pwm1.ChangeDutyCycle(2.0)
                            time.sleep(0.250)
                            pwm.ChangeDutyCycle(7.0)
                            time.sleep(0.450)
                            pwm.ChangeDutyCycle(2.0)
                            Red=0.01
 #                           time.sleep(2)
                    else:
                        time.sleep(0.01)                       
                   # print('%s: %.2f\t' % (Green,Red,Uncertain), end ='')
                    if (show_camera):
                        cv2.imshow('edgeimpulse', img)
                        if cv2.waitKey(1) == ord('q'):
                            break
                elif "bounding_boxes" in res["result"].keys():
                    print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                    for bb in res["result"]["bounding_boxes"]:
                        print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                next_frame = now() + 100
        finally:
            if (runner):
                runner.stop()
# framee_count=0
if __name__ == "__main__":
    main(sys.argv[1:])
cap.release()
cv2.destroyAllWindows()