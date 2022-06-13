import socket
import sys
import pynput
from pynput import keyboard
import numpy as np
from numpy.core import uint16, uint8, int8
import math
import struct
import cv2
import depthai as dai
import numpy as np
import time


global WayPoints
global WPi
global calibrate
global lin_vel
global ang_vel
global buffer_size
global TCP_IP
global TCP_PORT
global prev_error
global difftheta
global dumpfile


######################################################[Waypoints]########################################################

#from nazla to left bldg (high-level authority)
waypoints1 = [
[21.49391690638578, 39.24527957049558],
[21.493898188794372, 39.245183010979716],
[21.493885710398764, 39.245078404837514],
[21.493864497123774, 39.24497782200849],
[21.493853266565168, 39.24489601464087],
[21.493842036005688, 39.244815548377645],
[21.493828309765156, 39.24474312874074],
[21.493808344322087, 39.244637181494156],
[21.493767165583883, 39.24457951394437],
[21.49371350842683, 39.244516482038186],
[21.49365485988372, 39.244440039088126],
[21.493581237211007, 39.244383712703865],
[21.493518845086324, 39.244338115154704],
[21.49342775253625, 39.244315316380124],
[21.493397804288154, 39.24430861085819],
[21.493340403456237, 39.24430190533346],
[21.493280506917916, 39.24429385870713],
[21.493231840962387, 39.24428312987204]
]



#from nazla to admission of acceptance 
waypoints2 = [
[21.493894445276563, 39.24518368152737]
,[21.4938857103995, 39.24511662630801]
,[21.49387447984255, 39.24504957108865]
,[21.493864497124513, 39.24497983366052]
,[21.493858257925403, 39.244910096232395]
,[21.49384453168639, 39.24484304101304]
,[21.493829557606016, 39.24477866800246]
,[21.493824566245543, 39.24473038824453]
,[21.493813335683843, 39.2446606508164]
,[21.493794618079125, 39.244593595597046]
,[21.493784635355606, 39.24452385816891]
,[21.493768413426515, 39.24443936851363]
,[21.49375343933828, 39.24436560777234]
,[21.49374345661195, 39.24429989365737]
,[21.493732226044006, 39.244232838438016]
,[21.49372473899822, 39.244160418801115]
,[21.493707269223243, 39.24409470468614]
,[21.49369728649373, 39.2440263083624]
,[21.49368356023955, 39.24394852430795]
,[21.49368855160486, 39.243897562341246]
,[21.493739713089393, 39.243874763566666]
,[21.49378588319415, 39.243860011418406]
]


#from nazla to admision of acceptance
waypoints3= [
[21.493894445276563, 39.24518368152737]
,[21.4938857103995, 39.24511662630801]
,[21.49387447984255, 39.24504957108865]
,[21.493864497124513, 39.24497983366052]
,[21.493858257925403, 39.244910096232395]
,[21.49384453168639, 39.24484304101304]
,[21.493829557606016, 39.24477866800246]
,[21.493824566245543, 39.24473038824453]
,[21.493813335683843, 39.2446606508164]
,[21.493794618079125, 39.244593595597046]
,[21.493784635355606, 39.24452385816891]
,[21.493768413426515, 39.24443936851363]
,[21.49375343933828, 39.24436560777234]
,[21.49374345661195, 39.24429989365737]
,[21.493732226044006, 39.244232838438016]
,[21.49372473899822, 39.244160418801115]
,[21.493707269223243, 39.24409470468614]
,[21.49369728649373, 39.2440263083624]
,[21.49368356023955, 39.24394852430795]
,[21.493557852487612, 39.24392037905878]
,[21.493499874424863, 39.24391235353806]
,[21.493451379711775, 39.243889996531564]
,[21.493467232892584, 39.24378141336612]
,[21.493480627251355, 39.243690455077854]
,[21.493497868800414, 39.24359471595535]
,[21.493512218993523, 39.243497485877896]
,[21.493522734725634, 39.24340172572806]
,[21.49353291184351, 39.24331326659996]
,[21.493554005861807, 39.24317374408495]
,[21.493571413687366, 39.24304778071939]
,[21.49356455055451, 39.24290562364062]
,[21.493494326199784, 39.24295000041186]
,[21.49343817325524, 39.242938601024576]
,[21.493393250884004, 39.24293323660702]
,[21.493345832806558, 39.24292988384408]
,[21.493300910406816, 39.24292116666556]
,[21.493255987993187, 39.24291110838266]
,[21.493219176560583, 39.24290909672608]
,[21.49318610865555, 39.242904402860724]
,[21.493119348896865, 39.242903061754156]
,[21.493055084804038, 39.242892332919055]
,[21.492995188148267, 39.242883615740546]
,[21.492957752721765, 39.24287959242662]
,[21.49291532589809, 39.2428695341437]
,[21.492882881848118, 39.24286416972616]
,[21.492845446396856, 39.242859475860804]
,[21.492803019540435, 39.2428500881301]
,[21.492759968747027, 39.24284606481693]
,[21.49271317438708, 39.24284204149636]
,[21.49269695234046, 39.24282192493055]
,[21.492684473841823, 39.242808513886686]
,[21.49267324319214, 39.242794432290616]
,[21.492663884316737, 39.242781691798946]
,[21.492655149365834, 39.242756210815585]
,[21.492650157965066, 39.24270122553572]
,[21.492657645066142, 39.242642887494874]
,[21.4926632603917, 39.24258656111062]
,[21.492665132166834, 39.24254431632242]
,[21.49267823459311, 39.24247927273414]
,[21.492675738893162, 39.242406853097236]
]




#=================================================[TCP link intialization]================================================
"""
TCP_IP: the ip of the TCP server
TCP_PORT: the port for the TCP socket
buffer_size: the length of the recieving buffer size.
"""
TCP_IP = "192.168.1.193"
TCP_PORT = 25000
buffer_size=12;
prev_error=float('inf')

"""
WayPoints: the waypoints the robot follows in orderly manner
WPi: the number of way points
lin_vel: intial linear velcity
ang_vel: intial angular velocity
calibrate: when True the robot is in calibration mode and controlled only by arrows keys, when False it follows the waypoints.
Vmax: maximum linear velocity the robot can move by.
"""
WPi = 0;
lin_vel = 0;
ang_vel = 0;  
calibrate = True;
Vmax=0.8;
difftheta = 0
dumpfile = False

def wait_to_unlock():
     while 1:
        ESP_IP = "192.168.1.4"
        ESP_PORT = 8000
        s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s2.connect((ESP_IP, ESP_PORT))
        s2.settimeout(100)
        print("[Connected]: connected to the LOCK ESP8266.")

        HOST = '192.168.1.109'   #IP of the current device running the python code
        PORT = 8000
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


        #Bind socket to Host and Port
        try:
            s.bind((HOST, PORT))
        except socket.error as err:
            print ('[ERROR]: Bind Failed, Error Code: ' + str(err[0]) + ', Message: ' + err[1])
            sys.exit()
            
        
        #listen(): This method sets up and start TCP listener.
        s.listen(10)

        conn, addr = s.accept()
        print ('[connected]: Connected with ' + addr[0] + ':' + str(addr[1]))
        for i in range(1000):
            pass
        buf = conn.recv(8).decode("utf-8")
        print("unlocked")
        s2.send(bytes("3",'utf-8'))
        time.sleep(3)

        break



def main():
    ####################################################[TCP-link with the mobile app]#####################################
    
    print ('[intitializing]: the socket is being initilized...')
    HOST = '192.168.1.109'   #IP of the current device running the python code
    PORT = 8000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    #Bind socket to Host and Port
    try:
        s.bind((HOST, PORT))
    except socket.error as err:
        print ('[ERROR]: Bind Failed, Error Code: ' + str(err[0]) + ', Message: ' + err[1])
        sys.exit()
        
    
    #listen(): This method sets up and start TCP listener.
    s.listen(10)
    print ('[Running]: Socket is now listening')
    global WayPoints
    WayPoints=[[]]

    while 1:
        conn, addr = s.accept()
        print ('[connected]: Connected with ' + addr[0] + ':' + str(addr[1]))
        for i in range(1000):
            pass
        buf = conn.recv(8).decode("utf-8")
        break



    buf = int(buf)
    print(buf)
    ############################################[connect to the 2nd ESP to lock]#################################################
    ESP_IP = "192.168.1.4"
    ESP_PORT = 8000
    # Establish connection
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s2.connect((ESP_IP, ESP_PORT))
    s2.settimeout(100)
    print("[Connected]: connected to the LOCK ESP8266.")



    if (buf == 0):
        WayPoints=waypoints1
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == 1):
        WayPoints=waypoints2
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == 2):
        WayPoints=waypoints3
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == -1):
        print("Maintenance mode is ON")
        s2.send(bytes("4",'utf-8'))
        time.sleep(3)


        while 1:
            s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s2.connect((ESP_IP, ESP_PORT))
            s2.settimeout(100)
            print("[Connected]: connected to the LOCK ESP8266.")

            conn, addr = s.accept()
            print ('[connected]: Connected with ' + addr[0] + ':' + str(addr[1]))
            for i in range(1000):
                pass
            buf = conn.recv(8).decode("utf-8")
            print("Maintenance mode is OFF")
            s2.send(bytes("2",'utf-8'))
            time.sleep(3)

            break

    
    s2.close()
    s.close()
    # wait_to_unlock()



    ####################################################[init the TCP connection]##########################################
    print("[INFO] Program Starting!")
    global lin_vel;
    global ang_vel;

    print("[INFO] Initializing TCP connection ...")
    s = TCP_init()
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()  # start to listen on a separate thread

    ####################################################[init the Oak-D camera]##########################################

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    # monoLeft = pipeline.cr
    monoRight = getMonoCamera(pipeline, isLeft = False)

    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)

    
    # Set XlinkOut for disparity, rectifiedLeft, and rectifiedRight
    xoutDisp = pipeline.createXLinkOut()
    xoutDisp.setStreamName("disparity")
    
    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")

    stereo.disparity.link(xoutDisp.input)
    
    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)
    
    with dai.Device(pipeline) as device:

        
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        disparityQueue = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
        rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
        rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)


        # Calculate a multiplier for colormapping disparity map
        disparityMultiplier = 255 / stereo.initialConfig.getMaxDisparity()

        ###################################################[the main loop]################################################
        while True:
            t0= time.time()

            disparity = getFrame(disparityQueue)
            disparity = (disparity * disparityMultiplier).astype(np.uint8)
            thresh=disparity[30:350,:]
            avg1=thresh[:,0:150].mean()
            avg2=thresh[:,100:530].mean()
            avg3=thresh[:,480:639].mean()
 
            if (not calibrate):

                lin_vel = Vmax*np.exp(-2*abs(difftheta))
                ang_vel = PIDcontroller(difftheta);
                
                if(avg2<20):                #middle way is EMPTY
                   
                    print("Go Forward")

                    
                elif(avg1<25):              #left way is EMPTY
                    print("Go Left")
                    ang_vel = 0.07+0.002*(avg2+avg3);

                    
                elif(avg3<25):              #right way is EMPTY
                    print("Go Right")
                    ang_vel =-0.07-0.002*(avg1+avg2);

                    
                else:                       #No-way to move
                    print('Stop')
                    lin_vel = 0
                    ang_vel = 0

           
            """TODO
            [#] put the avg calc here before the recieve
            [] use the avg for changing the ang velocity
            
            """

            send(s,ang_vel,-lin_vel)

            t1=time.time()
            td=t1-t0
            if (td<0.001):
                time.sleep(0.001-td)


            receive(s)
                

        listener.join()  # remove if main thread is polling self.keys





def TCP_init():
    global TCP_IP
    global TCP_PORT    
    # Establish connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.settimeout(20)
    print("[Connected]: successfully connected to TCP server.")
    return s




def send(s,steer,speed):
    # Send Commands
    scale = -115.68;
    speed = scale*speed;
    steer = scale*steer;
    
    if(speed<0):
        speed = speed + 65536;
    if(steer<0):
        steer = steer + 65536;
    
    start = uint16(43981)
    steerp = uint16(steer)
    speedp = uint16(speed)

    chkSum = (start ^ steerp) ^ speedp
    chkSum = uint16(chkSum)
    write_buffer = np.array([start, steerp,speedp, chkSum],dtype=uint16)
    write_buffer = write_buffer.tobytes()
    s.send(write_buffer)  





def receive(s):
    global WayPoints;
    global WPi;
    global lin_vel;
    global ang_vel;
    global buffer_size;
    global difftheta;
    
    feedback = s.recv(buffer_size)
    if len(feedback) >= buffer_size:
        Yaw = np.float32(struct.unpack('f',feedback[0:4]))
        PosX = np.float32(struct.unpack('f',feedback[4:8]))
        PosY = np.float32(struct.unpack('f',feedback[8:12]))
        
        R,setYaw = getBearingDistance(PosY, PosX,WayPoints[WPi][0], WayPoints[WPi][1], 1)   

    
        difftheta = wrapToPi(setYaw-Yaw)
        if (not calibrate):
            lin_vel = Vmax*np.exp(-2*abs(difftheta));

            if(R<1.5):
                WPi = WPi + 1;
                if(WPi>=len(WayPoints)):
                    wait_to_unlock()
                   
            
        print(f"WP{WPi+1}  Yaw = {Yaw*180/np.pi}, PosY = {PosY}, PosX ={PosX}, difftheta = {difftheta*180/np.pi}, R = {R}\n");
        # print(f"Yaw= {Yaw*180/np.pi}, Set Yaw = {setYaw*180/np.pi}")






def PIDcontroller(error):
    global prev_error;
    if(prev_error==float('inf')):
        prev_error = error;
    
    Kp = 0.25;
    Ki = 0;
    Kd = 0.06;


    Derror = error - prev_error;
    output = error*Kp + Derror*Kd;
    
    prev_error = error;
    
    return output






def getBearingDistance(a1,a2,b1,b2,mode):
   
    if(mode==0):             #indoors (co-ordinates in meters)  
        theta = -(math.atan2(b1-a1,b2-a2)-np.pi/2);
        theta = wrapToPi(theta);
        R = math.sqrt((b1-a1)**2 + (b2-a2)**2); 
    else:                       
        lat1 = a1* np.pi/180;
        lat2 = b1* np.pi/180;
        lon1 = a2* np.pi/180;
        lon2 = b2* np.pi/180;
        dlat = (lat2-lat1);
        dlon = (lon2-lon1);


        temp = math.sin(dlat/2)*math.sin(dlat/2) + math.sin(dlon/2)*math.sin(dlon/2)*math.cos(lat1)*math.cos(lat2);
        R = 2*math.atan2(math.sqrt(temp),math.sqrt(1-temp))*6371e3;
        theta = wrapToPi(math.atan2(math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dlon),math.sin(dlon)*math.cos(lat2)));
        return R,theta
  
    
  
    
def wrapToPi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi;    
    
 
    
 
    
def on_press(key):
    global lin_vel
    global ang_vel
    global calibrate
    if key == keyboard.Key.up:
        lin_vel = lin_vel+0.1
    elif key == keyboard.Key.down:
        lin_vel = lin_vel-0.1
    elif key == keyboard.Key.left:
        ang_vel = ang_vel+0.05
    elif key == keyboard.Key.right:
        ang_vel = ang_vel-0.05
    elif key == keyboard.Key.space:  
        lin_vel = 0
        ang_vel = 0  
    elif key == keyboard.Key.tab:
        print('mode switch')
        calibrate= not calibrate
  

def getFrame(queue):
  # Get frame from queue
  frame = queue.get()
  # Convert frame to OpenCV format and return
  return frame.getCvFrame()


def getMonoCamera(pipeline, isLeft):
  # Configure mono camera
  mono = pipeline.createMonoCamera()

  # Set Camera Resolution
  mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  mono.setFps(fps=15)#set to 10 to reduce computation and in turn reduce lag
  
  if isLeft:
      # Get left camera
      mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
  else :
      # Get right camera
      mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
  return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    # Configure stereo pair for depth estimation
    stereo = pipeline.createStereoDepth()
    # Checks occluded pixels and marks them as invalid
    stereo.setLeftRightCheck(True)
    
    # Configure left and right cameras to work as a stereo pair
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    return stereo

def mouseCallback(event,x,y,flags,param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX = x
        mouseY = y






#################################################[Maintennce mode with Report]##############################################

def mainWithFile():
    # Opening a file
    file1 = open('logfile.txt', 'w')

    
    # Writing a string to file
    file1.write(f"starting the program at {time.time()}")
    
    # Closing file
    file1.close()




     ####################################################[TCP-link with the mobile app]#####################################
    
    file1.write('[intitializing]: the socket is being initilized...')
    
    HOST = '192.168.1.109'   #IP of the current device running the python code
    PORT = 8000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    #Bind socket to Host and Port
    try:
        s.bind((HOST, PORT))
    except socket.error as err:
        print ('[ERROR]: Bind Failed, Error Code: ' + str(err[0]) + ', Message: ' + err[1])
        sys.exit()
        
    
    #listen(): This method sets up and start TCP listener.
    s.listen(10)
    file1.write('[Running]: Socket is now listening')
    global WayPoints
    WayPoints=[[]]

    while 1:
        conn, addr = s.accept()
        print ('[connected]: Connected with ' + addr[0] + ':' + str(addr[1]))
        for i in range(1000):
            pass
        buf = conn.recv(8).decode("utf-8")
        break



    buf = int(buf)
    file1.write(buf)
    ############################################[connect to the 2nd ESP to lock]#################################################
    ESP_IP = "192.168.1.4"
    ESP_PORT = 8000
    # Establish connection
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s2.connect((ESP_IP, ESP_PORT))
    s2.settimeout(100)
    file1.write("[Connected]: connected to the LOCK ESP8266.")



    if (buf == 0):
        WayPoints=waypoints1
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == 1):
        WayPoints=waypoints2
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == 2):
        WayPoints=waypoints3
        s2.send(bytes("1",'utf-8'))
        time.sleep(3)
        s2.close()

    elif(buf == -1):
        file1.write("Maintenance mode is ON")
        s2.send(bytes("4",'utf-8'))
        time.sleep(3)


        while 1:
            s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s2.connect((ESP_IP, ESP_PORT))
            s2.settimeout(100)
            file1.write("[Connected]: connected to the LOCK ESP8266.")

            conn, addr = s.accept()
            file1.write('[connected]: Connected with ' + addr[0] + ':' + str(addr[1]))
            for i in range(1000):
                pass
            buf = conn.recv(8).decode("utf-8")
            file1.write("Maintenance mode is OFF")
            s2.send(bytes("2",'utf-8'))
            time.sleep(3)

            break

    
    s2.close()
    s.close()
    # wait_to_unlock()



    ####################################################[init the TCP connection]##########################################
    file1.write("[INFO] Program Starting!")
    global lin_vel;
    global ang_vel;

    file1.write("[INFO] Initializing TCP connection ...")
    s = TCP_init()
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()  # start to listen on a separate thread

    ####################################################[init the Oak-D camera]##########################################

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    # monoLeft = pipeline.cr
    monoRight = getMonoCamera(pipeline, isLeft = False)

    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)

    
    # Set XlinkOut for disparity, rectifiedLeft, and rectifiedRight
    xoutDisp = pipeline.createXLinkOut()
    xoutDisp.setStreamName("disparity")
    
    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")

    stereo.disparity.link(xoutDisp.input)
    
    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)
    
    with dai.Device(pipeline) as device:

        
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        disparityQueue = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
        rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
        rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)


        # Calculate a multiplier for colormapping disparity map
        disparityMultiplier = 255 / stereo.initialConfig.getMaxDisparity()

        ###################################################[the main loop]################################################
        while True:
            t0= time.time()

            disparity = getFrame(disparityQueue)
            disparity = (disparity * disparityMultiplier).astype(np.uint8)
            thresh=disparity[0:250,:]
            avg1=thresh[:,0:150].mean()
            avg2=thresh[:,100:530].mean()
            avg3=thresh[:,480:639].mean()
 
            if (not calibrate):

                lin_vel = Vmax*np.exp(-2*abs(difftheta))
                
                if(avg2<20):                #middle way is EMPTY
                    ang_vel = PIDcontroller(difftheta);
                    file1.write("Go Forward")

                    
                elif(avg1<25):              #left way is EMPTY
                    file1.write("Go Left")
                    ang_vel = 0.065+0.0015*(avg2+avg3);

                    
                elif(avg3<25):              #right way is EMPTY
                    file1.write("Go Right")
                    ang_vel =-0.065-0.0015*(avg1+avg2);

                    
                else:                       #No-way to move
                    file1.write('Stop')
                    lin_vel = 0
                    ang_vel = 0



            send(s,ang_vel,-lin_vel)

            t1=time.time()
            td=t1-t0
            if (td<0.001):
                time.sleep(0.001-td)


            receive(s)
                

        listener.join()  # remove if main thread is polling self.keys




"""TODO:
    [] replace the IP (here) and (ON andriod) to 192.168.1.3 (jetson's IP)
    """
 



if __name__ == '__main__':
    if(not dumpfile):
        main()
    else:
        mainWithFile()