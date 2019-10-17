import time
import socket
import numpy


consumers=[]

def SendImage(buffer):

    for i in range(len(consumers)):
        consumers[i].sendByUdp(buffer)
    return

class Consumer():

    def __init__(self,socketId,host,port):
        self.socketId=socketId
        self.host=host
        self.port=port
        self.chunk_size=1024


        self.socket=None

    def start(self):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        #self.sock.bind(server_address)
        return
    def stop(self):
        self.sock.close()
        return
    def sendByUdp(self,buffer):
        buf = bytes(buffer)
        while len(buf) > 0:
            s = 0
            e = self.chunk_size
            if e > len(buf):
                e = len(buf)
            self.sock.sendto(buf[s:e], (self.host, int(self.port)))
            buf = buf[e:]


def startConsumers(newConsumers):
    global consumers

    if len(newConsumers)!=1:
        raise Exception('len(newConsumers)!=1')

    #clear all consumers
    for i in range(len(consumers)):
        consumers[i].stop()
    consumers=[]

    #add new consumers
    for i in range(len(newConsumers)):
        c=Consumer(newConsumers[i]['socketId'],newConsumers[i]['host'],newConsumers[i]['udpPort'])
        c.start()
        consumers.append(c)

    return
def stopConsumers(dirtyConsumers):
    if len(dirtyConsumers) >0:
        raise Exception('len(dirtyConsumers)>0')

    return

if __name__=='__main__':
    import cv2
    from register import connection_manager
    import jpeg_meta
    import os

    cm = connection_manager()
    cm.connect(os.environ['COORD_SERVER'], username=os.environ['ROBOT_LOGIN'], password=os.environ['ROBOT_PASSWORD'],
               robotId=os.environ['ROBOT_ID'])

    while (True):
        time.sleep(1.0)
        if cm.checkReadyToWork():
            print('ready to work')
            break

    startConsumers(cm.newConsumers)

    cap = cv2.VideoCapture(0)

    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        buf = encimg.reshape((encimg.shape[0]))

        buffer=jpeg_meta.insertMetaData(buf,[])
        SendImage(buffer)




