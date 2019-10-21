import socketio
import time
import uuid
import sys

class connection_manager:

    connected=False
    authorized=False
    registered=False

    def __init__(self):
        self.sio=None
        self.register_requestId=''
        self.login_requestId=''
        self.newConsumers=None
        self.dirtyConsumers=None

    def checkReadyToWork(self):
        if self.connected and self.authorized and self.registered and (not self.newConsumers is None) and (not self.dirtyConsumers is None):
            return True
        else:
            return False

    def connect(self,url,username,password,robotId):
        self.sio = socketio.Client()
        self.sio.on('connect',self.connect_event)
        self.sio.on('connect_error', self.connect_error_event)
        self.sio.on('disconnect', self.disconnect_event)
        self.sio.on('message', self.message_event)


        self.username=username
        self.password=password
        self.robotId=robotId

        #starts a sequence of callbacks: connect,login,register
        print('start sequence of callbacks')
        self.sio.connect(url,transports=['websocket'])


    def login(self,requestId,login,password):
        self.sio.emit('message',{'requestId': requestId,'event':'users:login','body':{'login':login,'password':password}})
        self.login_requestId=requestId
        return
    def register(self,requestId,robotId):
        self.sio.emit('message',{'requestId':requestId, 'event': 'robots:register' ,'body':{'robotId':robotId}})
        self.register_requestId=requestId
        return

    def connect_event(self):
        self.connected=True
        print('connected')
        self.login(str(uuid.uuid4()),login=self.username,password=self.password)

    def connect_error_event(self):
        print('connection error')
    def disconnect_event(self):
        self.connected = False
        self.authorized=False
        self.registered=False
        
        print('disconnected')
        #exit the node
        sys.exit(0)

    #main handler
    def message_event(self,message):

        #manage with responses
        if 'requestId' in message.keys():

            #login
            if message['requestId']==self.login_requestId:
                if message['status'] == 'success':
                    print('authorized')
                    self.authorized=True
                    self.register(str(uuid.uuid4()),self.robotId)
            #register
            if message['requestId']==self.register_requestId:
                if message['status']=='success':
                    print('registered')
                    self.registered=True

        if 'event' in  message.keys():
            if(message['event']=='robots:consumers:update'):

                self.newConsumers=message['body']['newConsumers']
                self.dirtyConsumers=message['body']['dirtyConsumers']
                print(self.newConsumers)
                print(self.dirtyConsumers)
                return

        print('unknown message: ',message)







