#sudo apt-get install bluetooth libbluetooth-dev
#sudo python3 -m pip install pybluez
from bluetooth import *

global btConnected
global BTadr
global BTsocket
btConnected=False
counter=0
BTadr= "C0:49:EF:F9:F4:F6"
#78:21:84:BB:4D:2E
#B8:D6:1A:AA:26:8E
#40:22:D8:60:B4:3A
#78:21:84:BB:4D:2E
#C0:49:EF:F9:F4:F6

BTsocket=BluetoothSocket( RFCOMM )

def btConnect():
    global btConnected
    global BTsocket
    global BTadr
    try:
        print('connecting...')
        BTsocket.connect((BTadr, 1))
        print('connected')
        #BTsocket.send('{"expression":0}\n')
        BTsocket.send('BT connected\n')
        
        btConnected=True
    except IOError:
        btConnected=False
        
        print("BT error")
    
def isConnected():
    global btConnected
    return btConnected

def sendBTData(data):
    global BTsocket
    global btConnected
    try:
        BTsocket.send(data)
    except IOError:
        BTsocket.close()
        btConnected=False
        print('send error')
        
    
def isData():
    global BTsocket,counter
    global btConnected
    
    try:
        data = BTsocket.recv(1024)
        if data==b'\r\n':
            return 'none'
        else:
            
            return data
    except:
        btConnected=False
        print('BT timeout!')
            
def isData1():
    global BTsocket,counter
    global btConnected
    while True:
        try:
            data = BTsocket.recv(1024)
            if data==b'\r\n':
                return 'none'
            else:
                #print('data:',data)
                return data
        except:
            btConnected=False
            print('BT timeout!')


btConnect()
