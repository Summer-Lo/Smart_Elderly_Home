import serial
import serial.tools.list_ports
import json
import os
import os.path
import sys
import pathlib 
import paho.mqtt.client as mqtt
import socket
import threading
import time

import RPi.GPIO as GPIO
def on_connect(mqttc, userdata,flags, rc):
    global bConnected
    print("Connected with result code "+str(rc))
    if rc!=0 :
        mqttc.reconnect()
    else:
        bConnected=True

def on_disconnect(mqttc, userdata, flags,rc):
    global bConnected
    if rc != 0:
        bConnected=False
        print("Unexpected disconnection. Reconnecting...")
        mqttc.reconnect()
    else :
        bConnected=False
        print ("Disconnected successfully" )

def on_message(client, userdata, message):
    print("mqtt message received " ,message.payload.decode("utf-8"))
    print("message topic=",message.topic)
    data=message.payload.decode("utf-8")
    # data=data.replace('"',"'")
    data=json.loads(data)
    
    
    # data={'From': 'nurse', 'To': 'client', 'StationNo': '1', 'Cmd': 50, 'State': 14}
    # data={"From":"mobile","To":"client","StationNo":"999","Cmd":50,"State":14}

    print('jsonData:',data)

ESPresetPin=37
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ESPresetPin, GPIO.OUT)
GPIO.output(ESPresetPin,0)
time.sleep(1)
GPIO.output(ESPresetPin, 1)

c_time_interval=10
time_interval=0
hostName=socket.gethostname()
host=hostName.split('-')
clientID=str(int(host[1])-1)
print('clientID', clientID)


topic_breathState='msg/breathState'


currentPath=pathlib.Path(__file__).parent.resolve()
print('currentPath:',currentPath)
configFile = os.path.join(currentPath, 'config.json')
print('configFile:',configFile)
config = json.loads(open(configFile).read())
print('config:',config['ip'])
print('port:',config['port'])
connstr= f"http://{config['ip']}:{config['port']}"
broker_address=config['ip']; 
print('start mqtt')
mqttClient = mqtt.Client('breath'+clientID)

breathData=""

mqttClient.on_connect = on_connect
mqttClient.on_disconnect = on_disconnect
mqttClient.on_message=on_message #attach function to callback
#mqttClient.on_log=on_log #attach function to callback
mqttClient.connect(broker_address)
mqttClient.loop_start()



def blink() :
    global breathData
    global c_time_interval,time_interval

    ESP_breathState={
       
        "BreathState": 1,
        "BreathMsg": "Movement",
        "BreathBpm": 0,
        "BreathDistance": 0
        
    }

    threading.Timer(1, blink).start()
    time_interval=time_interval+1
    
    if time_interval>c_time_interval:
        time_interval=0
        print('esp breath:',breathData)
        try:
            data=  {"StationNo":"1","Cmd":0,"State":"14"}
            data["StationNo"]=clientID
            ESP_breathState["BreathState"]=breathData["State"]
            ESP_breathState["BreathMsg"]=breathData["Msg"]
            ESP_breathState["BreathBpm"]=breathData["Bpm"]
            ESP_breathState["BreathDistance"]=breathData["Distance"]
            data["State"]=ESP_breathState
            print('breath data:',data)
            mqttClient.publish(topic_breathState,json.dumps(data))#publish
        except:
            pass

blink() 

USB_num = [0]
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
                
ser = serial.Serial('/dev/ttyACM0', 115200)
rl = ReadLine(ser)

        
# ports = serial.tools.list_ports.comports()
# for value in ports:
#     print(value)
blink()  
while True:
    breath = rl.readline()
    breath=breath.decode('UTF-8')
    # print(breath)
    try:
        breathData=json.loads(breath)
        
    except:
        pass
    #print(breathData["Msg"])
    # #x = a.decode('UTF-8').strip().split(':')
#     if a.decode('UTF-8').strip() != 'BEGIN' and a.decode('UTF-8').strip() !='END':
#         print(x)
        