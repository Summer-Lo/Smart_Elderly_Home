#1023may28
import os
import os.path
import sys
import pathlib 
import argparse
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import paho.mqtt.client as mqtt
from numba import jit
import cv2
import RPi.GPIO as GPIO
import time
from timeloop import Timeloop
from datetime import timedelta
from datetime import datetime
from timeit import default_timer as timer
from imutils import perspective
from imutils import contours
#from scipy.spatial import distance as dist
import imutils
import math
import pointClass as Point
from lib.Util import post_process_depth_frame, colored_map
import tkinter as tk
import joblib
import shutil
import random
# import PIL
# from PIL import Image, ImageTk
import socketio
import socket
import threading
import json,yaml
#import pandas as pd
import base64
import constant as cn
import detector
import schedule as sch

import removeDirectory as rm

bWithinTime,bMaualOn,bManualOff=False,False,False
directoryLogFile = './directoryLog.txt'
directory_array = []
# from tkinter import *
# from tkinter import filedialog

row=60
col=80
ww=320
hh=240

cx,cy=30,40
a=0.0111
b=-306.53
c_scale=4
font_scale = 0.6
font = cv2.FONT_HERSHEY_SIMPLEX
color = (255, 255, 255)  #BGR,blue
thickness = 2



prcoessed_depth_colormap=None
depth_colormap=None
ledRunningPin=31
ledMonitorPin=33
btnPin=35
c_class_lying=0
c_class_sit=1
c_class_offbed=2
className=['lying','sit','offbed']
bLyingPredicted=False
bSitPredicted=False
tx1,ty1,tx2,ty2=71,72,110,300
tcx,tcy=0,0
themp_x1,themp_y1=0,0
rx1,ry1,rx2,ry2=10,10,20,20
bMouseDown=False
bBreathSenorError=False
currentPath=pathlib.Path(__file__).parent.resolve()
print('currentPath:',currentPath)
imageDir = os.path.join(currentPath, 'images')
print('imageDir:',imageDir)

paraFile = os.path.join(currentPath, 'bedExitPara.json')
print('paraFile:',paraFile)
systemConfigFile = os.path.join(currentPath, 'systemConfig.json')
print('systemConfigFile:',systemConfigFile)
configFile = os.path.join(currentPath, 'config.json')
print('configFile:',configFile)

if not os.path.isfile(directoryLogFile):
    file1 = open(directoryLogFile, "w")
    file1.close()
else:
    directory_array = []
    file1 = open(directoryLogFile, 'r')
    Lines = file1.readlines()
    # Strips the newline character
    for line in Lines:
        directory_array.append(line.strip())
    print('directory:',directory_array)
    print('dirctory len:',len(directory_array))
  
   
     
        

  
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ledRunningPin, GPIO.OUT)
GPIO.setup(ledMonitorPin, GPIO.OUT)


bAlert,bNeedAlarm=False,False
bSave=False


predictArray=0,0,(0,0),(0,0),(0,0,0)

stateMsg=['none','sleep','sit on bed','off bed','leave','fall','abnormal breath','others','waiting...','no motion',
               'lying','both leave','bed only','start monitor','stop monitor','login','logout','power on','power off','kinect error',
               'breath error','sleep locked','lost connection','not installed','power on fail','power off fail','stand on bed','ask token','backing up','backup done',
               'waiting backing','start disk low','stop disk low','sit bed head','sit bed end','','','','','not activated',
               'power off','','','','','','','','','',
                'unknown','no body','siy','lean','stand','leave','potential fall','fall','sensor error','lost connection'];

bedName=['bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
'bed 101','bed 102','bed 103','bed 104','bed 105',
]
breathMsgArray=['bpm','movement','tracking','no movement','initializing']
breathMsg=''
breathBpmArray=[0,0,0]

c_clientTimeout=30
clientTimeout=0
c_monitorDelay=5

lyingCount=0

monitorDelay=c_monitorDelay
bServerConnected=True
bMqttConnected=False
bSleepLocked=False
bOffbedLocked=False
bLocked='none'
bRemoveBedsideOnly=False
bBreathLocked=False
prevPointBedsideOffset=0
prevState=cn.c_station_state_monitor_stop
w, h = 640, 480

bedImageArray = [[0]*w]*h
scale=2
paras_level=0,0
keepalive_interval=0
xyArray = 0,0,0,0
bed_width=0
lyingPoint=[0]*10
lyingPointIdx=0
x1,y1,x2,y2=0,0,0,0
c_sensorAngle=0
c_sendImageInterval=0
c_motion_th,c_no_motion_count=0,0 
c_bed_motion_detect_count=5
bed_motion_detect_count=0
c_floor_level,c_bed_level=0,0
c_yLeave=0
c_bed_head_line=0
c_min_bedside_point,c_min_lying_point,c_min_sit_point=0,0,0
c_pointAboveSit,c_pointAboveSitHead,c_pointAboveSitEnd, c_pointLying,c_pointBedsideRight=0,0,0,0,0
c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow,c_pointBedside=0,0,0,0
c_pointBedsideLow=0
c_thermal_adjust_count=0
c_thermal_change=0
c_min_thermal_th=0
c_lyingCount,c_predict_lying_count,c_breath_count,c_breath_distance=0,0,0,0
c_total_day_to_keep=5
c_keepalive_interval=60
c_clipping_distance_in_meters ,c_threshold_distance_in_meters=0,0

c_lying_level,c_sit_level=0,0
c_use_breath_detect=1
c_use_thermal_detect=0
c_bSaveDepth=0
c_bSaveImage=0
c_save_interval=0
save_interval=0
c_bRotateImage=0

c_depth_offset=0,0,0,0

MARGIN=False,False,False,False,0,0,0,0,'0','0'
c_detect_side='both'

c_other_bedend_offset=100
c_FOV_H,c_FOV_V,c_FOV_D=85,58,94
no_motion_count=0

c_save_mode=cn.c_MOTION_SAVE
c_save_image_type=cn.c_SAVE_COLOR
weightModel='depth.tflite'
conf_thres=0.7
deltaAngleX=0
hostName=socket.gethostname()
host=hostName.split('-')
clientID=str(int(host[1])-1)
print('clientID', clientID)
clientName=bedName[int(clientID)]
#sio.emit('clientPowerState message', {'hostName':hostName,'powerState':0})
bKeepAlive=False

powerSchedule=None
detectOptions=None
alertType=None
detectZone=None
detectMargin=None

reStartAfterStopDelay=1

thresholdParas=None
bBlink=False

c_alarmInterval=10
alarmIntervalCount=0

config = json.loads(open(configFile).read())
print('config:',config['ip'])
print('port:',config['port'])
connstr= "http://{config['ip']}:{config['port']}"
#connstr= f"http://localhost:{config['port']}"


sio = socketio.Client()

clientTimeout=0


broker_address=config['ip']; 
topic_mobile_command="msg/mobileCommand"
topic_server_keepalive='device/server'
topic_clientState='msg/clientState'
topic_breathState='msg/breathState'
topic_client_command='msg/clientCommand'
topic_command='msg/command'
topic_temperature='msg/temperature'
topic_clientImage='msg/clientImage'
topic_ImageToMobile="msg/imageToMobile"
topic_region='msg/region'
topic_set_schedule="msg/setSchedule"	
topic_read_schedule="msg/readSchedule"
topic_read_option="msg/readOption"
topic_set_option="msg/setOption"
topic_read_alert="msg/readAlert"
topic_set_alert="msg/setAlert"
topic_read_zone="msg/readZone"
topic_set_zone="msg/setZone"
topic_read_margin="msg/readMargin"
topic_set_margin="msg/setMargin"
topic_read_para="msg/readPara"
topic_set_para="msg/setPara"
topic_askState="msg/askState"
print('start mqtt')
mqttClient = mqtt.Client(clientID)


def logDirectory(data,c_total_day_to_keep,directory_array,directoryLogFile):
    if len(directory_array)<c_total_day_to_keep:
        file1 = open(directoryLogFile, "a")  # append mode
        file1.write(data)
        file1.write("\n")
        file1.close()
    else:
        deleteDirectory(imageDir,directory_array[0])
        arr = np.array(directory_array)
        directory_array = np.roll(arr, -1)
        directory_array[c_total_day_to_keep-1]=data
        print('directory_array:',directory_array)
        with open(directoryLogFile, 'w') as fp:
            for item in directory_array:
                # write each item on a new line
                fp.write("%s\n" % item)

                
        

        
        
        
def createSaveDir(imageDir):
    global savePath,current_date,current_time
    current_date=datetime.now().strftime('%Y-%m-%d')
    current_time=datetime.now().strftime('%H-%M')
    print('current time:',current_date)
    print('current time:',current_time)
    savePath=imageDir+"/"+current_date+"/"
    
    if not os.path.exists(savePath):
        print('create image directory')
        os.makedirs(savePath)
        logDirectory(current_date,c_total_day_to_keep,directory_array,directoryLogFile)

        
def deleteDirectory(dirPath,dirName):
    print('delete:',dirPath+'/'+dirName)
    # Delete all contents of a directory using shutil.rmtree() and  handle exceptions
    try:
        shutil.rmtree(dirPath+"/"+dirName)
    except:
        print('Error while deleting directory')
        
   


def on_connect(mqttc, userdata,flags, rc):
    global bMqttConnected
    print("Connected with result code "+str(rc))
    if rc!=0 :
        mqttc.reconnect()
    else:
        bMqttConnected=True

def on_disconnect(mqttc, userdata, flags,rc):
    global bMqttConnected
    if rc != 0:
        bMqttConnected=False
        print("Unexpected disconnection. Reconnecting...")
        mqttc.reconnect()
    else :
        bMqttConnected=False
        print ("Disconnected successfully" )
maxT1,maxT2=0,0

def emitClientState():
    global bMqttConnected
    global current_date,current_time
    global clientID,clientName,clientState
    global bKeepAlive,bAlert,bNeedAlarm ,bSleepLocked
    current_date=datetime.now().strftime('%Y-%m-%d')
    current_time=datetime.now().strftime('%H:%M:%S')
    msg={'From':'client',
                                                'Date':current_date,
                                                'Time':current_time,
                                                'StationNo':clientID,
                                                'Name':clientName,
                                                'State':clientState,
                                                'BreathBpm':'0',
                                                'BreathState':'0',
                                                'BreathQty':'0',
                                                'bKeepAlive':bKeepAlive,
                                                'bAlert':bAlert ,
                                                'bSleepLocked':bSleepLocked,
                                                'bHardware':False,
                                                'bNeedAlarm':bNeedAlarm

                                                }
    if bMqttConnected:
        print('publish clientstate:',msg)
        
        mqttClient.publish(topic_clientState,json.dumps(msg))#publish
    else:
        print('mqtt disconnected!')   
markedImage=None
def sendImage(image):
    global clientID,clientName,c_bed_level
    encoded_string=depth2base64(image)
    current_date=datetime.now().strftime('%Y-%m-%d')
    current_time=datetime.now().strftime('%H:%M:%S')
    msg={'From':'client',
                                                'Date':current_date,
                                                'Time':current_time,
                                                'StationNo':clientID,
                                                'Name':clientName,
                                                'State':clientState,
                                                'Img':encoded_string, 
                                                'BedDistance':c_bed_level

                                                }
    if bMqttConnected:
        print('publish clientImage:')
        
        mqttClient.publish(topic_clientImage,json.dumps(msg))#publish
    else:
        print('mqtt disconnected!')
   
def sendImageToMobile(image):
    global clientID,clientName,c_bed_level
    encoded_string=depth2base64(image)
    current_date=datetime.now().strftime('%Y-%m-%d')
    current_time=datetime.now().strftime('%H:%M:%S')
    msg={'From':'client',
                                                'Date':current_date,
                                                'Time':current_time,
                                                'StationNo':clientID,
                                                'Name':clientName,
                                                'State':clientState,
                                                'Img':encoded_string, 
                                                'BedDistance':c_bed_level

                                                }
    if bMqttConnected:
        print('publish image to mobile:')
        
        mqttClient.publish(topic_ImageToMobile,json.dumps(msg))#publish
    else:
        print('mqtt disconnected!')
        
def setStartMonitor():
    global bMqttConnected,bMonitor
    global current_date,current_time
    global clientID,clientName,clientState,prevState
    global bKeepAlive,bAlert,bNeedAlarm ,bSleepLocked
    global maxT1,maxT2
    global prcoessed_depth_colormap
    global clientTimeout,monitorDelay
    global bLyingPredicted,bSitPredicted
    global bOffbedLocked,bSitLocked,bLyingLocked
    global counter,bGetPointOffset,lyingCount
    global breath_count,bBreathLocked
    global depth_baseline,bedImageArray
    global bPassby
    global bed_motion_detect_count
    clientTimeout=0
    loadSystemConfig()
    bed_motion_detect_count=0
    bPassby=False
    bMonitor=True
    bLyingPredicted=False
    bSitPredicted=False
    monitorDelay=0
    bAlert=False
    bOffbedLocked=False
    bSitLocked=False
    bLyingLocked=False
    bSleepLocked=False

    bGetPointOffset=True
    counter=0

    clientState=cn.c_station_state_monitor_start
    prevState=clientState
    predictedState='none'
    lyingCount=0
    breath_count=0
    bBreathLocked=False
    depth_baseline=None
    bedImageArray = [[0]*w]*h
    print('start command',clientState)
    emitClientState()
    rm.delDir()
   

def setStopMonitor():
    global bMqttConnected,bMonitor
    global current_date,current_time
    global clientID,clientName,clientState,prevState
    global bKeepAlive,bAlert ,bNeedAlarm,bSleepLocked
    global maxT1,maxT2
    global prcoessed_depth_colormap
    global clientTimeout,monitorDelay
    global bLyingPredicted,bSitPredicted
    global bOffbedLocked,bSitLocked,bLyingLocked
    global counter,bGetPointOffset,lyingCount
    global breath_count,bBreathLocked
    global bPassby
    clientTimeout=0
    bPassby=False
    bMonitor=False
    bAlert,bNeedAlarm=False,False
    
    bLyingLocked=False
    bSitLocked=False
    bSleepLocked=False
    if bManualOff:
        clientState=cn.c_station_state_power_off
    else:
        clientState=cn.c_station_state_monitor_stop
    prevState=clientState

    print('stop command',clientState)
    emitClientState()

#{'BreathCounter': 11, 'StationNo': 1, 'BreathState': 0, 'BreathBpm': 18.0, 'BreathDistance': 1.4}

#{'StationNo': '1', 'Cmd': 0, 'State': {'State': 2, 'Msg': 'tracking', 'Bpm': '0', 'Distance': '0'}}
def determineBreath(data):
    global detectOptions
    global clientTimeout,bMqttConnected,clientID
    global breathMsg,breathMsgArray,breath_count
    global breathStatus,bBreathLocked
    global c_breath_distance
    global no_motion_count
    global bBreathSenorError
   
    sno=data["StationNo"]
    print('sno:',sno)

    data=data["State"]
    
    if data!='':
        if str(sno)==clientID:
            print('realsense client breath:',data)
            
            if data["BreathState"]==cn.c_breath_state_breathrate :
                breathMsg=str(data["BreathBpm"])+"->"+str(round(data["BreathDistance"],1))
                    
                if data["BreathDistance"]<=c_breath_distance:
                    if int(data["BreathBpm"])>0:
                        breath_count=breath_count+1
                        no_motion_count=0
                        
            else:
                if data["BreathState"]==cn.c_breath_state_no_movement:
                    bBreathLocked=False
                    
                breath_count=0
                print('breathMsg:',breathMsg)
                breathMsg=breathMsgArray[int(data["BreathState"])]
            if detectOptions["UseBreathSensorDetectPresence"]:
                print('use breath senseor to detect')
                if  breath_count >c_breath_count:
                    bBreathLocked=True
                    bLocked='breath locked'
                    print('breath locked')
            bBreathSenorError=False
    else:
        bBreathSenorError=True
        print('breath error............')       

#####################################
# {"From":"nurse","To":"client","StationNo":"2","isEnable":true,"OnTime":"21:00","OffTime":"09:00","Mon":false,"Tue":true,"Wed":true,"Thu":false,"Fri":false,"Sat":false,"Sun":false}

def readSchedule():
    global powerSchedule,bManualOff,bMaualOn
    powerSchedule=sch.readSchedule()
    if powerSchedule!=None:
        
        print('powerSchedule:',powerSchedule)
        bManualOff,bMaualOn=False,False
        mqttClient.publish(topic_read_schedule,json.dumps(powerSchedule))
    else:
        print('schedule file not found')


def setSchedule(data):
    print('set schedule:',data)
    with open("powerSchedule.json", "w") as outfile:
        json.dump(data, outfile)
    readSchedule()
##################################

def readOption():
    global detectOptions
    if os.path.isfile('detectOptions.json'):
        with open('detectOptions.json', 'r') as openfile:

            detectOptions = json.load(openfile)
        
        print('options:',detectOptions)

        mqttClient.publish(topic_read_option,json.dumps(detectOptions))
    else:
        print('detectOptions file not found')


def setOption(data):
    print('set option:',data)
    with open("detectOptions.json", "w") as outfile:
        json.dump(data, outfile)
    readOption()
########################################


def readAlert():
    global alertType
    if os.path.isfile('alertType.json'):
        with open('alertType.json', 'r') as openfile:

            alertType = json.load(openfile)
        
        print('alertType:',alertType)

        mqttClient.publish(topic_read_alert,json.dumps(alertType))
    else:
        print('alertType file not found')


def setAlert(data):
    print('set alert:',data)
    with open("alertType.json", "w") as outfile:
        json.dump(data, outfile)
    readAlert()
#########################################3
def readZone():
    global detectZone,bed_width
    global xyArray,x1,y1,x2,y2,cx,cy,c_yLeave,bGetPointOffset,counter
    if os.path.isfile('detectZone.json'):
        with open('detectZone.json', 'r') as openfile:

            detectZone = json.load(openfile)
        
        print('detectZone:',detectZone)
       
        x1=detectZone['x1']
        y1=detectZone['y1']
        x2=detectZone['x2']
        y2=detectZone['y2']
        # if y2>479-c_yLeave:
        #     y2=479-c_yLeave
        xyArray=x1,y1,x2,y2
        bed_width=y2-y1
        bGetPointOffset=True
        counter=0
        mqttClient.publish(topic_read_zone,json.dumps(detectZone))
    else:
        print('detectZone file not found')


def setZone(data):
    print('set zone:',data)
    with open("detectZone.json", "w") as outfile:
        json.dump(data, outfile)
    readZone()
###################################
def readMargin():
    global detectMargin,MARGIN
    if os.path.isfile('detectMargin.json'):
        with open('detectMargin.json', 'r') as openfile:

            detectMargin = json.load(openfile)
        MARGIN=detectMargin["ChkFront"],detectMargin["ChkBack"],detectMargin["ChkLeftSide"],detectMargin["ChkRightSide"],int(detectMargin["FrontMargin"]),int(detectMargin["BackMargin"]),int(detectMargin["LeftMargin"]),int(detectMargin["RightMargin"]),detectMargin["BedHeadPos"],detectMargin["DetectBedside"]
        
        print('detectMargin:',detectMargin)
        mqttClient.publish(topic_read_margin,json.dumps(detectMargin))
    else:
        print('detectMargin file not found')


def setMargin(data):
    print('set margin:',data)
    with open("detectMargin.json", "w") as outfile:
        json.dump(data, outfile)
    readMargin()
###################################

def readPara():
    global thresholdParas
    if os.path.isfile('thresholdParas.json'):
        with open('thresholdParas.json', 'r') as openfile:

            thresholdParas = json.load(openfile)
        
        print('thresholdParas:',thresholdParas)
        mqttClient.publish(topic_read_para,json.dumps(thresholdParas))
    else:
        print('thresholdParas file not found')


def setPara(data):
    print('set paras:',data)
    with open("thresholdParas.json", "w") as outfile:
        json.dump(data, outfile)
    readPara()
###################################
# mqtt message received  {"From":"nurse","To":"client","StationNo":"1","Cmd":50,"State":14}
# message topic= msg/command
# jsonData: {'From': 'nurse', 'To': 'client', 'StationNo': '1', 'Cmd': 50, 'State': 14}

#from mobile
#"{\"From\":\"mobile\",\"To\":\"client\",\"StationNo\":1,\"Cmd\":49,\"State\":13}"
#jsonData: {"From":"mobile","To":"client","StationNo":1,"Cmd":49,"State":13}
#mqtt message received  {'To': '2', 'Cmd': 'powerOn'}

def on_message(client, userdata, message):
    global bMqttConnected,bMonitor,bServerConnected
    global current_date,current_time
    global clientID,clientName,clientState,prevState
    global bKeepAlive,bAlert,bNeedAlarm ,bSleepLocked
    global maxT1,maxT2
    global prcoessed_depth_colormap
    global xyArray,x1,y1,x2,y2,cx,cy,c_yLeave,bGetPointOffset,counter
    global bManualOff,bMaualOn,depth_baseline
    global bPassby
    
   # print("mqtt message received " ,message.payload.decode("utf-8"))
    bServerConnected=True
    print("message topic=",message.topic)
    if  message.topic==topic_server_keepalive:
        return
     
    data=message.payload.decode("utf-8")
    # data=data.replace('"',"'")
    data=json.loads(data)
    print("mqtt message received " ,data)
    # data={'From': 'nurse', 'To': 'client', 'StationNo': '1', 'Cmd': 50, 'State': 14}
    # data={"From":"mobile","To":"client","StationNo":"999","Cmd":50,"State":14}

    # print('jsonData:',data)
    # print('Cmd:',data['Cmd'])
    # print('stationNo:',data['StationNo'])
    # print('clientID:',clientID)
    #print("message qos=",message.qos)
    #print("message retain flag=",message.retain)
    
    bMqttConnected=True
    
    if data['StationNo']=='99' or data['StationNo']=='98'  or data['StationNo']==clientID  or data['StationNo']==int(clientID):
        if message.topic==topic_command:
        
            
            print("mqtt command:",data)
            if int(data['Cmd'])==cn.c_station_msg_power_on:
                if c_bSaveDepth==1 or c_bSaveImage==1:
                    createSaveDir(imageDir)
                setStartMonitor()
                bMaualOn=True
                bManualOff=False
            if int(data['Cmd'])==cn.c_station_msg_power_off: 
                bManualOff=True
                bMaualOn=False
                setStopMonitor()   
                
            if int(data['Cmd'])==cn.c_station_msg_start_monitor:
                if clientState!=cn.c_station_state_power_off:
                    setStartMonitor()
                
            elif int(data['Cmd'])==cn.c_station_msg_stop_monitor:
                if clientState!=cn.c_station_state_power_off:
                    setStopMonitor()
            elif int(data['Cmd'])==cn.c_station_msg_manual_sleep_locked:
                if clientState!=cn.c_station_state_power_off and clientState!=cn.c_station_state_monitor_stop :
                    if pointAboveSit<c_pointAboveSit or pointBedside<c_pointBedside:
                        bSleepLocked=True
                        depth_baseline=None
                        bPassby=False
                        bAlert=False
                        bNeedAlarm=False
                        prevState=cn.c_station_state_sleep_locked
                        clientState=cn.c_station_state_sleep_locked
                        depth_baseline=None
                        print('manual sleep lock')
                        emitClientState()   
            elif int(data['Cmd'])==cn.c_station_msg_frame_with_marker:
                if clientState!=cn.c_station_state_power_off:
                    sendImage(markedImage)
            elif int(data['Cmd'])==cn.c_station_msg_frame:
                if clientState!=cn.c_station_state_power_off:
                    sendImage(markedImage)
            elif int(data['Cmd'])==cn.c_station_msg_read_schedule: 
                readSchedule()
            elif int(data['Cmd'])==cn.c_station_msg_read_option: 
                readOption()
            elif int(data['Cmd'])==cn.c_station_msg_read_alert: 
                readAlert()
            elif int(data['Cmd'])==cn.c_station_msg_read_zone: 
                readZone()
            elif int(data['Cmd'])==cn.c_station_msg_read_margin: 
                readMargin()
            elif int(data['Cmd'])==cn.c_station_msg_read_parameter: 
                readPara()
        elif message.topic==topic_mobile_command:     
            print('mobile command:',data)
            if clientState!=cn.c_station_state_power_off:
                sendImageToMobile(markedImage)
            
        elif message.topic==topic_temperature:        
       
            maxT1=data['maxT1']
            maxT2=data['maxT2']
      
        elif message.topic==topic_breathState:
            determineBreath(data)
        elif message.topic==topic_set_schedule:
            setSchedule(data)
        elif message.topic==topic_set_option:
            setOption(data)
        elif message.topic==topic_set_alert:
            setAlert(data)
        elif message.topic==topic_set_zone:
            setZone(data)
        elif message.topic==topic_set_margin:
            setMargin(data)
        elif message.topic==topic_set_para:
            setPara(data)
        elif message.topic==topic_askState:
            emitClientState()
    
            

mqttClient.on_connect = on_connect
mqttClient.on_disconnect = on_disconnect
mqttClient.on_message=on_message #attach function to callback
#mqttClient.on_log=on_log #attach function to callback
mqttClient.connect(broker_address)
mqttClient.loop_start()
mqttClient.subscribe(topic_mobile_command)
mqttClient.subscribe(topic_server_keepalive)
mqttClient.subscribe(topic_client_command)
mqttClient.subscribe(topic_command)
mqttClient.subscribe(topic_temperature)
mqttClient.subscribe(topic_region)
mqttClient.subscribe(topic_breathState)
mqttClient.subscribe(topic_set_schedule)
mqttClient.subscribe(topic_set_option)
mqttClient.subscribe(topic_set_alert)
mqttClient.subscribe(topic_set_zone)
mqttClient.subscribe(topic_set_margin)
mqttClient.subscribe(topic_set_para)
mqttClient.subscribe(topic_askState)
#publishing {"From":"nurse","To":"client","StationNo":"1","Cmd":49,"State":13}

def loadPara():

    global para,paras_level,c_offset,c_sensorAngle,minPointArr,c_pointThreshold
    
    global c_sendImageInterval,c_motion_th,c_no_motion_count,no_motion_count, c_bed_motion_detect_count
    global c_floor_level,c_bed_level
    global c_yLeave,c_predict
    global c_bed_head_line
    global c_min_bedside_point,c_min_lying_point,c_min_sit_point
    global c_pointAboveSit,c_pointAboveSitHead,c_pointAboveSitEnd,  c_pointLying,c_pointBedsideRight,c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow,c_pointBedside,c_pointBedsideLow
    global c_diff_point_motion_th,c_diff_th
    global c_lyingCount,c_predict_lying_count,c_breath_count,c_breath_distance
    global c_detect_bed_motion_box_offset
    
    global c_depth_offset

    global c_thermal_change,c_thermal_adjust_count,thermal_adjust_count,c_min_thermal_th
    global c_use_motion_detect
    global c_lying_level,c_sit_level
    global c_other_bedend_offset
    global c_calc_bed_min_level
    global c_auto_detect_bed
    global c_pointOtherSide,c_pointOtherBedend
    with open(paraFile) as json_file:
        para = json.load(json_file)
        print('angle para:',para)


    c_auto_detect_bed=para["c_auto_detect_bed"]

    c_yLeave=para["c_yLeave"]
    c_bed_head_line=para["c_bed_head_line"]


    c_floor_level=para["c_floor_level"]
    if c_bed_level ==0:
        c_bed_level=para["c_bed_level"]     
    c_calc_bed_min_level=para["c_calc_bed_min_level"]


    
    c_lying_offset=para["c_lying_offset"]
    c_sit_offset=para["c_sit_offset"]
    c_bedside_offset=para["c_bedside_offset"]
    c_other_offset=para["c_other_offset"]
    c_fall_offset=para["c_fall_offset"]
    c_other_bedend_offset=para["c_other_bedend_offset"]
    c_detect_bed_motion_box_offset=para["c_detect_bed_motion_box_offset"]

    c_lyingCount=para["c_lying_count"]
    c_predict_lying_count=para["c_predict_lying_count"]

    
    c_motion_th=para["c_motion_th"]
    c_no_motion_count=para["c_no_motion_count"]
    c_use_motion_detect=para["c_use_motion_detect"]
    c_bed_motion_detect_count=para["c_bed_motion_detect_count"]

    c_min_bedside_point=para["c_min_bedside_point"]
    c_min_lying_point=para["c_min_lying_point"]
    c_min_sit_point=para["c_min_sit_point"]


    c_diff_th=para["c_diff_th"]
    c_diff_point_motion_th=para["c_diff_point_motion_th"]   
    c_pointLying=para["c_pointLying"]

    c_pointAboveSitHead=para["c_pointAboveSitHead"]
    c_pointAboveSitEnd=para["c_pointAboveSitEnd"]
    c_pointAboveSit=para["c_pointAboveSit"]
    
    c_pointBedsideRight=para["c_pointBedsideRight"]
    c_pointBedsideRightLow=para["c_pointBedsideRightLow"]
    c_pointBedsideLeft=para["c_pointBedsideLeft"]
    c_pointBedsideLeftLow=para["c_pointBedsideLeftLow"]
    c_pointBedside=para["c_pointBedside"]
    c_pointBedsideLow=para["c_pointBedsideLow"]
    
    c_pointOtherSide=para["c_pointOtherSide"]
    c_pointOtherBedend=para["c_pointOtherBedend"]

    c_breath_count=para["c_breath_count"]
    c_breath_distance=para["c_breath_distance"]

    c_min_thermal_th=para["c_min_thermal_th"]
    c_thermal_adjust_count=para["c_thermal_adjust_count"]
    c_thermal_change=para["c_thermal_change"]
    


    paras_level=c_floor_level,c_bed_level
    c_offset=c_lying_offset,c_sit_offset,c_bedside_offset,c_other_offset,c_fall_offset
    minPointArr=c_min_bedside_point,c_min_lying_point,c_min_sit_point
    c_pointThreshold= c_pointLying,c_pointAboveSit,c_pointAboveSitHead,c_pointAboveSitEnd, c_pointBedside,c_pointBedsideLow, c_pointBedsideRight,c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow
    c_lying_level=c_bed_level-c_lying_offset
    c_sit_level=c_bed_level-c_sit_offset
    c_depth_offset=c_lying_offset,c_sit_offset,c_bedside_offset,c_other_offset,c_fall_offset
    print('paras_level:',paras_level)
    print('c_offet:',c_offset)
   

def loadSystemConfig():
    global c_save_mode,conf_thres,c_save_image_type
    global c_bSaveDepth,c_bSaveImage,c_save_interval

    global c_use_breath_detect,c_use_thermal_detect
    global sendImageInterval
    global weightModel,c_predict,conf_thres,c_bRotateImage
    global c_sensorAngle
    global  deltaAngleX
    global c_FOV_H,c_FOV_V,c_FOV_D
    global c_clipping_distance_in_meters ,c_threshold_distance_in_meters
    global c_keepalive_interval
    with open(systemConfigFile) as json_file:
        
        systemconfig = json.load(json_file)
        print('systemconfig:',systemconfig)
    c_FOV_H=systemconfig["c_FOV_H"]
    c_FOV_V=systemconfig["c_FOV_V"]
    c_FOV_D=systemconfig["c_FOV_D"]    

    c_predict=systemconfig["c_predict"]
    weightModel=systemconfig["weightModel"]
    conf_thres=systemconfig["conf_thres"]

   
    c_clipping_distance_in_meters = systemconfig["c_clipping_distance_in_meters"]
    c_threshold_distance_in_meters = systemconfig["c_threshold_distance_in_meters"]
    
    c_bRotateImage=systemconfig["c_bRotateImage"]
    c_sensorAngle=systemconfig["c_sensorAngle"]
    c_bSaveDepth=systemconfig["c_bSaveDepth"]
    c_bSaveImage=systemconfig["c_bSaveImage"]
    c_save_mode=systemconfig["c_save_mode"]
    c_save_image_type=systemconfig["c_save_image_type"]
    c_save_interval=systemconfig["c_save_interval"]
    c_sendImageInterval=systemconfig["c_sendImageInterval"]
    
    
    
    c_use_breath_detect=systemconfig["c_use_breath_detect"]
    c_use_thermal_detect=systemconfig["c_use_thermal_detect"]
    c_total_day_to_keep=systemconfig["c_total_day_to_keep"]
    c_keepalive_interval=systemconfig["c_keepalive_interval"]


    sendImageInterval=c_sendImageInterval
    if c_sensorAngle==0:
        deltaAngleX=0
    else:
        deltaAngleX=(float)(c_FOV_H/640)
    
    print('deltaAngleX:',deltaAngleX)
    print('c_total_day_to_keep:',c_total_day_to_keep)
    

    loadPara()

    


    
cx=x1+round((x2-x1)/2)
cy=y1+round((y2-y1)/2)
cursor_x=cx 
cursor_y=cy
xyArray=x1,y1,x2,y2
bed_width=y2-y1
depth_array=[]
paras_level=0,0
c_offset=0,0,0,0,0
minPointArr=c_min_bedside_point,c_min_lying_point,c_min_sit_point


deltaAngleX=0
c_pointThreshold= c_pointLying,c_pointAboveSit,c_pointAboveSitHead,c_pointAboveSitEnd, c_pointBedside,c_pointBedsideLow, c_pointBedsideRight,c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow
pointAboveSit=0
pointLying=0
pointBedHead=0
pointBedEnd=0
pointBedsideRight=0
pointBedsideLeft=0
pointBedsideRightLow=0
pointBedsideLeftLow=0
pointBedside=0
pointBedsideLow=0
pointBedsideOffset=0
breath_count=0
bBedsideMotion=False
bBedMotion=False
pointOffsetArr= pointLying, pointAboveSit,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow
bGetPointOffset=True
bMotion=False
bMonitor=False
bSitLocked=False
bLyingLocked=False
bPassby=False
bOtherPerson=False
thermal_adjust_count=0
thermal_th=40
sendImageInterval=c_sendImageInterval
clientState=cn.c_station_state_unknown
predictedState=cn.c_station_state_bed_only
breathStatus= {'StationNo': 0, 'BreathCounter': 2, 'BreathBpm': 0.0, 'BreathDistance': 0.0, 'BreathState': 4}

print('breathStatus:',breathStatus['BreathState'])
loadSystemConfig()
readOption()
readMargin()
readZone()
readAlert()
ix = -1
iy = -1
drawing = False
arr=[]
for i in range(w):
    col = []
    for j in range(h):
        col.append(0.0)
    arr.append(col)

pc = rs.pointcloud()


arr = np.arange(w*h).reshape(h,w)
bg = np.arange(w*h).reshape(h,w)

bg[240, 320]=10

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  c_clipping_distance_in_meters meters away
 #meter
clipping_distance = c_clipping_distance_in_meters / depth_scale
c_threshold_distance_in_meters = .1 #meter
threshold_distance = c_threshold_distance_in_meters / depth_scale
print('clipping_distance:',clipping_distance)
print('threshold_distance:',threshold_distance)
def funcGet():
    txt=entry.get()
    print('clicked',txt)


    
#@jit 
def compareDepth(prevDepth,curDepth,margin ,c_yLeave,xyArray,c_diff_th,c_diff_point_motion_th):
    
    bBedsideMotion=False
    bBedMotion=False
    bMotion=False
    x1,y1,x2,y2=xyArray
    c_check_front_side,c_check_back_side,c_check_left_side,c_check_right_side,c_front_margin,c_back_margin,c_left_margin,c_right_margin,c_bed_head_pos,c_detect_side=margin


    arr1=curDepth[y1+c_detect_bed_motion_box_offset:y2-c_detect_bed_motion_box_offset,x1+c_detect_bed_motion_box_offset:x2-c_detect_bed_motion_box_offset]
    arr2=prevDepth[y1+c_detect_bed_motion_box_offset:y2-c_detect_bed_motion_box_offset,x1+c_detect_bed_motion_box_offset:x2-c_detect_bed_motion_box_offset]

    bedDiff = np.subtract(arr2,arr1)
   
    bedDiffPoint = (bedDiff > c_diff_th).sum()
    if bedDiffPoint>c_diff_point_motion_th:
        bBedMotion=True
        bMotion=True


    if c_detect_side==cn.c_detect_leftside:    #left
        arr1=curDepth[y1-c_yLeave:y1,x1:x2]
        arr2=prevDepth[y1-c_yLeave:y1,x1:x2]
        LeaveDiff = np.subtract(arr2,arr1)
        bedsideDiffPoint = (LeaveDiff > c_diff_th).sum()
        if bedsideDiffPoint>c_diff_point_motion_th:
            bBedsideMotion=True
            bMotion=True
    elif c_detect_side==cn.c_detect_rightside:    #right
        arr1=curDepth[y2:y2+c_yLeave,x1:x2]
        arr2=prevDepth[y2:y2+c_yLeave,x1:x2]
        LeaveDiff = np.subtract(arr2,arr1)
        bedsideDiffPoint = (LeaveDiff > c_diff_th).sum()
        if bedsideDiffPoint>c_diff_point_motion_th:
            bBedsideMotion=True
            bMotion=True
    else: #both
        arr1=curDepth[y1-c_yLeave:y1,x1:x2]
        arr2=prevDepth[y1-c_yLeave:y1,x1:x2]
        LeaveDiff = np.subtract(arr2,arr1)
        bedsideDiffPoint = (LeaveDiff > c_diff_th).sum()
        if bedsideDiffPoint>c_diff_point_motion_th:
            bBedsideMotion=True
            bMotion=True
        arr1=curDepth[y2:y2+c_yLeave,x1:x2]
        arr2=prevDepth[y2:y2+c_yLeave,x1:x2]
        LeaveDiff = np.subtract(arr2,arr1)
        bedsideDiffPoint = (LeaveDiff > c_diff_th).sum()
        if bedsideDiffPoint>c_diff_point_motion_th:
            bBedsideMotion=True
            bMotion=True
    

    return bBedsideMotion,bBedMotion,bMotion,bedsideDiffPoint,bedDiffPoint


#@jit                         
def funcAngle(margin,c_other_bedend_offset,c_offset,depthArray,diffArray,verts,bedLevel,c_pointThreshold,xyArray,yLeave,c_bed_head_line,paras_level,c_sensorAngle,deltaAngleX,pointOffsetArr,pointBedsideOffset,minPointArr):
    

    pointAboveSitOffset,pointBedsideLowOffset,pointBedsideRightOffset,pointBedsideLeftOffset,pointBedsideRightLowOffset,pointBedsideLeftLowOffset,pointLyingOffset=pointOffsetArr
    
    c_pointLying,c_pointAboveSit,c_pointAboveSitHead,c_pointBedside,c_pointBedsideLow, c_pointAboveSitEnd, c_pointBedsideRight,c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow =c_pointThreshold

    w, h = 640, 480
    x1,y1,x2,y2=xyArray
    cx,cy=240,320
    step=1
   
    c_lying_offset,c_sit_offset,c_bedside_offset,c_other_offset,c_fall_offset=c_offset
    c_floor_level,c_bed_level=paras_level
    c_lying_level=c_bed_level-c_lying_offset
    c_sit_level=c_bed_level-c_sit_offset
    pointAboveSit=0
    pointLying=0
    pointBed=0
    pointBedsideRight=0
    pointBedsideLeft=0
    pointBedsideRightLow=0
    pointBedsideLeftLow=0
    pointBedside=0
    pointBedsideLow=0
    pointBedHead=0
    pointBedEnd=0
    pointOtherBedside=0
    pointOtherBedend=0
    xSitCoor=0
    ySitCoor=0
    xRightCoor=0
    yRightCoor=0
    xLeftCoor=0
    yLeftCoor=0
    sitTotalWeight=0
    leftTotalWeight=0
    rightTotalWeight=0
    maxBedDist=0
    count=0

    mx=x1+int((x2-x1)/2)
    
         
    
    c_check_front_side,c_check_back_side,c_check_left_side,c_check_right_side,c_front_margin,c_back_margin,c_left_margin,c_right_margin,c_bed_head_pos,c_detect_side=margin

    if c_detect_side==cn.c_detect_leftside: 
        arr1=diffArray[y1-c_yLeave-c_front_margin:y1-c_yLeave,x1:x2]
        pointOtherBedside = (arr1 > c_other_offset).sum()
        # print('left pointOtherBedside1:',pointOtherBedside)

        arr1=diffArray[y1-c_yLeave:y1,x1:x2]    #leave region
        pointBedsideLeft = (arr1 > c_bedside_offset).sum()
        pointBedside=pointBedside+pointBedsideLeft
        # print('left pointBedside:',pointBedsideLeft)
        pointBedsideLeftLow = (arr1 > c_fall_offset).sum() #fall
        pointBedsideLow=pointBedsideLow+pointBedsideLeftLow
        # print('left pointBedsideLeftLow:',pointBedsideLeftLow)
        pointOtherBedend=0
        if c_check_left_side:
            arr1=diffArray[y1-c_yLeave-c_front_margin:y2,x1-c_left_margin:x1] #bed end
            pointOtherBedend = (arr1 > c_lying_offset).sum()
        if c_check_right_side:
            arr1=diffArray[y1-c_yLeave-c_front_margin:y2,x2:x2+c_right_margin] #bed end
            pointOtherBedend =pointOtherBedend+ (arr1 > c_lying_offset).sum()
        
    if c_detect_side==cn.c_detect_rightside: 
        arr1=diffArray[y2+c_yLeave:y2+c_yLeave+c_back_margin,x1:x2]
        pointOtherBedside = (arr1 > c_other_offset).sum()
        # print('right pointOtherBedside:',pointOtherBedside)

        arr1=diffArray[y2:y2+c_yLeave,x1:x2]    #leave region
        pointBedsideRight = (arr1 > c_bedside_offset).sum()
        pointBedside=pointBedside+pointBedsideRight
        # print('right pointBedside:',pointBedsideRight)
        pointBedsideRightLow = (arr1 > c_fall_offset).sum() #fall
        pointBedsideLow=pointBedsideLow+pointBedsideRightLow
        # print('right pointBedsiderightLow:',pointBedsideRightLow)
        pointOtherBedend=0
        if c_check_left_side:
            arr1=diffArray[y2:y2+c_yLeave+c_back_margin,x1-c_left_margin:x1] #bed end
            pointOtherBedend = (arr1 > c_lying_offset).sum()
        if c_check_right_side:
            arr1=diffArray[y2:y2+c_yLeave+c_back_margin,x2:x2+c_right_margin] #bed end
            pointOtherBedend =pointOtherBedend+ (arr1 > c_lying_offset).sum()
    # print('pointOtherBedend:',pointOtherBedend)

    arr1=diffArray[y1:y2,x1:mx] #bed
    pointBedHead = (arr1 > c_sit_offset).sum()
    pointAboveSit=pointAboveSit+pointBedHead
    # print('pointBedHead:',pointBedHead)

    pointLying = (arr1 > c_lying_offset).sum()

    arr1=diffArray[y1:y2,mx:x2] #bed
    pointBedEnd = (arr1 > c_sit_offset).sum()
    pointAboveSit=pointAboveSit+pointBedEnd
    # print('pointBedEnd:',pointBedEnd)

    # print('pointAboveSit:',pointAboveSit)



 
    pointBedsideOffset1=pointBedside
    
    pointOffset=pointLying,pointAboveSit,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow

    resultPointArr=pointOtherBedside,pointOtherBedend,pointBed,pointBedHead,pointBedEnd, pointLying,pointAboveSit,pointBedside,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow,xSitCoor,ySitCoor,xRightCoor,yRightCoor,xLeftCoor,yLeftCoor

    return resultPointArr,pointOffset,pointBedsideOffset1,maxBedDist





prevPointBedside=0
def  determineState(resultPointArr):
    global detectOptions
    global c_pointThreshold,clientState,prevState
    global bSitLocked,bLyingLocked,bSleepLocked,bAlert,bNeedAlarm
    global pointBedsideOffset,cx,cy
    global lyingCount,c_lyingCount,c_predict_lying_count
    global lyingPoint,lyingPointIdx
    global bLocked,bPassby,bOtherPerson
    global breathStatus,bBreathLocked
    global bLyingPredicted,c_predict
 
    global bed_motion_detect_count,c_bed_motion_detect_count
    global depth_baseline
    cgCenterX=cx
    cgCenterY=cy
    state=prevState
    
    pointOtherBedside,pointOtherBedend, pointBed,pointBedHead,pointBedEnd, pointLying,pointAboveSit,pointBedside,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow,xSitCoor,ySitCoor,xRightCoor,yRightCoor,xLeftCoor,yLeftCoor=resultPointArr
    c_pointLying,c_pointAboveSit,c_pointAboveSitHead,c_pointAboveSitEnd,c_pointBedside,c_pointBedsideLow,c_pointBedsideRight,c_pointBedsideRightLow,c_pointBedsideLeft,c_pointBedsideLeftLow =c_pointThreshold
    
    if not bSleepLocked:
        if state==cn.c_station_state_lying or state==cn.c_station_state_bed_only:
            bPassby=False
            bAlert=False
            bNeedAlarm=False
            if pointAboveSit>c_pointAboveSit:
                if pointBedHead>pointBedEnd:
                    state=cn.c_station_state_sit_bed_head
                
                else:
                    state=cn.c_station_state_sit_bed_end
                bLyingPredicted=False
            
            elif pointBedside>c_pointBedside :
                state=cn.c_station_state_off_bed
                
            else:
                if c_use_thermal_detect:
                    if maxT1<thermal_th and  maxT2<thermal_th:
                        state=cn.c_station_state_bed_only
                else:
                    if detectOptions["MotionSleepLock"]:
                        
                        if bed_motion_detect_count>c_bed_motion_detect_count:
                            print('motion sleep locked')
                            bSleepLocked=True
                            bPassby=False
                            depth_baseline=None
                            state=cn.c_station_state_sleep_locked
                    else:
                        if pointLying>c_pointLying:
                            print('lying point sleep locked')
                            bSleepLocked=True
                            bPassby=False
                            depth_baseline=None
                            state=cn.c_station_state_sleep_locked
    
            
            return state


 # alertType: {'From': 'nurse', 'To': 'client', 'StationNo': '1', 
    # 'SitBedHead': True, 'SitBedEnd': True, 'OffBed': True, 'LeavingBed': True, 'OtherPerson': True,
    # 'BothLeave': True, 'AdnormalBreathing': True, 'StandOnBed': False,
    # 'EnableManaulSitBedHeadReset': False, 'EnableManaulSitBedEndReset': False,
    # 'EnableManaulOffBedReset': True, 'EnableManaulLeaveReset': True, 
    # 'EnableManaulOtherReset': True, 'EnableManaulBothLeavelReset': True,
    # 'EnableManaulBreathReset': False, 'EnableManaulStandOnBedRese': False,
    # 'SitBedHeadDelay': 1, 'SitBedEndDelay': 1, 'OffBedDelay': 1, 'BedLeaveDelay': 1,
    # 'OtherPersonDelay': 1, 'BothLeaveDelay': 1, 'AbnormalBreathDelay': 1, 'StandOnBedDelay': 1}

    if  bSleepLocked:
        if prevState==cn.c_station_state_leave or prevState==cn.c_station_state_both_leave :
            return state

        if  prevState==cn.c_station_state_other_person:
            if  alertType["EnableManaulOtherReset"]:
                
                if pointBedside<c_pointBedside and pointAboveSit<c_pointAboveSitEnd  and  pointAboveSit<c_pointAboveSitHead :
                    if pointLying<c_pointLying:
                        state=cn.c_station_state_bed_only
                        bLyingLocked=False
                        bSleepLocked=False
                        bSitLocked=False
                        lyingCount=0
                        bAlert=False

                        bPassby=False
                        bBreathLocked=False
                    
            return state
        elif prevState==cn.c_station_state_sleep_locked:
            if bPassby:
                if pointAboveSit>c_pointAboveSit:
                    bAlert=True
                    
                    state=cn.c_station_state_other_person
                    if alertType["OtherPerson"]:
                        bNeedAlarm=True
                    else:
                        bNeedAlarm=False
                elif pointBedside<c_pointBedside:
                    if c_use_thermal_detect==1:
                        if maxT1<thermal_th and maxT2<thermal_th:
                            state=cn.c_station_state_both_leave
                            
                            bAlert=True
                            if alertType["BothLeave"]:
                                bNeedAlarm=True
                            else:
                                bNeedAlarm=False
                            return state
                        else:
                            if pointLying<c_pointLying:
                                state=cn.c_station_state_both_leave
                            
                                bAlert=True
                                bNeedAlarm=True
                                return state
                    else:
                        bPassby=False
                        bed_motion_detect_count=0
                else:
                    bed_motion_detect_count=0
                
               
                
            elif pointAboveSit>c_pointAboveSit:
                bSitLocked=True
                bAlert=True
                bLocked='Sit locked'
                if pointBedHead> pointBedEnd:
                    state=  cn.c_station_state_sit_bed_head 
                    if alertType["SitBedHead"]:
                        bNeedAlarm=True
                    else:
                        bNeedAlarm=False
                else:
                    state=cn.c_station_state_sit_bed_end
                    if alertType["SitBedEnd"]:
                        bNeedAlarm=True
                    else:
                        bNeedAlarm=False

                
                
            elif pointBedside>c_pointBedside:
                bed_motion_detect_count=0
                bPassby=True
             
            return state    
                
        elif prevState==cn.c_station_state_sit_bed_head :
            if pointBedside>c_pointBedside:
                state=cn.c_station_state_off_bed
                
                bAlert=True
                bNeedAlarm=True
                
            elif pointAboveSit<c_pointAboveSit:
                if not alertType["EnableManaulSitBedHeadReset"]:

                    state=cn.c_station_state_sleep_locked
                    
                    bAlert=False
                    bNeedAlarm=False
            

            return state
        elif  prevState==cn.c_station_state_sit_bed_end:
            
            if pointBedside>c_pointBedside:
                state=cn.c_station_state_off_bed
                
                bAlert=True
                
            elif pointAboveSit<c_pointAboveSit:
                if not alertType["EnableManaulSitBedEndReset"]:
                    state=cn.c_station_state_sleep_locked
                    
                    bAlert=False
                    bNeedAlarm=False

            return state

        elif prevState==cn.c_station_state_off_bed:
 
            if pointBedside<c_pointBedside:                
                bSitLocked=False
                bLyingLocked=False
                state=cn.c_station_state_leave
                
                bAlert=True
                bNeedAlarm=True
            return state
       
      
        return state
    
    '''
    before sleep locked
    '''
#     if breathStatus["BreathState"]==cn.c_breath_state_no_movement:
#         if prevState!=cn.c_station_state_monitor_start:
#             state=cn.c_station_state_bed_only
#         bLyingLocked=False
#         bSleepLocked=False
#         bSitLocked=False
#         lyingPointIdx=0
#         bAlert=False
#         bLocked='none'
#         return state     
    bPassby=False
    bNeedAlarm=False
    bAlert=False
    if prevState==cn.c_station_state_monitor_start:
       
        if pointBedside<c_pointBedside and pointAboveSit<c_pointAboveSitEnd  and  pointAboveSit<c_pointAboveSitHead :
            #if xSitCoor==0 and  ySitCoor==0:
            state=cn.c_station_state_bed_only
            bLyingLocked=False
            bSleepLocked=False
            bSitLocked=False
            lyingCount=0
            bAlert=False
            bLocked='none'
            bBreathLocked=False
            return state        

    if prevState==cn.c_station_state_bed_only or prevState==cn.c_station_state_monitor_start:
        if pointBedside>c_pointBedside:
            state=cn.c_station_state_off_bed
            print('off bed')
            return state
        if pointAboveSit<c_pointAboveSit :
            if c_use_thermal_detect==1:
                if maxT1>thermal_th or maxT2>thermal_th:
                    
                    state=cn.c_station_state_lying
            else:
                if pointLying>c_pointLying:
                    state=cn.c_station_state_lying
                    
            return state  
    if prevState==cn.c_station_state_off_bed or prevState==cn.c_station_state_monitor_start:
        if not c_predict:
            if pointBedside<c_pointBedside and pointAboveSit<c_pointAboveSit :
                if maxT1>thermal_th or maxT2>thermal_th:
                    
                    state=cn.c_station_state_lying
                    lyingCount=0
                        
                else:
                    lyingCount=0
                    state=cn.c_station_state_bed_only
                return state       
        if pointBedside>c_pointBedside :
            
            return state
        if pointBedside<c_pointBedside and pointAboveSit<c_pointAboveSitEnd  and  pointAboveSit<c_pointAboveSitHead and pointLying<c_pointLying:
            #if xSitCoor==0 and  ySitCoor==0:
            state=cn.c_station_state_bed_only
            bLyingLocked=False
            bSleepLocked=False
            bSitLocked=False
            bAlert=False
            bLocked='none'
            bBreathLocked=False
            return state 

          
        if pointAboveSit>c_pointAboveSit:
            if pointBedHead>pointBedEnd:
                state=cn.c_station_state_sit_bed_head
            else:
                state=cn.c_station_state_sit_bed_end
        return state
    if prevState==cn.c_station_state_sit_bed_head or prevState==cn.c_station_state_monitor_start:
        if pointAboveSit>c_pointAboveSit:
            if pointBedHead>pointBedEnd:
                state=cn.c_station_state_sit_bed_head
            else:
                state=cn.c_station_state_sit_bed_end
            return state
     
        if pointBedside>c_pointBedside:
            lyingCount=0
            state=cn.c_station_state_off_bed
            return state
         
        if c_use_thermal_detect==1:
            if maxT1>thermal_th or maxT2>thermal_th:
                state=cn.c_station_state_lying
                if bBreathLocked:
                    print('breath sleep locked')
                    bSleepLocked=True
                    bPassby=False
                    depth_baseline=None
                    clientState=cn.c_station_state_sleep_locked
                    bLocked='sleep locked'
                    bSitLocked=False
                lyingCount=0 
            else:
                state=cn.c_station_state_bed_only
        else:
           
            state=cn.c_station_state_lying    

        return state
    
    if prevState==cn.c_station_state_sit_bed_end or prevState==cn.c_station_state_monitor_start:
        if pointAboveSit>c_pointAboveSit:
            if pointBedHead>pointBedEnd:
                state=cn.c_station_state_sit_bed_head
            else:
                state=cn.c_station_state_sit_bed_end
            return state

        if pointBedside>c_pointBedside:
            
            state=cn.c_station_state_off_bed
            return state
        if c_use_thermal_detect==1: 
            if maxT1>thermal_th or maxT2>thermal_th:

                state=cn.c_station_state_lying
                if bBreathLocked:
                    print('breath sleep locked')
                    bSleepLocked=True
                    depth_baseline=None
                    state=cn.c_station_state_sleep_locked
                    bLocked='sleep locked'
                    bSitLocked=False
            else:
                lyingCount=0 
                state=cn.c_station_state_bed_only
        else:
            state=cn.c_station_state_lying 

        return state  


    return state

temp_x1=0
temp_y1=0
def draw_rectangle_with_drag(event, x, y, flags, param):
    global detectZone
    global bCaptureBG,cx,cy,x1,y1,x2,y2,depth_array,xyArray ,drawing
    global temp_x1,temp_y1,cursor_x,cursor_y
    global bGetPointOffset,counter,c_yLeave
    if event == cv2.EVENT_RBUTTONDOWN:
        pass
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        temp_x1 = x
        temp_y1 = y
        cursor_x=x
        cursor_y=y
        #print('dist:',depth_array[y,x])

    elif event == cv2.EVENT_MOUSEMOVE:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

        if x-temp_x1>50 and y-temp_y1>50:
            x1=temp_x1
            y1=temp_y1
            x2=x
            y2=y
            
            # if y2>479-c_yLeave:
            #     y2=479-c_yLeave
            xyArray=x1,y1,x2,y2
            detectZone["x1"]=x1
            detectZone["x2"]=x2
            detectZone["y1"]=y1
            detectZone["y2"]=y2
            setZone(detectZone)
            # print('save xyArray',xyArray)
            # np.save('xyConfig',xyArray)
            bGetPointOffset=True
            counter=0
            
        #bCaptureBG=True



def depth2base64(color_map):
    
    img=color_map
    cv_img=cv2.resize(img, dsize=(320, 240), interpolation=cv2.INTER_CUBIC)
    
#     cv2.imwrite("out.jpg",cv_img)
# 
#     with open("out.jpg", "rb") as image2string:
#         f=image2string.read()
    
    f,buf = cv2.imencode('.jpg', cv_img)
    encoded_string = base64.b64encode(buf)

    img=base64.b64decode(encoded_string)
    #encoded_string=b'data:image/png;base64,' + encoded_string
    #encoded_string='data:image/png;base64,'+encoded_string.decode('utf-8')
    encoded_string=encoded_string.decode('utf-8')
#     print('encoded string:',encoded_string)
#     text_file = open("encoded string.txt", "w")
#     text_file.write(encoded_string)
#     text_file.close()
# 
#     decodeit = open('out1.jpg', 'wb')
#     decodeit.write(img)
#     decodeit.close()
    return encoded_string
pointAboveSit=0
pointBedsideRight=0
pointBedsideLeft=0
pointLying=0


def trackbarCallback(val):
    global clientState,bMonitor,prevState
    global bAlert,bLyingLocked,bSitLocked
    global bSleepLocked,lyingCount
    global bGetPointOffset,counter,breath_count
    global bBreathLocked
    global bLyingPredicted,bSitPredicted,bOffbedLocked
    print('button val:',val)
    if val==1 and clientState==cn.c_station_state_monitor_stop:
        setStartMonitor()

        
    else:
        setStopMonitor()



def maxMin(data):
    global cx,cy,a,b

   
    maxval= np.max(data)
    minval= np.min(data)
    
    max_index_col = np.argmax(data, axis=0)
    max_index_row = np.argmax(data, axis=1)
    
    mean=round(np.mean(data),2)
    
    
    maxval=round(a*maxval+b,2)
    minval=round(a*minval+b,2)
    mean=round(a*mean+b,2)
    
    
    return maxval,minval,mean ,max_index_col ,max_index_row       

def temperature_of_raspberry_pi():
    cpu_temp = os.popen("vcgencmd measure_temp").readline()
    cpu_temp=cpu_temp.replace("temp=", "")
    print('pi temperature:',cpu_temp)

bError=False

def blink():
    global pipeline,bError
    global detectOptions,reStartAfterStopDelay
    global bMqttConnected
    global ledRunningPin,ledMonitorPinblink
    global bBlink,bMonitor
    global clientState
    global clientStateCode
    global   c_alarmInterval
    global   alarmIntervalCount
    global hostName
    global bMotion,no_motion_count
    global bMonitor,bSave
    global clientTimeout,c_clientTimeout
    global monitorDelay,c_monitorDelay
    
    global pointAboveSit,pointBedside,pointLying,prevPointBedside
    global pointBedsideOffset,bLyingLocked,bSleepLocked
    global prevPointBedsideOffset, pointBedsideOffset
    global bRemoveBedsideOnly,save_interval
    global c_sendImageInterval,sendImageInterval
    global c_thermal_adjust_count,thermal_adjust_count,c_thermal_change,thermal_th
    global maxT1,maxT2
    global bWithinTime,bMaualOn,bManualOff,powerSchedule
    global depth_baseline
    global keepalive_interval,bServerConnected
    global captureCount
    threading.Timer(1, blink).start()
    if not bError:
        
        save_interval  =save_interval+1
        if captureCount<2:
            captureCount+=1
        no_motion_count=no_motion_count+1
        if save_interval>= c_save_interval:
            save_interval=0
            bSave=True
            temperature_of_raspberry_pi()
            # if not bRealsenseOK and clientState!=cn.c_station_state_power_off:
            #     os.system('sudo reboot')
            

        
        if clientState!=cn.c_station_state_power_off:
            if not bMonitor :
                
                if int(detectOptions["ReStartAfterStopDelay"])>0:
                    reStartAfterStopDelay=reStartAfterStopDelay+1
                    if reStartAfterStopDelay>int(detectOptions["ReStartAfterStopDelay"])*60:
                        setStartMonitor()
                        reStartAfterStopDelay=0
        keepalive_interval+=1
        if keepalive_interval>c_keepalive_interval:
            keepalive_interval=0
            bServerConnected=False
            

        alarmIntervalCount=alarmIntervalCount+1
        if alarmIntervalCount>c_alarmInterval:
            if breathMsg=='no movement':
                depth_baseline=None

            bWithinTime=sch.compareTime(powerSchedule)
            if bWithinTime:
                bMaualOn=False
                if bManualOff:
                    clientState=cn.c_station_state_power_off
                    bMonitor=False
                else:
                    if clientState==cn.c_station_state_power_off:
                        setStartMonitor()
            else:
                bManualOff=False    
                if not bMaualOn:
                    clientState=cn.c_station_state_power_off
                    
                    bMonitor=False

            if clientState!=cn.c_station_state_power_off:
                if abs(prevPointBedsideOffset-pointBedsideOffset)<1000:
                    if not bMotion:
                        if not bSleepLocked:
                            pass
                            #bRemoveBedsideOnly=True
                prevPointBedsideOffset=pointBedsideOffset
            print('manual on/off:',bMaualOn,bManualOff)
            print('send clientKeepAlive message',clientState)
        
                
            alarmIntervalCount=0
        # sio.emit('clientKeepAlive message', {'hostName':hostName,'stationNo':clientID})
            
            current_date=datetime.now().strftime('%Y-%m-%d')
            current_time=datetime.now().strftime('%H-%M-%S')
            bKeepAlive=True
            if clientState!=cn.c_station_state_power_off:
                if bLyingLocked and clientState==cn.c_station_state_lying:
                    print('blying sleeplocked')
                    bSleepLocked=True
                    bPassby=False
                    depth_baseline=None
                    bLocked='sleep locked'
                    clientState=cn.c_station_state_sleep_locked
            emitClientState()
        
        
    
        # sendImageInterval=sendImageInterval-1
        # if sendImageInterval<0:
        #     sendImageInterval=c_sendImageInterval
        #    sendImageInterval=random.randint(1,c_sendImageInterval)
            
        #     sendImage()
    
            
depth_baseline = None
prevDepth=None
curDepth=None
captureCount=0
   


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)




########################################
########################################
########################################


bCaptureBG=True
captureCount=0
cv2.namedWindow('colorMap', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar('Button','colorMap',0,1,trackbarCallback)
cv2.setMouseCallback('colorMap',
                     draw_rectangle_with_drag)



path = './directoryLog.txt'

if not os.path.isfile(path):
    file1 = open(path, "w")
    file1.close()
    

    

        
def detectMotion(image,gray,state):
    global detectMargin
    global static_back,c_motion_th
    global xyArray
    global c_yLeave
    bBedsideMotion=False
    bBedMotion=False
    bMotion=False
    x1,y1,x2,y2=xyArray
    # Difference between static background
    # and current frame(which is GaussianBlur)

    diff_frame = cv2.absdiff(static_back, gray,c_motion_th)
    
    # If change in between static background and
    # current frame is greater than c_motion_th it will show white color(255)
    thresh_frame = cv2.threshold(diff_frame, c_motion_th, 255, cv2.THRESH_BINARY)[1]
    thresh_frame = cv2.dilate(thresh_frame, None, iterations = 2)
 
    # Finding contour of moving object
    cnts,_ = cv2.findContours(thresh_frame.copy(),
                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
    for contour in cnts:          
        if cv2.contourArea(contour) < 500:
            continue
        
        (x, y, w, h) = cv2.boundingRect(contour)
        
        if x>x1 and x<x2:
            if c_detect_side==cn.c_detect_bothside:#both
                if y<y1 and y>y1-c_yLeave or y>y2 and y<y2+c_yLeave :
                    bBedsideMotion=True
                    bMotion = True
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 3)
            elif c_detect_side==cn.c_detect_leftside:#left
                if y>y2 and y<y2+c_yLeave :
                    bBedsideMotion=True
                    bMotion = True
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 3)
            else:
                if y<y1 and y>y1-c_yLeave:
                    bBedsideMotion=True
                    bMotion = True
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 3)
            
            

            if  y>y1 and y<y2:
                bBedMotion=True
                bMotion = True
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 3)
       
        static_back=None
        
    
    return bBedsideMotion,bBedMotion,bMotion


            

def putText(images,message,pos,textColor):
    
    fontScale = .8
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2

    cv2.putText(images,message, pos, font,fontScale, textColor, thickness, cv2.LINE_AA)
          
def saveData(img):
    global c_bSaveImage,c_bSaveDepth,imageCount,savePath
    global depth_array
    

    if c_bSaveImage==1:
        print('imageCount:',imageCount)
        fname=datetime.now().strftime('%H-%M-%S')
        #cv_img=cv2.resize(prcoessed_depth_colormap, dsize=(320, 240), interpolation=cv2.INTER_CUBIC)
        
        # print('save image fname:',savePath+stateMsg[clientState]+"-"+fname+".jpg")
        if  bServerConnected:
            
            cv2.imwrite(savePath+fname+"-"+stateMsg[clientState]+"-"+str(bSleepLocked)+"-"+breathMsg+".jpg",img)
        else:
            cv2.imwrite(savePath+fname+"-"+stateMsg[0]+"-"+str(bSleepLocked)+"-"+breathMsg+".jpg",img)

        saveData=imageCount+1
    if c_bSaveDepth:
        # print('save depth fname:',savePath+stateMsg[clientState]+"-"+fname+".jpg")
        if  bServerConnected:
            
            np.save(savePath+fname+"-"+stateMsg[clientState]+"-"+str(bSleepLocked)+"-"+breathMsg,depth_array)
        else:
            np.save(savePath+fname+"-"+stateMsg[0]+"-"+str(bSleepLocked)+"-"+breathMsg,depth_array)

bPredicted=False

def predictLoop():
    global weightModel,conf_thres
    global prcoessed_depth_colormap,depth_colormap
    
    global scale,predictArray,c_predict

    global bLyingPredicted,bSitPredicted,bPredicted
    global bAlert,bNeedAlarm,clientState,bSitLocked,bSleepLocked
    global predictedState,lyingCount
    color=cn.TEXT_COLOR_BLUE
    TFLITE_MODEL_PATH = "depth.tflite" #@param {type:"string"}
    options = detector.ObjectDetectorOptions(
          num_threads=4,
          score_threshold=conf_thres,
    )
    start_point=(0,0)
    end_point=(0,0)
    bPredicted=False
    
    while True:
        if c_predict==1:
            if prcoessed_depth_colormap is not None:
                start1 = timer()
                
                #depth_gray_3d=np.dstack([depth_gray,depth_gray,depth_gray])
                #depth_col=0ormap = cv2.cvtColor(depth_colormap, cv2.COLOR_RGB2BGR)
                detects = detector.ObjectDetector(model_path=weightModel, options=options)
                image = cv2.cvtColor(prcoessed_depth_colormap, cv2.COLOR_BGR2RGB)
                
                
                detections = detects.detect(image)
                
                bDetected,detected_class,score,bounding_box= detector.visualize(depth_colormap, detections)
                #print("predict time:",str( round(timer()-start1,2)))   
                result_text = detected_class + ' (' + str(score) + ')'
                text_location = (cn.MARGIN + bounding_box[0],
                                 cn.MARGIN + cn.ROW_SIZE + bounding_box[1])
                if score>conf_thres:
                    start_point =bounding_box[0]*scale, bounding_box[1]*scale
                    end_point = bounding_box[2]*scale, bounding_box[3]*scale
                    bPredicted=True
                    #print('start point:',start_point)
                    # cv2.rectangle(depth_colormap, start_point, end_point, txtColor,2 )
                    # putText(depth_colormap, result_text+'-'+str(score),cn.cn.org[0], txtColor)
                    # cv2.imshow('colorMap', depth_colormap)
                    predictArray=detected_class,score,start_point,end_point,color,bounding_box
                    #print('predictArray:',predictArray)
                    if detected_class==className[c_class_offbed]:
                        color=cn.TEXT_COLOR_RED
                        bLyingPredicted=False
                        bSitPredicted=False
                        
                    elif detected_class==className[c_class_sit]:
                        color=cn.TEXT_COLOR_YELLOW
                        bLyingPredicted=False
                        bSitPredicted=True
                        
                        
                    elif detected_class==className[c_class_lying]:
                        color=cn.TEXT_COLOR_BLUE
                        #if start_point.getX >x1 and end_point.getX()<x2:
                        bLyingPredicted=True
                        bSitPredicted=False
                   
                    else:
                        bLyingPredicted=False
                        detected_class='none'
                        bPredicted=False
                else:
                    bLyingPredicted=False
                    detected_class='none'
                    bPredicted=False

  
                      

    

class predictThread (threading.Thread):   
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):                  
        print ("Starting detect" + self.name)
        predictLoop()
         
  

def center_point(bounding_box):
    
    return (bounding_box[0]+(bounding_box[2]-bounding_box[0])/2)*scale,(bounding_box[1]+(bounding_box[3]-bounding_box[1])/2)*scale

def adjustThermalThreshold(bMotion):
    global thermal_th,c_thermal_adjust_count,c_thermal_change,thermal_adjust_count
    global maxT1,maxT2,c_min_thermal_th
    global clientState
    if bMotion:
        thermal_adjust_count=0
    else:
        thermal_adjust_count=thermal_adjust_count+1
        if thermal_adjust_count>c_thermal_adjust_count:
            thermal_adjust_count=0
            if np.maximum(maxT1,maxT2)>c_min_thermal_th:
                thermal_th=np.maximum(maxT1,maxT2)+c_thermal_change

def find_bed_level(depth_array):
    global deltaAngleX,x1,x2,y1,y2,c_sensorAngle
    
    global c_yLeave,detectZone
    global c_auto_detect_bed
    global paras_level,xyArray
    
    
    c_floor_level,c_bed_level=paras_level

    if c_auto_detect_bed==1:
        
        y1,y2=findEdge(depth_array)
        
        xyArray=x1,y1,x2,y2
        detectZone["x1"]=x1
        detectZone["x2"]=x2
        detectZone["y1"]=y1
        detectZone["y2"]=y2
        setZone(detectZone)
       
        print('y1,y2:',y1,y2)

    cy=y1+int((y2-y1)/2)
    print('cy:',cy)
    x=range(x1,x2,10)
    arrX=[]
    count=0
    dist=0
    
    for n in x:
        angle=round(c_sensorAngle-deltaAngleX*n) 
        
        dd=int(round(depth_array[cy,n])*math.cos(angle*math.pi/180))
        if dd>c_calc_bed_min_level:
            dist=dist+dd
            count+=1
    if count>0: 
        c_bed_level=round(dist/count)

 
    
    
    paras_level  =c_floor_level,c_bed_level
    return paras_level

def findEdge(depth_array):
    global c_yLeave,bed_width
    edgeY1,edgeY2=-1,-1
    at=0
    c_check_front_side,c_check_back_side,c_check_left_side,c_check_right_side,c_front_margin,c_back_margin,c_left_margin,c_right_margin,c_bed_head_pos,c_detect_side=MARGIN
    print('margin:',MARGIN)
    cx=x1+int((x2-x1)/2)
    step=10
    if c_detect_side==cn.c_detect_bothside:
        print('bothside')
        y=range(y1-c_yLeave,y2+c_yLeave,step)
    elif c_detect_side==cn.c_detect_leftside:
        print('leftside')
        for y in range(y2,int(480/4),-step):
            if int(depth_array[y-step,cx]) >c_calc_bed_min_level and int(depth_array[y+step,cx]) >c_calc_bed_min_level: 
            
                if at==0 and edgeY1!=-99:
                    #print('depth diff:',int(depth_array[y-step,cx])-int(depth_array[y+step,cx]))
                    if abs(int(depth_array[y-step,cx]) -int(depth_array[y+step,cx]))>300:    
                        
                        edgeY1=y
                        at=1
            else:
                edgeY1==-99
        
        if edgeY1==-1 or edgeY1==-99:
            edgeY1=y1
            edgeY2=y2
        else:
            
            edgeY2=edgeY1+bed_width
            


 
            
    else:
        print('rightside')
        y=range(y1,y2+c_yLeave,step)
   
    if edgeY2>479-c_yLeave:
        edgeY2=479-c_yLeave
    return edgeY1,edgeY2



counter=0
imageCount=0


static_back = None
label_image=None
bSendZone=False
readSchedule()
blink()
def run():
    global xyArray
    global c_bed_motion_detect_count,bed_motion_detect_count
    global static_back,label_image,bToggle,bSave,bMotion,counter,sleep_count
    global clientState,bSleepLocked,c_scale,bSitLocked
    global trackbar_val,c_bRotateImage
    global x1,y1,x2,y2,cx,cy,a1,b1,a2,b2,ccx,ccy,cursor_x,cursor_y
    global bBlink,bMonitor,c_predict
    global pointOffsetArr,pointBedsideOffset,bRemoveBedsideOnly
    global prevState,bSendZone,imageCount
    global c_bSaveImage,c_bSaveDepth
    global depth_baseline
  
 
    global prcoessed_depth_colormap,predictArray,depth_colormap
    global weightModel,conf_thres
    global c_save_mode
    global bLyingPredicted,bSitPredicted,bPredicted
    global precdictedState,bOffbedLocked
    global bMouseDown
    global tcx,tcy,tx1,ty1,tx2,ty2,rx1,ry1,rx2,ry2,hh,ww
    global lyingCount,c_lying_level,c_sit_level
    global maxT1,maxT2,thermal_th,thermal_adjust_count
    global markedImage
    global c_bed_level
    global deltaAngleX
    global no_motion_count
    global paras_level,c_offset
    global bPassby,bOtherPerson
    global pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow
    global c_other_bedend_offset
    global bWithinTime,bMaualOn,bManualOff
    global captureCount
    global prevDepth
    global MARGIN
    c_lying_offset,c_sit_offset,c_bedside_offset,c_other_offset,c_fall_offset=c_offset
    txtColor=cn.TEXT_COLOR_WHITE
    lineColor=cn.LINE_COLOR_WHITE
    bedLineColor=cn.LINE_COLOR_BLUE
    LeaveLineColor=cn.LINE_COLOR_GREEN
    detected_class='none'

    if c_bSaveDepth==1 or c_bSaveImage==1:
        createSaveDir(imageDir)
    TFLITE_MODEL_PATH = "depth.tflite" #@param {type:"string"}
    # Load the TFLite model
    options = detector.ObjectDetectorOptions(
          num_threads=4,
          score_threshold=conf_thres,
    )

    if c_predict==1:
        thread1 = predictThread(1, "prediction", 1)
        thread1.start()
    

    while True:
        #try:
        
                    # Wait for a coherent pair of frames: depth and color
            start = timer()
            
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame :
                print('no frame')
                continue
            if clientState==cn.c_station_state_power_off:
                print('power off')
                continue
            
            # Convert images to numpy arrays
            depth_array = np.asanyarray(depth_frame.get_data())
            if c_bRotateImage==1:
                depth_array = np.rot90(depth_array,2)
            # depth_array=np.flip(depth_array,0)
            # depth_array=np.flip(depth_array,1)
            grey_color = 153
            bg_removed = np.where((depth_array > clipping_distance) | (depth_array <= 0), clipping_distance, depth_array)
            
            if depth_baseline is None:
                print('get  baseline')
                depth_baseline = bg_removed
                continue
            if prevDepth is None:
                prevDepth=bg_removed
        
            
  
            bBedsideMotion,bBedMotion,bMotion,bedsideDiffPoint,bedDiffPoint=compareDepth(prevDepth,bg_removed,MARGIN,c_yLeave,xyArray,c_diff_th,c_diff_point_motion_th)
            if bBedsideMotion:
                bed_motion_detect_count=0
            # try:
            #     bBedsideMotion,bBedMotion,bMotion,bedsideDiffPoint,bedDiffPoint=compareDepth(prevDepth,bg_removed,MARGIN,c_yLeave,xyArray,c_diff_th,c_diff_point_motion_th)
            # except:
            #     print('compare error')
            #     pass
            # print('bedside,on bed diff:',bedsideDiffPoint,bedDiffPoint)
            # print('bedside,on bed motion:',bBedsideMotion,bBedMotion)
            if bedDiffPoint>c_diff_point_motion_th or bedsideDiffPoint>c_diff_point_motion_th:
                prevDepth=bg_removed
                
            diffArray = np.subtract(depth_baseline,bg_removed)
            diffArray = np.where((diffArray <0) | (diffArray>clipping_distance) , 0,diffArray)
            difference_color = cv2.applyColorMap(cv2.convertScaleAbs(diffArray, alpha=0.03), cv2.COLORMAP_JET)
            bg_removed_color = cv2.applyColorMap(cv2.convertScaleAbs(bg_removed, alpha=0.03), cv2.COLORMAP_JET)

            prcoessed_depth_colormap = colored_map(depth_frame)
            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_array, alpha=0.03), cv2.COLORMAP_JET)
            if c_bRotateImage==1:
                prcoessed_depth_colormap = cv2.rotate(prcoessed_depth_colormap, cv2.cv2.ROTATE_180)
            
            depth_gray=cv2.convertScaleAbs(depth_array, alpha=0.08)
            gray = cv2.GaussianBlur(depth_gray, (21, 21), 0)
            if static_back is None:
                print('get gray')
                static_back = gray
                
                continue
            label_image=gray
            
            images= prcoessed_depth_colormap.copy()
            #bBedsideMotion,bBedMotion,bMotion=detectMotion(images,gray,clientState)
            if bBedMotion:
                
                bed_motion_detect_count=bed_motion_detect_count+1
            if bMotion:
                no_motion_count=0

            prcoessed_depth_colormap=cv2.resize(prcoessed_depth_colormap, dsize=(320, 240), interpolation=cv2.INTER_CUBIC)
            
            points = pc.calculate(depth_frame)

            # Pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()

            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
            c_check_front_side,c_check_back_side,c_check_left_side,c_check_right_side,c_front_margin,c_back_margin,c_left_margin,c_right_margin,c_bed_head_pos,c_detect_side=MARGIN

            resultPointArr,pointOffset,pointBedsideOffset1,maxBedDist1=funcAngle(MARGIN,c_other_bedend_offset,c_offset,depth_array,diffArray,verts,c_bed_level,c_pointThreshold,xyArray,c_yLeave,c_bed_head_line,paras_level,c_sensorAngle,deltaAngleX,pointOffsetArr,pointBedsideOffset,minPointArr)
            
            angle=round(c_sensorAngle-deltaAngleX*cursor_x)
            # distAtCursor=round(int(depth_array[cursor_y,cursor_x])*math.cos(angle*math.pi/180))
            distAtCursor=round(int(verts[cursor_y*640+cursor_x][2]*1000)*math.cos(angle*math.pi/180))

            #verts[j*640+i][2]*1000
            if no_motion_count>c_no_motion_count:
                no_motion_count=0
                if not bSleepLocked:

                    paras_level=find_bed_level(depth_array)
                    
                    c_floor_level,c_bed_level=paras_level
                    c_lying_level=c_bed_level-c_lying_offset
                    c_sit_level=c_bed_level-c_sit_offset
                
                    print('paras_level:',paras_level)

            if bRemoveBedsideOnly:
                pointBedsideOffset=pointBedsideOffset1
                bRemoveBedsideOnly=False
                print('reset bedside offset :',pointBedsideOffset)
            if bGetPointOffset:

                
                pointBedsideOffset=pointBedsideOffset1
                
                pointLying, pointAboveSit,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow=pointOffset
                pointOffsetArr=pointOffset
                maxBedDist=maxBedDist1

                    
                counter=counter+1
                    
                    
            
            pointOtherBedside,pointOtherBedend,pointBed ,pointBedHead,pointBedEnd,pointLying,pointAboveSit,pointBedside,pointBedsideLow,pointBedsideRight,pointBedsideLeft,pointBedsideRightLow,pointBedsideLeftLow,xSitCoor,ySitCoor,xRightCoor,yRightCoor,xLeftCoor,yLeftCoor=resultPointArr
            #print("process time:",str( round(timer()-start,2)))

        
            
            

            if not bMonitor:
                
                clientState=cn.c_station_state_monitor_stop
                bAlert,bNeedAlarm=False,False
                bSave=False
                bSitLocked=False
                bLyingLocked=False
                bSleepLocked=False
                bOffbedLocked=False

        
            detected_class='none'
            if bPredicted and bMonitor:
                detected_class,score,start_point,end_point,color,bounding_box=predictArray
                predictedImage=cv2.rectangle(images, start_point, end_point, color,4 )
                putText(images,detected_class+"-"+str(score), start_point,  color)
                #cv2.imshow('predictedImage', predictedImage) putText(label_image,'Start point:'+str(start_point), cn.org[7],txtColor)
                #putText(label_image,'Start point:'+str(start_point), cn.org[6],txtColor)
                #putText(label_image,'End point:'+str(end_point), cn.org[7],txtColor)
                centerX,centerY=center_point(bounding_box)
                #putText(label_image,'center:'+str(centerX)+','+str(centerY), cn.org[8],txtColor)
                putText(label_image,'Predict:'+detected_class, cn.org[9],txtColor)
            elif c_predict==1:
                putText(label_image,'Predict:', cn.org[9],txtColor)

            clientState=determineState(resultPointArr)    
            if clientState==cn.c_station_state_bed_only or clientState==cn.c_station_state_monitor_stop:
                adjustThermalThreshold(bMotion)  
            
            if clientState != prevState :
                current_date=datetime.now().strftime('%Y-%m-%d')
                current_time=datetime.now().strftime('%H-%M-%S')

                print('clientState:',clientState)

                try:
                    
                    
                    bKeepAlive=False
                    emitClientState()
                except:
                    pass
            
            normalDist=round(depth_array[cursor_y,cursor_x])
            
            cgDist=depth_array[int(ySitCoor),int(xSitCoor)]
            
            images=cv2.circle(images,(cursor_x,cursor_y),5,lineColor,2)
            
            x1,y1,x2,y2=xyArray
            if c_detect_side==cn.c_detect_bothside:#both
                images=cv2.rectangle(images,(x1,y1-c_yLeave),(x2,y2+c_yLeave),LeaveLineColor,2)
                if c_check_front_side:
                    images=cv2.line(images,(x1,y1-c_yLeave-c_front_margin),(x2,y1-c_yLeave-c_front_margin),lineColor,2)
                if c_check_left_side:
                    images=cv2.line(images,(x1-c_left_margin,y1-c_yLeave-c_front_margin),(x1-c_left_margin,y2+c_yLeave+c_back_margin),lineColor,2)
                if c_check_right_side:
                    images=cv2.line(images,(x2+c_right_margin,y1-c_yLeave-c_front_margin),(x2+c_right_margin,y2+c_yLeave+c_back_margin),lineColor,2)
                if c_check_back_side:
                    images=cv2.line(images,(x1,y2+c_yLeave+c_back_margin),(x2,y2+c_yLeave+c_back_margin),lineColor,2)
            elif c_detect_side==cn.c_detect_leftside: #left
                images=cv2.rectangle(images,(x1,y1-c_yLeave),(x2,y2),LeaveLineColor,2) 
                images=cv2.line(images,(x1,y1-c_yLeave-c_front_margin),(x2,y1-c_yLeave-c_front_margin),lineColor,2)
                if c_check_front_side:
                    images=cv2.line(images,(x1,y1-c_yLeave-c_front_margin),(x2,y1-c_yLeave-c_front_margin),lineColor,2)
                if c_check_left_side:
                    images=cv2.line(images,(x1-c_left_margin,y1-c_yLeave-c_front_margin),(x1-c_left_margin,y2+c_back_margin),lineColor,2)
                if c_check_right_side:
                    images=cv2.line(images,(x2+c_right_margin,y1-c_yLeave-c_front_margin),(x2+c_right_margin,y2+c_back_margin),lineColor,2)
                if c_check_back_side:
                    images=cv2.line(images,(x1,y2+c_back_margin),(x2,y2+c_back_margin),lineColor,2)
            else:#right
                images=cv2.rectangle(images,(x1,y2),(x2,y2+c_yLeave),LeaveLineColor,2) 
                images=cv2.line(images,(x1,y2+c_yLeave+c_back_margin),(x2,y2+c_yLeave+c_back_margin),lineColor,2)
                if c_check_front_side:
                    images=cv2.line(images,(x1,y1-c_front_margin),(x2,y1-c_front_margin),lineColor,2)
                if c_check_left_side:
                    images=cv2.line(images,(x1-c_left_margin,y1-c_front_margin),(x1-c_left_margin,y2+c_yLeave+c_back_margin),lineColor,2)
                if c_check_right_side:
                    images=cv2.line(images,(x2+c_right_margin,y1-c_front_margin),(x2+c_right_margin,y2+c_yLeave+c_back_margin),lineColor,2)
                if c_check_back_side:
                    images=cv2.line(images,(x1,y2+c_yLeave+c_back_margin),(x2,y2+c_yLeave+c_back_margin),lineColor,2)
            
            images=cv2.rectangle(images,(x1,y1),(x2,y2),bedLineColor,2)
            '''
            images = cv2.rectangle(images,(x1,y1),(x2,y2),(0,255,0),3)
            images = cv2.rectangle(images,(x1,y1-c_yLeave),(x2,y1),(0,255,0),3)
            images = cv2.rectangle(images,(x1,y2+c_yLeave),(x2,y2),(0,255,0),3)
            '''
            if bSendZone:
                bSendZone=False
                encoded_string=depth2base64(prcoessed_depth_colormap)
                '''
                try:
                    sio.emit('clientImage message', {'From':'client',
                                                        'StationNo':clientID,
                                                        'Img':encoded_string

                                                        })
                except:
                    pass
                '''
            # Show images
            putText(label_image,'image count:'+ str(imageCount), cn.org[0], txtColor)
            putText(label_image,'angle-cursor:'+str(c_sensorAngle)+"-" +str(cursor_x)+":"+str(cursor_y)+"-->"+str(normalDist), cn.org[1], txtColor)
            
            putText(label_image, 'bed,lying,sit level:'+str(c_bed_level)+':'+str(c_lying_level)+':'+str(c_sit_level), cn.org[2], txtColor)
            putText(label_image, 'bedside pt:'+'('+str(c_pointBedside)+')'+str(pointBedsideLeft)+':'+str(pointBedsideRight), cn.org[3],txtColor)
            putText(label_image,'lower bedside pt:'+'('+str(c_pointBedsideLow)+')'+str(pointBedsideLeftLow)+':'+str(pointBedsideRightLow), cn.org[4],txtColor)
            putText(label_image, 'sit pt'+'('+str(c_pointAboveSit)+')'+':'+str(pointBedEnd)+':'+str(pointBedHead), cn.org[5],txtColor)
            putText(label_image, 'lying pt'+'('+str(c_pointLying)+'):'+str(pointLying),cn.org[6],txtColor)
            putText(label_image,'bed pt:'+str(pointBed), cn.org[7], txtColor)
            # putText(label_image,'CG:'+str(xSitCoor)+':'+str(ySitCoor), cn.org[8], txtColor)
            putText(label_image,'other pt:'+'('+str(c_pointOtherSide)+':'+str(c_pointOtherBedend)+')'+str(pointOtherBedside)+':'+str(pointOtherBedend), cn.org[8], txtColor)

            

            putText(label_image,'State:'+stateMsg[clientState], cn.org[10],txtColor)
            if bBreathSenorError:
                putText(label_image,'Breath sensor error!:', cn.org[11], txtColor)

            else:
                putText(label_image,'Breath:'+breathMsg+'('+str(detectOptions["UseBreathSensorDetectPresence"] )+')', cn.org[11], txtColor)
            if clientState==cn.c_station_state_lying or clientState==cn.c_station_state_bed_only:
                putText(label_image,'motion detected'+'('+str(c_no_motion_count)+')'+'('+str(detectOptions["MotionSleepLock"] )+')'+str(bed_motion_detect_count), cn.org[12], txtColor)
            else:
                putText(label_image,'no motion:'+str(no_motion_count), cn.org[12], txtColor)

            if not bMqttConnected:
                putText(label_image,'mqtt connection fail', cn.org[13], txtColor)
            elif not bServerConnected:
                putText(label_image,'server connection fail', cn.org[13], txtColor)
            elif bPassby:
                putText(label_image,"other passby", cn.org[13], txtColor)
            else:
                putText(label_image,str(lyingCount), cn.org[13], txtColor)
            #putText(label_image,"process time:"+str( round(timer()-start,2)) ,cn.org[14], txtColor)
            putText(label_image,"process time:"+str( round(timer()-start,2)) ,cn.org[14], txtColor)
            
            if c_use_thermal_detect==1:
                putText(label_image,str(thermal_th)+":"+str(thermal_adjust_count)+"-->"+str(maxT1)+":"+str(maxT2), cn.org[15], txtColor)
            
            if bMonitor:
                if c_save_mode==cn.c_SLEEP_MOTION:
                    if bMotion and clientState==cn.c_station_state_sleep_locked:
                        if c_save_image_type== cn.c_SAVE_COLOR:
                            saveData(prcoessed_depth_colormap)
                        else:
                            saveData(label_image)
                elif c_save_mode==cn.c_STATE_CHANGE:
                    if prevState!=clientState:
                        if c_save_image_type== cn.c_SAVE_COLOR:
                            saveData(prcoessed_depth_colormap)
                        else:
                            saveData(label_image)
                elif c_save_mode==cn.c_STATE_CHANGE_SLEEP_MOTION:
                    if bMotion and clientState==cn.c_station_state_sleep_locked:
                        saveData(prcoessed_depth_colormap)
                    if prevState!=clientState:
                        if c_save_image_type== cn.c_SAVE_COLOR:
                            saveData(prcoessed_depth_colormap)
                        else:
                            saveData(label_image)
                    
                elif bSave and (c_bSaveImage or c_bSaveDepth):
                    if c_save_mode==cn.c_MOTION_SAVE:
                        if bMotion:
                            saveData(prcoessed_depth_colormap)
                    elif  c_save_mode==cn.c_ALWAYS_SAVE:
                        saveData(prcoessed_depth_colormap)
                    else:
                        if prevState!=clientState:
                            saveData(prcoessed_depth_colormap)
                    bMotion=False    
                    bSave=False
            prevState=clientState    
            markedImage=images
            bError=False
            cv2.imshow('colorMap', images)
            cv2.imshow('label_image', label_image)
    #             cv2.imshow('bg_removed_color', bg_removed_color)
    #             cv2.imshow('difference_color', difference_color)


            # cv2.imshow('masked_depth_frame_data_1d', masked_depth_frame_data_1d)
            
            cv2.waitKey(1)
        # except:
        #     pass
    bError=True
    mqttClient.stop()

if __name__ == "__main__":
    
    
    run()




#options: {'From': 'nurse', 'To': 'client', 'StationNo': '2', 'ReStartAfterStopDelay': '1', 'FramePerSecond': 1, 'NoOffDayToKeep': 7, 'Backup': False, 'LogData': True, 'LogDataWhenMotionDetected': True, 'DepthFrame': False, 'LogGrayScaleFrame': True, 'Breath': True, 'UseBreathSensorDetectPresence': True, 'MotionSleepLock': False, 'RealtimeBackup': False, 'MarkImage': True}
#detectMargin: {'From': 'nurse', 'To': 'client', 'StationNo': '2', 'ChkFront': True, 'ChkBack': False, 'FrontMargin': '20', 'LeftMargin': '20', 'RightMargin': '20', 'BackMargin': '20', 'ChkLeftSide': True, 'ChkRightSide': False, 'BedHeadPos': '1', 'DetectBedside': '0'}
# setZone:  {'From': 'nurse', 'To': 'client', 'StationNo': '2', 'Cmd': 'c_station_msg_region', 'x1': 128, 'y1': 104, 'x2': 376, 'y2': 317}
