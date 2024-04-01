#ls -l /dev/tty*
import sys, json as np
from datetime import datetime
from pymoduleconnector import ModuleConnector
from pymoduleconnector.ids import *
from time import sleep
import random
import platform
import json,yaml

hostName=platform.node()
print('hostname:',hostName)
ww=hostName.split('-')
clientID=str(int(ww[1])-1)
print('clientId:',clientID)

detection_zone=(1.4,1.8)
sensitivity=5
detzone_start = detection_zone[0]
detzone_end = detection_zone[1]

mc = ModuleConnector("/dev/ttyACM0", log_level=0)
sensor = mc.get_x4m200()
sleep(1) # Allow for MC to read waiting messages from module.

'''
('FirmwareID:', 'Annapurna')
('Version:', '1.4.7')
('Build:', '1.4.7+0.sha.4a097bfba7b4032aa30ee5b0f4cd7820d939e6e6')
('Serial number:', '100000039811')
Setting user settings: DetectionZone = 1.4 to 1.8, Sensitivity = 5
'''
try:
    sensor.set_sensor_mode(XTID_SM_STOP, 0) # Make sure no profile is running.
    print("Stopped already running profile.")
except RuntimeError:
        print("If not initialized, stop returns error. Still OK, just wanted to make sure the profile was not running.")
pass
 # Now flush old messages from module
print("Flushing any old data.")
while sensor.peek_message_respiration_sleep():
    rdata= sensor.read_message_respiration_sleep()
    print(rdata)
# Read module info
try:
    print("device info:")
    print("FirmwareID:", sensor.get_system_info(XTID_SSIC_FIRMWAREID))
    print("Version:", sensor.get_system_info(XTID_SSIC_VERSION))
    print("Build:", sensor.get_system_info(XTID_SSIC_BUILD))
    print("Serial number:", sensor.get_system_info(XTID_SSIC_SERIALNUMBER))
except RuntimeError:
    print("time out Firmware read")
pass

print("Setting user settings: DetectionZone = " + str(detzone_start) + " to " + str(detzone_end) + ", Sensitivity = " + str(sensitivity))
sensor.load_profile(XTS_ID_APP_RESPIRATION_2 )
sensor.set_sensitivity(sensitivity)
sensor.set_detection_zone(detzone_start, detzone_end)


sensor.set_sensor_mode(XTID_SM_RUN ,0)

prevState=-1
prevBPM=-1
counter=0
frameCount=0;
while 1:
    if sensor.peek_message_respiration_sleep():
        rdata = sensor.read_message_respiration_sleep()
        current_time=datetime.now().strftime('%H:%M:%S')
        counter=counter-1
        
        if counter<0:
            
            frameCount=frameCount+1
            if frameCount > 10000:
                frameCount=0
            counter=random.randint(5,10)
            
            
            str1='{"BreathCounter":'+str(frameCount)+',"StationNo":'+clientID+',"BreathState":'+str(rdata.sensor_state)+',"BreathBpm":'+str(round(rdata.respiration_rate))+',"BreathDistance":'+str(round(rdata.distance,1))+'}'
            
            str1=yaml.safe_load(str1)
            
            print(str1)
         
            sys.stdout.flush()
        
        '''
        if prevState!=rdata.sensor_state or prevBPM!=rdata.respiration_rate:
            prevState=rdata.sensor_state
            prevBPM=rdata.respiration_rate
            
            #print("BPM"+"&"+str(rdata.sensor_state)+"&"+str(rdata.respiration_rate)+"&"+str(rdata.movement_slow)+"&"+str(rdata.movement_fast))
            print('---------------------------')
            str1='{"From":"pi","Time":"'+current_time+',"Station":"'+clientID+'","BreathState":"'+str(rdata.sensor_state)+'","BreathBmp":"'+str(rdata.respiration_rate)+'","BreathDistance":"'+str(rdata.distance)+'"}"'
            print(str1)

            sys.stdout.flush()
        '''
    else:
        pass


