import json
import os
import os.path
import time
from timeloop import Timeloop
from datetime import timedelta
from datetime import datetime

bMaualOn,bManualOff=False,False
def readSchedule():
    
    if os.path.isfile('powerSchedule.json'):
        with open('powerSchedule.json', 'r') as openfile:

            powerSchedule = json.load(openfile)
        
        #print('powerSchedule:',powerSchedule)
        return powerSchedule

    else:
        print('schedule file not found')
        return None
        
def compareTime(powerSchedule):
    
    
    current_date=datetime.now().strftime('%Y-%m-%d')
    current_time=datetime.now().strftime('%H:%M')
    print('current time:',current_date)
    print('current time:',current_time)

    print('start time/end time:',powerSchedule["OnTime"],powerSchedule["OffTime"])


    if powerSchedule["OnTime"]<powerSchedule["OffTime"]:
        if current_time>powerSchedule["OnTime"] and current_time<powerSchedule["OffTime"]:
            # print('within time')
            
            bWithTime=True
        else:
            # print('out of time')
            
            bWithTime=False
    else:
        if current_time>current_time>powerSchedule["OnTime"] and current_time<'23:59' or current_time<powerSchedule["OffTime"]:
            # print('within time')
            
            bWithTime=True
        else:
            # print('out of time')
            
            bWithTime=False
    return  bWithTime
        
    
# powerSchedule=readSchedule()
# compareTime(powerSchedule)




