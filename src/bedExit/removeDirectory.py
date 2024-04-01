import shutil
import os
total_days_keep=7
path = '/home/pi-5/Documents/bedExit/images/2023-05-20'
imagePath = '/home/pi-5/Documents/bedExit/images/'
def listDir():
    global dir_list
    dir_list = os.listdir(imagePath)
    
    print("Files and directories in '", imagePath, "' :")
    
    # prints all 
    dir_list.sort()
    print(dir_list)
    print('totalDir:',len(dir_list))

def delDir():
    listDir()
    if len(dir_list)>7:
        try:
            shutil.rmtree(imagePath+dir_list[0])
            print("directory is removed successfully")
        except OSError as x:
            print("Error occured: %s : %s" % (path, x.strerror))



