import cv2
import numpy as np
class Visualiser(object):
    def __init__(self):
        self.display_id = None

    def __enter__(self):
        return self

    def __exit__(self,a,b,c):
        print("leaving")
        cv2.destroyAllWindows()
        return
    '''
    def show(self,a,fmt='jpeg',imgMode='RGB'):
        if self.display_id is not None:
            self.__updateDisplay__(a,fmt,imgMode)
        else:
            self.display_id = self.__showarray__(a, fmt,imgMode)

    def __showarray__(self,a, fmt='jpeg',imgMode='RGB'): # show array SIZE:(X,Y,3)
        #a = np.uint8(np.clip(a, 0, 255))
        a = np.uint8(a/np.max(a)*255)
        f = io.BytesIO()
        PIL.Image.fromarray(a,mode=imgMode).save(f, fmt) #.transpose(PIL.Image.ROTATE_270)
        return display(Image(data=f.getvalue()),display_id=True)

    def __updateDisplay__(self,a, fmt='jpeg', imgMode='RGB'):# show array SIZE:(X,Y,3)
        a = np.uint8(np.clip(a, 0, 255))
        f = io.BytesIO()
        PIL.Image.fromarray(a,mode=imgMode).save(f, fmt)#.transpose(PIL.Image.ROTATE_270)
        self.display_id.update(Image(data=f.getvalue()))
    '''
    def cvshow(self,image,title='RealSense',name='RealSense',stay=1):
        image = np.uint8(image/np.max(image)*255)
        # Show images
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow(name, 10,10);
        cv2.imshow(name, image)
        cv2.setWindowTitle(name, title)
        return cv2.waitKey(stay) #hold for 1ms, 0 for infinite display

if __name__ == "__main__":
	print("Unit test [Visualiser.py]")
	import numpy as np
	with Visualiser() as viewport:
		imgbuf = np.ones((300,1000))*255
		while 1:
			viewport.cvshow(imgbuf) #need to put in a loop as "cv2.waitKey(1)"

################################################################
################################################################
'''
Error log:

Problem (20210222):
Gtk-Message: Failed to load module "canberra-gtk-module"

Solution:
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module -y


'''
################################################################
################################################################
