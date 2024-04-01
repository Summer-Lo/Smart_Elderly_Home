import numpy as np
class DetectedObj:    
    def __init__(self, bbox, label, score, mask, frameReference):
        self.__LABELLIST = ["","Human", "human","toilet","5","6","7","8"] #Human readable Labels
        self.bbox = bbox #bounding box [xStart, yStart, xEnd, yEnd]
        self.label = self.__LABELLIST[label-1] if label <4 else label
        #print("label",label)
        self.score = score
        self.mask = mask
        self.frameReference = frameReference
        
        #Compute area of the instance with bbox start and end pt.
    def getArea(self):
        return (self.bbox[2]-self.bbox[0])*(self.bbox[3]-self.bbox[1])
    
        #
    def getBbox(self):
        return {
            "startPoint":{
                "x":self.bbox[0],
                "y":self.bbox[1]
            },
            "endPoint":{
                "x":self.bbox[2],
                "y":self.bbox[3]
            }
        }
    
    def getCenter(self):
        return {
            "x":(self.bbox[2]+self.bbox[0])/2,
            "y":(self.bbox[3]+self.bbox[1])/2
        }
    
    def getObj(self):
        bbox = list(map(int,self.bbox))
        #print(self.frameReference.frame)
        return self.frameReference.frame[bbox[1]:bbox[3],bbox[0]:bbox[2]]
        # return self.mask[bbox[0]:bbox[2],bbox[1]:bbox[3],:]
        # return self.mask[:,:,:]
        
    def getMaxVal(self):
        
        return np.max(self.getObj())
    
    def getMaxCoor(self):
        thisObj = self.getObj()
        ##patched code
        #print("len:" +str((thisObj.size)))
        
        if thisObj.size != 0:
            local_max_coor = np.unravel_index(thisObj.argmax(), thisObj.shape)
            start_point = tuple(map(int,self.bbox[1::-1]))#+(0,)  
            #print((start_point,local_max_coor))
            world_max_coor= np.add(start_point,local_max_coor)
            # start point + local mask max point
            # where max = linearized max / (col,row) width
            # return world_max_coor
            return {"x":world_max_coor[1],"y":world_max_coor[0]}
        else:
            return {"x":0,"y":0}
        ##=========
    def __str__(self):
        return str({"bbox":self.bbox,"label":self.label, "score":self.score, "mask":self.mask})
        # get mask 
        # get highest
    


class ROIObj(DetectedObj):
    def __init__(self, bbox,label,refFrame=None):
        #bbox size to np.array[mask]
        #[x1,y1,x2,y2]


        score = 1
        bbox = list(map(int,bbox))

        mask = np.zeros((480,848,1))
        mask[bbox[1]:bbox[3],bbox[0]:bbox[2]] = 0  
        if refFrame is None:
            super().__init__(bbox, label, score, mask, SensorFrame())
        else:
            super().__init__(bbox, label, score, mask, refFrame)

    def isInArea(self,obj,mode="CENTER"):
        boundingBox = self.getBbox()
        ptInBox = lambda pt, bbox : ((bbox["startPoint"]["x"] <= pt["x"]) and (pt["x"] <= bbox["endPoint"]["x"]) and (bbox["startPoint"]["y"] <= pt["y"]) and (pt["y"] <= bbox["endPoint"]["y"]))
        modes ={
            "CENTER":ptInBox(obj.getCenter(),boundingBox),
            "MAX":ptInBox(obj.getMaxCoor(),boundingBox),
            "BBOX":ptInBox(obj.getBbox()["startPoint"],boundingBox) and ptInBox(obj.getBbox()["endPoint"],boundingBox)
        }
        return modes[mode]

