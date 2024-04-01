import pyrealsense2.pyrealsense2 as rs
class Myrealsense(object):

    # do test here?
    def getCamMetadata(self):
        return self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
    
    def __init__(self): 
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        #self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # Declare filters
        dec_filter = rs.decimation_filter()   # Decimation - reduces depth frame density
        spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
        temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise
        #print(dir(dec_filter))
        
    def __enter__(self): 
        self.pipeline.start(self.config)
        print(self.getCamMetadata())
        '''
        for i in range(200):
            frames = self.pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            
            if not depth:
                i = i + 1
                continue
            else:
                processed_depth = post_process_depth_frame(depth)
                
                showarray(np.array(processed_depth.as_frame().get_data()))

        '''
        return self.pipeline
  
    def __exit__(self,a,b,c): 
        self.pipeline.stop()
        
if __name__ == "__main__":
    from Visualiser import *
    import time
    colorizer = rs.colorizer()
    with Visualiser() as viewport:
        with Myrealsense() as cam:
            try:
                while 1:
                    # Render
                    now = time.time()
                    frames = cam.wait_for_frames()
                    depth = frames.get_depth_frame()
                    if not depth:
                        continue
                    raw_colorized_depth = colorizer.colorize(depth)
                    raw_depth_colormap = np.asanyarray(raw_colorized_depth.get_data())
                    #detection_depth_colormap = np.array(prcoessed_depth_colormap)
                    dt = time.time() - now
                    viewport.cvshow(raw_depth_colormap,"RealSense %dFPS (%.2fms)" %(1.0/dt, dt*1000))

                
            finally:
                pass
