import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time
def post_process_depth_frame(depth_frame, decimation_magnitude=1.0, spatial_magnitude=2.0, spatial_smooth_alpha=0.5,
                         spatial_smooth_delta=20, temporal_smooth_alpha=0.4, temporal_smooth_delta=20):
    """
    Filter the depth frame acquired using the Intel RealSense device
    Parameters:
    -----------
    depth_frame          : rs.frame()
                           The depth frame to be post-processed
    decimation_magnitude : double
                           The magnitude of the decimation filter
    spatial_magnitude    : double
                           The magnitude of the spatial filter
    spatial_smooth_alpha : double
                           The alpha value for spatial filter based smoothening
    spatial_smooth_delta : double
                           The delta value for spatial filter based smoothening
    temporal_smooth_alpha: double
                           The alpha value for temporal filter based smoothening
    temporal_smooth_delta: double
                           The delta value for temporal filter based smoothening
    Return:
    ----------
    filtered_frame : rs.frame()
                     The post-processed depth frame
    """

    filters = [rs.disparity_transform(),
           rs.spatial_filter(),
           rs.temporal_filter(),
           rs.disparity_transform(False)]

    # Post processing possible only on the depth_frame
    assert (depth_frame.is_depth_frame())

    # Available filters and control options for the filters
    decimation_filter = rs.decimation_filter()
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter(0.2,50,1)
    hole_filling_filter = rs.hole_filling_filter()
    disparity_transform_on = rs.disparity_transform()
    disparity_transform_off = rs.disparity_transform(False)

    filter_magnitude = rs.option.filter_magnitude
    filter_smooth_alpha = rs.option.filter_smooth_alpha
    filter_smooth_delta = rs.option.filter_smooth_delta

    # Apply the control parameters for the filter
    decimation_filter.set_option(filter_magnitude, decimation_magnitude)
    spatial_filter.set_option(filter_magnitude, 5)
    #spatial_filter.set_option(filter_smooth_alpha, spatial_smooth_alpha)
    spatial_filter.set_option(filter_smooth_alpha, 0.25)
    spatial_filter.set_option(filter_smooth_delta, spatial_smooth_delta)
    #temporal_filter.set_option(filter_smooth_alpha, temporal_smooth_alpha)
    temporal_filter.set_option(filter_smooth_alpha, 0.2)

    #temporal_filter.set_option(filter_smooth_delta, temporal_smooth_delta)
    temporal_filter.set_option(filter_smooth_delta, 50)
    #temporal_filter.set_option()

    # Apply the filters
    filtered_frame = disparity_transform_on.process(depth_frame)
    filtered_frame = decimation_filter.process(filtered_frame)
    filtered_frame = spatial_filter.process(filtered_frame)
    filtered_frame = temporal_filter.process(filtered_frame)
    filtered_frame = hole_filling_filter.process(filtered_frame)
    filtered_frame = disparity_transform_off.process(filtered_frame)
    return filtered_frame

COUNTER = 0;
AVG = 0;
colorizer = rs.colorizer()

def getBaseline_1():
    with Myrealsense(rotation = cv2.ROTATE_90_CLOCKWISE) as cam:
        buf_baseline = cam.get_depth_frame().get_depth_data()
        timeElased = 0
        now = time.time()

        frameCount = 1
        while timeElased<10:
            if timeElased > 4:
                buf_baseline = buf_baseline + cam.get_depth_frame().get_depth_data()
                frameCount = frameCount + 1
            timeElased = time.time() - now
        norm = buf_baseline/frameCount
        return norm
def getBaseline(depth_frame,clipping_distance,depth_scale):


    depth_baseline = depth_frame.get_data()
    
    timeElased = 0
    now = time.time()
    frameCount = 1
    while timeElased<10:
        if timeElased > 4:
            processed_depth = post_process_depth_frame(depth_frame)
            depth_baseline = depth_baseline + np.array(processed_depth.as_frame().get_data())
            frameCount = frameCount + 1
        timeElased = time.time() - now
    norm = depth_baseline/frameCount
    return depth_baseline

def getBaseline_0(depth_frame,clipping_distance,depth_scale):

    processed_depth = post_process_depth_frame(depth_frame)
    processed_depth_frame_data = np.array(processed_depth.as_frame().get_data())
    depth_baseline = processed_depth_frame_data
    depth_image_1d = np.array(depth_baseline)
    bg_removed_1d = np.where((depth_image_1d > clipping_distance) | (depth_image_1d <= 0), clipping_distance, depth_image_1d)
    bg_removed_1d = clipping_distance - bg_removed_1d
    # bg_removed_1d=bg_removed_1d + 0.05/depth_scale
    #colormap = cv2.applyColorMap(cv2.convertScaleAbs(bg_removed_1d, alpha=0.03), cv2.COLORMAP_JET)
    #cv2.imshow('base line colormap', colormap)
    # depth_baseline = bg_removed_1d + 0.05/depth_scale
    return bg_removed_1d 


def background_removal_1(bg_removed_1d, depth_baseline, thershold_distance):
    # compute difference
    difference = cv2.subtract(bg_removed_1d,depth_baseline)
    # compute mask
    mask_1d = np.where((difference > thershold_distance), 1, 0)


    #masking input and cleanup
    kernel = np.ones((9,9),np.uint8)
    masked_depth_frame_data_1d = np.multiply(bg_removed_1d,mask_1d)
    masked_depth_frame_data_1d = cv2.morphologyEx(masked_depth_frame_data_1d, cv2.MORPH_OPEN, kernel)

    return mask_1d, masked_depth_frame_data_1d

def colored_map(rsdepthFrame):
    global COUNTER, AVG

    #colorized raw depth
    rsRaw_colorized_depth = colorizer.colorize(rsdepthFrame)
    #to np array
    raw_depth_colormap = np.asanyarray(rsRaw_colorized_depth.get_data())

    #pass through filter
    rsProcessed_depth = post_process_depth_frame(rsdepthFrame)
    #to np array
    npProcessed_depth_frame_data = np.array(rsProcessed_depth.as_frame().get_data())

    #colorized processed depth
    prcoessed_colorized_depth = colorizer.colorize(rsProcessed_depth)
    #to np array
    prcoessed_depth_colormap = np.asanyarray(prcoessed_colorized_depth.get_data())


    return  prcoessed_depth_colormap
#======================================================================================
def background_removal(rsdepthFrame,depth_baseline,clipping_distance, thershold_distance):
    global COUNTER, AVG

    #colorized raw depth
    rsRaw_colorized_depth = colorizer.colorize(rsdepthFrame)
    #to np array
    raw_depth_colormap = np.asanyarray(rsRaw_colorized_depth.get_data())

    #pass through filter
    rsProcessed_depth = post_process_depth_frame(rsdepthFrame)
    #to np array
    npProcessed_depth_frame_data = np.array(rsProcessed_depth.as_frame().get_data())

    #colorized processed depth
    prcoessed_colorized_depth = colorizer.colorize(rsProcessed_depth)
    #to np array
    prcoessed_depth_colormap = np.asanyarray(prcoessed_colorized_depth.get_data())


    #copy np arrray
    depth_image_1d = np.array(npProcessed_depth_frame_data)
    #clip all pixels beyond clipping distance
    bg_removed_1d = np.where((depth_image_1d > clipping_distance) | (depth_image_1d <= 0), clipping_distance, depth_image_1d)
    #change 0 ref from camera to clipping distance
    bg_removed_1d = clipping_distance - bg_removed_1d
    #global depth_baseline

    # if depth_baseline is None:
    #     depth_baseline = bg_removed_1d + 0.05/depth_scale
    #     #depth_baseline = getBaseline() #+ /depth_scale
    #     #print(depth_baseline)
    #     return (None,None,None,None,None)


    # compute difference
    difference = np.subtract(bg_removed_1d,depth_baseline)
    # compute mask
    mask_1d = np.where((difference > thershold_distance), 1, 0)


    #masking input and cleanup
    kernel = np.ones((9,9),np.uint8)
    masked_depth_frame_data_1d = np.multiply(clipping_distance - np.array(npProcessed_depth_frame_data),mask_1d)
    masked_depth_frame_data_1d = cv2.morphologyEx(masked_depth_frame_data_1d, cv2.MORPH_OPEN, kernel)

    return (bg_removed_1d, mask_1d, raw_depth_colormap, masked_depth_frame_data_1d, prcoessed_depth_colormap)
#======================================================================================
