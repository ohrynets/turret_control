import cv2
import numpy as np



def calc_optical_flow(original_frame, original_prev_frame, new_width=640, new_height=480):
    frame = cv2.resize(original_frame, (new_width, new_height))
    prev_frame = cv2.resize(original_prev_frame, (new_width, new_height))
    # Convert the frame to grayscale
    #new_frame = cv2.resize(frame, (new_width, new_height))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    # Calculate optical flow using Farneback method
    flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    # Compute the magnitude and angle of the flow
    magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])

    # Create an HSV image
    hsv = np.zeros_like(frame)
    hsv[..., 1] = 255

    # Set the hue according to the optical flow direction
    hsv[..., 0] = angle * 180 / np.pi / 2

    # Set the value according to the optical flow magnitude
    hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)

    # Convert HSV to BGR
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    #bgr = cv2.convertScaleAbs(cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX))
    #_, threshold = cv2.threshold(bgr, 70, 255, cv2.THRESH_BINARY)
    
    return bgr

def init_camera(camera_index=0, reset_width = 1600, reset_height = 1200):
    cap = cv2.VideoCapture(camera_index)  # Use camera_index for webcam or provide video file path
    if not cap.isOpened():
        print("Error: Could not open camera.")
        cap.release()
        cv2.destroyAllWindows()
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, reset_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, reset_height)
    return cap

def init_params(cap):
    # Define the original and resized dimensions
    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    new_width, new_height = 640, 480
    print(f"Original width: {original_width}, Original height: {original_height}")
    # Calculate the scaling factors
    scale_x = original_width / new_width
    scale_y = original_height / new_height
     # Return a dictionary with the camera parameters
    return {
        'cap': cap,
        'original_width': original_width,
        'original_height': original_height,
        'new_width': new_width,
        'new_height': new_height,
        'scale_x': scale_x,
        'scale_y': scale_y
    }

def read_camera(cap):
    ret, original_frame = cap.read()
    if not ret:
        return None
    return original_frame


def calculate_contours(flow, new_width, new_height, scale_x=1.0, scale_y=1.0, score_threshold=0.03, nms_threshold=0.05):
    flow_gray = cv2.cvtColor(flow, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(flow_gray, 25, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    bboxes = []
    scores = []
    norm_area_coeff = 1.0 / (new_width * new_height)
    for contour in contours:
        area = cv2.contourArea(contour)
        score = area * norm_area_coeff
        if score > 0.7:  # skip too large contours
            continue
        contour = contour.astype(np.float32)
        contour[:, :, 0] *= scale_x
        contour[:, :, 1] *= scale_y
        contour = contour.astype(np.int32)
        (x, y, w, h) = cv2.boundingRect(contour)
        bboxes.append([x, y, w, h])
        scores.append(score)
    indices = cv2.dnn.NMSBoxes(bboxes, scores, score_threshold, nms_threshold, eta=1.0, top_k=5)
    if len(indices) > 0:
        filtered_boxes = [bboxes[i[0]] if isinstance(i, (list, tuple, np.ndarray)) else bboxes[i] for i in indices]
    else:
        filtered_boxes = []
    return filtered_boxes

def draw_bounding_boxes(frame, boxes):
    for (x, y, w, h) in boxes:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)\
    
    return frame

def scale_image(frame, original_width, original_height):
    height, width = frame.shape[:2]
    new_frame = frame.copy()
    if height != original_height or width != original_width: 
        new_frame = cv2.resize(frame, (original_width, original_height))
    else:
        new_frame = frame
    return new_frame

def apply_optical_flow_blend(original_frame, original_flow, alpha = 0.8, gamma = 10):
    alpha = 0.8
    beta = 1 - alpha
    gamma = 10
    # Method 2: Using cv2.merge
    original_flow_inverted = (original_flow)
    #rgb_flow = cv2.merge([original_flow_inverted, original_flow_inverted, original_flow_inverted*0])
    #rgb_flow = cv2.cvtColor(original_flow_inverted, cv2.COLOR_RGB2)
    bgr_flow = original_flow 
    #print(f"Original shape:{original_frame.shape} mask:{rgb_flow2.shape}")
    blended = cv2.addWeighted(original_frame, alpha, bgr_flow, beta, gamma=gamma)
    return blended

def draw_optical_flow_countours(frame, flow, draw_boxes=True, new_width=None, new_height=None, scale_x=1.0, scale_y=1.0):
    original_height = frame.shape[0]
    original_width = frame.shape[1]
    scale_new_width = new_width
    scale_new_height =  new_height
    if new_height is None:
        scale_new_height = original_height
        
    if new_width is None:
        scale_new_width = original_width
        
    if flow is not None:
        original_flow = scale_image(flow, original_width, original_height)
        original_frame = scale_image(frame, original_width, original_height)
        if draw_boxes:
            boxes = calculate_contours(flow=original_flow, new_width=scale_new_width,
                                       new_height=scale_new_height, 
                                       scale_x=scale_x, scale_y=scale_y,
                                       score_threshold=0.02, nms_threshold=0.03)
        else:
            boxes = []
        blended_frame = apply_optical_flow_blend(original_frame, original_flow)
        if draw_boxes:
            blended_frame = draw_bounding_boxes(blended_frame, boxes)
        return blended_frame, boxes
    
    return frame, []
    
def run_local():
    # Get the camera parameters as a dictionary
    cap = init_camera()
    camera_params = init_params(cap)
    # Access the parameters from the dictionary
    original_width = camera_params['original_width']
    original_height = camera_params['original_height']
    new_width = camera_params['new_width']
    new_height = camera_params['new_height']
    scale_x = camera_params['scale_x']
    scale_y = camera_params['scale_y']


    prev_frame = read_camera(cap)
    prev_frame = cv2.resize(prev_frame, (new_width, new_height))

    count = 0


    while True:
        original_frame = read_camera(cap)
        if count % 10 != 0:
            continue
        frame = cv2.resize(original_frame, (new_width, new_height))
        
        flow = calc_optical_flow(frame, prev_frame)
        
        blended_frame, boxes = draw_optical_flow_countours(original_frame, flow, new_width, new_height, scale_x, scale_y)
        #print(f"Blended frame shape: {blended_frame.shape}, boxes: {boxes}")
        # boxes = calculate_contours(flow, new_width, new_height, scale_x, scale_y)
        # original_frame = scale_image(original_frame, original_width, original_height)
        original_flow = scale_image(flow, original_width, original_height)
        
        # blended = apply_optical_flow_blend(original_frame, original_flow)
        
        # blended_frame = draw_bounding_boxes(blended, boxes)
        # original_flow = draw_bounding_boxes(flow, boxes)
        
        
        cv2.imshow("Motion Detection", blended_frame)    
        cv2.imshow("Optical Flow", original_flow)
        #cv2.imshow("Blended", blended)
        prev_frame = frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    run_local()
