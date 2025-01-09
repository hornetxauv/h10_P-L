import cv2
import threading

def create_control_panel(control_panel_name, controls):
    cv2.namedWindow(control_panel_name)
    for control in controls.items():
        name, v = control
        default_value = None
        value, maximum, minimum, multiplier = [*v, *([default_value] * (4 - len(v)))] # https://stackoverflow.com/a/59303329/7577786
        maximum = (maximum if maximum else 255)
        minimum = (minimum if minimum != None else 0)
        multiplier = multiplier if multiplier != None else 1
        # Use default arguments to capture current 'name' value
        def on_change(val, name=name, minimum=minimum, multiplier=multiplier):
            val = val/multiplier
            if val < minimum:
                val = minimum
            controls[name][0] = val
        cv2.createTrackbar(
            name, 
            control_panel_name, 
            int(value*multiplier),
            int(maximum*multiplier), 
            on_change
        )
    input_thread = threading.Thread(target=cv2.waitKey)
    input_thread.daemon = True
    input_thread.start()
