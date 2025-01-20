import cv2
import threading
from typing import Optional, Union

class ControlPanelItem():
    value: Union[float, int]
    maximum: Optional[Union[float, int]]
    minimum: Optional[Union[float, int]]
    multiplier: Optional[int]
    set_to_int: Optional[bool]

    def __init__(self, value, maximum=255, minimum=0, multiplier=1, set_to_int=False):
        self.value = value
        self.maximum = maximum
        self.minimum = minimum
        self.multiplier = multiplier
        self.set_to_int = set_to_int

def create_control_panel(control_panel_name: str, controls, **kwargs):#: dict[str, ControlPanelItem]):
    cv2.namedWindow(control_panel_name)
    cv2.resizeWindow(control_panel_name, 700, 1500)
    for control in controls.items():
        name, v = control

        # Use default arguments to capture current 'name' value
        def on_change(val, name=name, minimum=v.minimum, multiplier=v.multiplier):
            val = val/multiplier
            if val < minimum:
                val = minimum
            controls[name].value = val if not v.set_to_int else round(val)
        cv2.createTrackbar(
            name, 
            control_panel_name, 
            int(v.value*v.multiplier),
            int(v.maximum*v.multiplier), 
            on_change
        )
    no_new_thread = kwargs.get("no_new_thread") or False
    if not no_new_thread:
        input_thread = threading.Thread(target=cv2.waitKey)
        input_thread.daemon = True
        input_thread.start()
