import cv2
import threading
from typing import Optional, TypedDict
from operator import itemgetter

class ControlPanelItem(TypedDict):
    value: float | int
    maximum: Optional[float | int]
    minimum: Optional[float | int]
    multiplier: Optional[int]
    set_to_int: Optional[bool]

    # def __init__(self, value, maximum, minimum, multiplier, set_to_int):
    #     print('hi')
    #     self.value = value
    #     self.maximum = maximum or 255
    #     self.minimum = minimum or 0
    #     self.multiplier = multiplier or 1
    #     self.set_to_int = set_to_int or False

# a=ControlPanelItem.__init__(value=1)
# print(a)

def create_control_panel(control_panel_name: str, controls: dict[str, ControlPanelItem]):
    cv2.namedWindow(control_panel_name)
    for control in controls.items():
        name, v = control
        # default_value = None
        # value, maximum, minimum, multiplier, set_to_int = [*v, *([default_value] * (4 - len(v)))] # https://stackoverflow.com/a/59303329/7577786
        # value, maximum, minimum, multiplier, set_to_int = itemgetter('value', 'maximum', 'minimum', 'multiplier', 'set_to_int')(v)
        value, maximum, minimum, multiplier, set_to_int = [v.get(f) for f in ['value', 'maximum', 'minimum', 'multiplier', 'set_to_int']]
        maximum = maximum if maximum else 255
        minimum = minimum if minimum != None else 0
        multiplier = multiplier if multiplier != None else 1
        set_to_int = set_to_int if set_to_int else False
        
        # Use default arguments to capture current 'name' value
        def on_change(val, name=name, minimum=minimum, multiplier=multiplier):
            val = val/multiplier
            if val < minimum:
                val = minimum
            controls[name][0] = val if set_to_int else int(val)
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
