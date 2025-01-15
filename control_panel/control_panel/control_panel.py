import cv2
import threading
from typing import Optional, TypedDict, Union
from operator import itemgetter

class ControlPanelItem():
    value: Union[float, int]
    maximum: Optional[Union[float, int]]
    minimum: Optional[Union[float, int]]
    multiplier: Optional[int]
    set_to_int: Optional[bool]

    # def __init__(self, value, maximum, minimum, multiplier, set_to_int):
    def __init__(self, value, maximum=255, minimum=0, multiplier=1, set_to_int=False):
        self.value = value
        self.maximum = maximum
        self.minimum = minimum
        self.multiplier = multiplier
        self.set_to_int = set_to_int
        # self.maximum = maximum or 255
        # self.minimum = minimum or 0
        # self.multiplier = multiplier or 1
        # self.set_to_int = set_to_int or False

# a=ControlPanelItem(value=1)
# a.get('maximum')

def create_control_panel(control_panel_name: str, controls, **kwargs):#: dict[str, ControlPanelItem]):
    cv2.namedWindow(control_panel_name)
    for control in controls.items():
        name, v = control
        # default_value = None
        # value, maximum, minimum, multiplier, set_to_int = [*v, *([default_value] * (4 - len(v)))] # https://stackoverflow.com/a/59303329/7577786
        # value, maximum, minimum, multiplier, set_to_int = itemgetter('value', 'maximum', 'minimum', 'multiplier', 'set_to_int')(v)
        # value, maximum, minimum, multiplier, set_to_int = [v.get(f) for f in ['value', 'maximum', 'minimum', 'multiplier', 'set_to_int']]
        # value, maximum, minimum, multiplier, set_to_int = v
        # maximum = maximum if maximum else 255
        # minimum = minimum if minimum != None else 0
        # multiplier = multiplier if multiplier != None else 1
        # set_to_int = set_to_int if set_to_int else False

        # Use default arguments to capture current 'name' value
        def on_change(val, name=name, minimum=v.minimum, multiplier=v.multiplier):
            val = val/multiplier
            if val < minimum:
                val = minimum
            controls[name].value = val if v.set_to_int else int(val)
        cv2.createTrackbar(
            name, 
            control_panel_name, 
            int(v.value*v.multiplier),
            int(v.maximum*v.multiplier), 
            on_change
        )
    print(kwargs)
    no_new_thread = kwargs.get("no_new_thread") or False
    print(no_new_thread)
    if not no_new_thread:
        input_thread = threading.Thread(target=cv2.waitKey)
        input_thread.daemon = True
        input_thread.start()

# create_control_panel('asd',{})