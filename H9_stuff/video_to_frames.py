# this script is used to extract frames from a video (gate.mp4) 
# and save them in a folder (frames/)

from detecto.utils import split_video

split_video('gate.mp4', 'frames/', step_size = 4)
