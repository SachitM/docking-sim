import sys
Path = 'src/align_navigation/scripts/PodLocationServer/'
sys.path.insert(1, Path)
from PodServer import *

a, b = GetPodLocAndWaypointsFileName(Path + 'PickupPodLoc.json', '12')