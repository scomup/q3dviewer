import time
t1 = time.time()
from q3dviewer.custom_items import *
t2 = time.time()
from q3dviewer.glwidget import *
from q3dviewer.viewer import *
from q3dviewer.base_item import *
from q3dviewer.base_glwidget import *
t3 = time.time()

from q3dviewer.Qt import Q3D_DEBUG
if Q3D_DEBUG:
    print("Import custom items: ", t2 - t1)
    print("Import base q3dviewer: ", t3 - t2)
