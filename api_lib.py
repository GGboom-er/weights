from maya import cmds
import os
import sys
try:
    from importlib import reload
except ImportError:
    pass
use_default = False

version = int(round(float(cmds.about(q=1, v=1))))
pyd_path = os.path.abspath(__file__+"/../cores/maya%i" % version).replace("\\", "/")
py_path = os.path.abspath(__file__+"/../cores/maya%s" % "default").replace("\\", "/")
if os.path.exists(pyd_path):
    path = pyd_path
else:
    path = py_path

if use_default:
    if path in sys.path:
        sys.path.remove(path)
    path = py_path

if path not in sys.path:
    sys.path.insert(0, path)
import sculpt_api
import bezier_api
import weight_tool_api
reload(sculpt_api)
reload(bezier_api)
reload(weight_tool_api)



