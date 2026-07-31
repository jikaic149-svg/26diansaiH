"""maixpy.skill 真机入口：运行已同步到板端的完整视觉工程。"""

import os
import sys


APP_DIR = "/root/steelball_horizontal_trough_pro"

if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)
os.chdir(APP_DIR)

from main import main


if __name__ == "__main__":
    main()
