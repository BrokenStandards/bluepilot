import os
import platform
from openpilot.common.basedir import BASEDIR

# BluePilot: mapd binaries are committed in third_party/mapd_bp (BP fork of pfeiferj mapd),
# one static binary per architecture: 'mapd' (arm64/device) and 'mapd-x86_64' (PC).
MAPD_BIN_DIR = os.path.join(BASEDIR, 'third_party/mapd_bp')
MAPD_BIN_NAME = 'mapd' if platform.machine() == 'aarch64' else 'mapd-x86_64'
MAPD_PATH = os.path.join(MAPD_BIN_DIR, MAPD_BIN_NAME)
# End BluePilot
