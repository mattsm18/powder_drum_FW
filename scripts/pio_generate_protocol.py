#
# Title: scripts/pio_generate_protocol.py
# Purpose:
# - PlatformIO pre-build hook. Wired in via platformio.ini's extra_scripts.
# - Runs scripts/generate_protocol.py before the build so
#   include/protocol_generated.h is always fresh from
#   config/pd_comms_protocol_v1.0.json — nobody has to remember to run it
#   by hand.
#
# NB: kept separate from generate_protocol.py so that script stays a plain,
# standalone CLI tool you (or the software side) can run outside of a PIO
# build context too.

Import("env")

import subprocess
import sys
from pathlib import Path

project_dir = Path(env.subst("$PROJECT_DIR"))
generator   = project_dir / "scripts" / "generate_protocol.py"

print(f"[protocol] regenerating include/protocol_generated.h from pd_comms_protocol_v1.0.json ...")

result = subprocess.run([sys.executable, str(generator)], cwd=project_dir)

if result.returncode != 0:
    sys.stderr.write("[protocol] generate_protocol.py failed — aborting build\n")
    env.Exit(1)