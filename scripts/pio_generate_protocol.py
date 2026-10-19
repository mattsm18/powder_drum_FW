#
# Title: scripts/pio_generate_protocol.py
# Purpose:
# - PlatformIO pre-build hook. Wired in via platformio.ini's extra_scripts.
# - Runs scripts/generate_protocol.py before the build so
#   include/protocol_generated.h is always regenerated from the latest
#   protocol definition in config/.
#

Import("env")

import re
import subprocess
import sys
from pathlib import Path

project_dir = Path(env.subst("$PROJECT_DIR"))
generator = project_dir / "scripts" / "generate_protocol.py"
config_dir = project_dir / "config"


def find_latest_protocol() -> Path:
    candidates = list(config_dir.glob("pd_comms_protocol_v*.json"))

    if not candidates:
        sys.stderr.write(f"[protocol] No protocol files found in {config_dir}\n")
        env.Exit(1)

    version_re = re.compile(r"pd_comms_protocol_v(\d+(?:\.\d+)*)\.json$")

    def version(path: Path):
        match = version_re.match(path.name)
        if not match:
            return ()
        return tuple(int(part) for part in match.group(1).split("."))

    return max(candidates, key=version)


protocol = find_latest_protocol()

print(f"[protocol] regenerating include/protocol_generated.h from {protocol.name}...")

result = subprocess.run(
    [sys.executable, str(generator)],
    cwd=project_dir,
)

if result.returncode != 0:
    sys.stderr.write("[protocol] generate_protocol.py failed — aborting build\n")
    env.Exit(1)