#!/bin/bash
# launch_arm.sh  — source this instead of calling ros2 launch directly
VENV_SITE="${HOME}/.springcontroller_venv/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages"

# fail loudly if the venv isn't there
if [ ! -d "$VENV_SITE" ]; then
    echo "ERROR: venv site-packages not found at $VENV_SITE" >&2
    exit 1
fi


export PYTHONPATH="${VENV_SITE}:${PYTHONPATH}"

exec ros2 launch bph_statemachine arm.launch.py "$@"
