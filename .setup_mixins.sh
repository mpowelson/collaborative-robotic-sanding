wget https://raw.githubusercontent.com/jrgnicho/ros_development_config/master/general/colcon_ws_setup.py
python3 colcon_ws_setup.py

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export COLCON_DEFAULTS_FILE="${DIR}/.colcon_defaults.yaml"
