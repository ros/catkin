from __future__ import print_function
import argparse
import os
import stat
import sys

try:
    from catkin.environment_cache import generate_environment_script, generate_static_environment_script
except ImportError:
    # find the import relatively to make it work before installing catkin
    sys.path.append(os.path.join('@catkin_EXTRAS_DIR', '..', 'python'))
    from catkin.environment_cache import generate_environment_script, generate_static_environment_script


parser = argparse.ArgumentParser(description='Generate cached environment script.')
parser.add_argument('--static', action='store_true', help='The flag to skip sourcing the env script and generate a static environment')
args = parser.parse_args()

if not args.static:
    code = generate_environment_script('@ENV_SCRIPT')
else:
    code = generate_static_environment_script('@PREPEND_SPACE_DIR', @CMAKE_PREFIX_PATH, '@PYTHON_INSTALL_DIR')

with open('@OUTPUT_SCRIPT', 'w') as f:
    #print('Generate script for cached environment "%s"' % '@OUTPUT_SCRIPT')
    f.write('\n'.join(code))

mode = os.stat('@OUTPUT_SCRIPT').st_mode
os.chmod('@OUTPUT_SCRIPT', mode | stat.S_IXUSR)
