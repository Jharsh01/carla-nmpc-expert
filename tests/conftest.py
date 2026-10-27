import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# the built module (cmake --build build) and the repository's packages
sys.path[:0] = [os.path.join(REPO, "build", "python"), REPO]
