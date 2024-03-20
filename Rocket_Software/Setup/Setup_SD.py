#region Setup python packages
import os

# Removing the pip security
os.system('sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED')

# List of the package we need
pip_list = {
    "adafruit-circuitpython-bno055",
    "qtm==2.0.6",
    "opencv-contrib-python",
    "scipy",
    "--upgrade --force-reinstall adafruit-blinka"
    "matplotlib"
}

git_list = {

}

for package in pip_list:
    os.system(f"pip install {package}")
for git in git_list:
    os.system(str(git))
#endregion
