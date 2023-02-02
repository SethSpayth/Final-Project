"pip install playsound==1.2.2"
import os
from playsound import playsound
os.chdir(os.path.dirname(os.path.abspath(__file__)))
playsound("a.wav")
