import cv2
import numpy as np
import pyttsx

def tts(output) :
    engine = pyttsx.init()
    engine.setProperty('rate', 100)
    engine.say(output)
    engine.runAndWait()
