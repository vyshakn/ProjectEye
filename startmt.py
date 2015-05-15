from threading import Thread
from Queue import Queue
import tts
import inputctl
import processmod

inpQ = Queue(maxsize=0)

tts.tts(" Starting Program ")


inpctl = Thread(target = inputctl.inpctl , args = (inpQ,))
processmodl = Thread(target = processmod.process , args = (inpQ,))

# Starting Input Control Module
inpctl.start()

# Starting Processing Module
processmodl.start()
