#!/usr/bin/env python
from Tkinter import *

top = Tk()

def helloCallBack():
   tkMessageBox.showinfo( "Hello Python", "Hello World")

B1 = Button(top, text ="Start", command = helloCallBack, height = 10, width = 50)
B2 = Button(top, text ="Cancel", command = helloCallBack, height = 10, width = 50)
B3 = Button(top, text ="Stop", command = helloCallBack, height = 10, width = 50, state = DISABLED)

B1.pack(side = LEFT)
B2.pack(side = LEFT)
B3.pack(side = LEFT)

top.mainloop()