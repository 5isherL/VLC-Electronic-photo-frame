# Hope You All Like It
#Email- namah123jain@gmail.com
#Video Link- https://youtu.be/EqqC88QszOc
#channel-link:- https://www.youtube.com/channel/UCUGAq4ALoWW4PDU6Cm1riSg
#Developer:-Namah Jain (All About Code)
 
#Imports:---------
#install all of them using pip
from tkinter import * #Comes Defalut With Python3
from tkinter import filedialog as fd
from tkinter import messagebox as ms
import PIL # Install Using PIP
import serial
import os, sys
from PIL import ImageTk, Image


#Main Class Object
class application:
    def __init__(self,master):
        self.master = master
        self.c_size = (800,480)
        self.setup_gui(self.c_size)
        self.img=None
 
    def setup_gui(self,s):
        Label(self.master,text = 'Image Viewer',pady=5,bg='white',
            font=('Consolas',16)).pack()
        self.canvas = Canvas(self.master,height=s[1],width=s[0],
            bg='black',bd=10,relief='ridge')
        self.canvas.pack()
        txt = '''
                                                    Waiting for image loading
        '''
        self.wt = self.canvas.create_text(s[0]/2-270,s[1]/2,text=txt
            ,font=('',30),fill='white')
        f=Frame(self.master,bg='white',padx=10,pady=10)
        Button(f,text='Open Image',width=15, height=2,
        command=self.make_image).pack(side=LEFT)
        f.pack()
        self.status=Label(self.master,text = 'Current Image: None',bg='gray',
            font=('Consolas',12),bd=2,fg='black',relief='sunken',anchor=W)
        self.status.pack(side=BOTTOM,fill=X)
        
        Button(f, 
        text='Transmmit',
        width=15, height=2, 
        command=self.transmmit).pack(side=LEFT)
        f.pack()

    def transmmit(self):
        try:
            ser = serial.Serial(comP.get(), baudrate = Baud.get(), timeout = 0.5)        
            dataFile = open(File, 'rb')
            size2read = 1
            data2send = dataFile.read(size2read)
            while len(data2send) > 0:
                ser.write(data2send)
                data2send = dataFile.read(size2read)
            dataFile.close()
            ser.write(b"!!")
            ser.close()
            self.status['text']='Trans Succes'
        except:
            ms.showerror('Error!','Trans Fail !')

    def make_image(self):
        try:
            global File
            File = fd.askopenfilename()
            try:
                self.pilImage = Image.open(File)
                re=self.pilImage.resize((800,480),Image.ANTIALIAS)
                self.img = ImageTk.PhotoImage(re)
                self.canvas.delete(ALL)
                self.canvas.create_image(self.c_size[0]/2+10,self.c_size[1]/2+10,
                    anchor=CENTER,image=self.img)
                self.status['text']='Image Path:'+File
            except:
                self.status['text']='File Path:'+File
        except:
            ms.showerror('Error!','File type is unsupported.')

#creating object of class and tk window-
root=Tk()
root.configure(bg='white')
root.title('Image Viewer')
Baud = StringVar()
comP = StringVar()
sendString = StringVar()
Label(text = "Baud Rate").pack()
Baud.set('19200')
comP.set('COM8')
entry = Entry(root, textvariable=Baud).pack()
Label(text = "COMPORT").pack()
entry = Entry(root, textvariable=comP).pack()
application(root)

root.resizable(0,0)
root.mainloop()
