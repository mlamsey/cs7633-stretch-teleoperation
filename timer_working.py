import tkinter as tk
import time
import pandas as pd
from cgitb import text
import time
from tkinter import *
from tkinter.ttk import *
import tkinter as tk
from tkinter import scrolledtext
from tkinter import ttk
from tkmacosx import Button #to be commented out for PC users
from tkinter import font as tkFont
from tkinter.messagebox import showinfo
from tkinter.simpledialog import askstring

class TimeRecorderGUI:

    def __init__(self, master):
        self.master = master
        self.master.title("Time Recorder")
        self.master.geometry("400x300")
        
        # Initialize variables
        self.participant_code = ''
        self.task_name = ''
        self.start_time = None
        self.end_time = None
        self.duration = 0
        self.elapsed_time_label = tk.Label(master, text='00:00:00', font=('Arial', 24))
        self.save_button = tk.Button(master, text='Finish', command=self.save_task)
        
        # Create and place widgets
        self.elapsed_time_label.pack(pady=50)
        self.save_button.pack()
        
        # Bind keyboard events
        master.bind('s', self.start_recording)
        master.bind('e', self.end_recording)

    def start_recording(self, event):
        if self.start_time is None:
            self.start_time = time.time()
            self.update_elapsed_time()
            print("Started recording time...")

    def end_recording(self, event):
          if self.start_time is not None:
            self.end_time = time.time()
            self.duration = self.end_time - self.start_time
            self.start_time = None
            self.update_elapsed_time()
            print("Stopped recording time.")

            self.participant_code, self.task_name = self.get_task_info()
            if self.participant_code and self.task_name:
                print("Task information: Participant Code = %s, Task Name = %s" % (self.participant_code, self.task_name))
            else:
                print("Input cancelled.")

    def update_elapsed_time(self):
        if self.start_time is not None:
            self.duration = time.time() - self.start_time
        elapsed_time = time.strftime("%H:%M:%S", time.gmtime(self.duration))
        self.elapsed_time_label.config(text=elapsed_time)
        self.master.after(1000, self.update_elapsed_time)
    
    
    def get_task_info(self):
        task_list=[]
        def save_task_callback():
            clicked= StringVar()
            clicked2= StringVar()
            participant_code = participant_code_entry.get()
            task_name = task_name_entry.get()
            self.popup.destroy()
            self.participant_code = participant_code
            self.task_name = task_name
            #modality_list = OptionMenu(self, clicked, "Hand Gestures","GUI","Xbox")
            #modality_list.place(x=50,y=100)
            #task_list = OptionMenu(self, clicked2, "Manipulation","Driving")
           # task_list.place(x=50,y=120)
            #self.task_name = clicked.get()+"-"+clicked2.get()
            if self.participant_code and self.task_name:
                data = pd.DataFrame({
                    'Participant Code': [self.participant_code],
                    'Task Name': [self.task_name],
                    'Duration': [self.duration]
                })
                data.to_csv('recorded_times'+self.participant_code+'.csv', mode='a', index=False, header=not os.path.exists('recorded_times.csv'))
                print("Data added to DataFrame.")

        self.popup = tk.Toplevel(self.master)
        self.popup.title('Save Task')
        tk.Label(self.popup, text='Participant Code:').grid(row=0, column=0)
        tk.Label(self.popup, text='Task Name:').grid(row=1, column=0)
        participant_code_entry = tk.Entry(self.popup)
        task_name_entry = tk.Entry(self.popup)
        participant_code_entry.grid(row=0, column=1)
        task_name_entry.grid(row=1, column=1)
        tk.Button(self.popup, text='Save', command=save_task_callback).grid(row=2, column=1)
        self.popup.focus_set()
        self.popup.grab_set()
        self.popup.wait_window()

    def save_task(self):
        if self.start_time is not None:
            self.end_time = time.time()
            self.duration = self.end_time - self.start_time
            self.start_time = None
            self.update_elapsed_time()
            print("Recording has been completed.")

            self.part
if __name__ == '__main__':
    root = tk.Tk()
    app = TimeRecorderGUI(root)
    root.mainloop()
