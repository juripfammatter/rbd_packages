# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 18:09:22 2023

@author: juri pfammatter
"""


import customtkinter
import subprocess
import roslibpy
from time import sleep
class App(customtkinter.CTk):
    # constructor
    def __init__(self):
        super().__init__()

        self.em_stop = True

        customtkinter.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
        customtkinter.set_default_color_theme("dark-blue")  # Themes: "blue" (standard), "green", "dark-blue"
        customtkinter.set_widget_scaling(0.1)  # window geometry dimensions
        customtkinter.set_window_scaling(0.2)  # window geometry dimensions

        title_font = ("Ubuntu Mono", 87)
        text_font = ("Ubuntu Mono", 50)

        self.geometry("1700x2000")
        self.title("RoboDog III User Interface")

        # Main Frame
        master_frame = customtkinter.CTkFrame(master=self, fg_color="#101020")
        master_frame.pack(pady=0, padx=0, fill="both", expand=True)

        sub_master_frame = customtkinter.CTkFrame(master=master_frame, fg_color="#202030")
        sub_master_frame.pack(pady=50, padx=20, fill="both", expand=True)


        # Title Frame
        title_frame = customtkinter.CTkFrame(master=sub_master_frame, fg_color="#202030")
        title_frame.pack(pady=20, padx=20)

        title = customtkinter.CTkLabel(master=title_frame, justify=customtkinter.LEFT, font = title_font, text= "RoboDog III User Interface")
        title.pack(padx=10, pady = 10)

        # Media Frame
        media_frame = customtkinter.CTkFrame(master=sub_master_frame, 
                                             fg_color="#303040")

        media_frame.pack(pady=20, padx=20,fill="both", expand=True)


        grid_frame = customtkinter.CTkFrame(master=media_frame, 
                                            fg_color="#303040")

        grid_frame.grid(row = 2, column = 2, pady=0, padx=100)


        # Labels
        label_1 = customtkinter.CTkLabel(master=grid_frame, 
                                         justify=customtkinter.LEFT, 
                                         font = text_font, 
                                         text= "Emergency Stop:")


        label_2 = customtkinter.CTkLabel(master=grid_frame, 
                                         justify=customtkinter.CENTER, 
                                         font=text_font, 
                                         text = "Current State:")
        
        self.status_text = customtkinter.CTkLabel(master=grid_frame, 
                                            justify=customtkinter.CENTER, 
                                            font=text_font, 
                                            text_color= "#FFFFFF",
                                            fg_color="#FF0000",
                                            padx = 45,
                                            pady = 13,
                                            corner_radius= 5,
                                            text = "Emergency Stop",
                                            width=600)


        # Buttons
        self.em_button = customtkinter.CTkButton(master=grid_frame, 
                                           command=self.em_callback, 
                                           font=text_font,
                                           border_spacing=30,
                                           hover_color="#505080",
                                           fg_color="#7070D0",
                                           text= "Disable",
                                           width=600)


        


        label_1.grid(row=0, column=0, padx=200, pady = 50)
        label_2.grid(row=1, column=0, padx=200, pady = 50)
        self.em_button.grid(row=0, column=1, padx=10, pady = 50)
        self.status_text.grid(row=1, column=1, padx=10, pady = 50)


        """ FYI: run these first in terminals
        roslaunch rosbridge_server rosbridge_websocket.launch
        rosrun tf2_web_republisher tf2_web_republisher
        """
        # rosbridge client
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run();        print('rosbridge connection status:', self.client.is_connected)

        # rosbridge service
        self.service = roslibpy.Service(self.client, '/rbd_pos_controller/emergency_stop_service', 'std_srvs/SetBool')
        self.request = roslibpy.ServiceRequest()

        # status listener
        self.listener = roslibpy.Topic(self.client, '/rbd_status', 'std_msgs/String')   
        self.listener.subscribe(self.status_callback)


    # deconstructor
    def __del__(self):
        print("Terminating rosbridge connection")
        self.client.terminate()


    def status_callback(self, message):
        self.status = message['data']


    def em_callback(self):
        if(self.em_stop):
            print("Terminating emergency stop")

            # Idle
            self.em_stop = False

            self.status_text.configure(text = self.status, fg_color = "#505080")
            self.em_button.configure(text = "Enable")

            # call service
            self.request['data'] = False
            self.result = self.service.call(self.request)
            print('Service response: {}'.format(self.result['loggers']))

        else:
            print("Initiating emergency stop")

            # EmStop
            self.em_stop = True

            self.status_text.configure(text = self.status, fg_color = "#FF0000")
            self.em_button.configure(text = "Disable")

            # call service
            self.request['data'] = True
            self.result = self.service.call(self.request)
            print('Service response: {}'.format(self.result['loggers']))
            



    def run_script(self):
        # Replace "path/to/script.sh" with the actual path to your Bash script
        subprocess.call(["/bin/bash", "path/to/script.sh"])

    def set_state(self, state, color):
        self.status_text.configure(text = state, text_color = color)
        
        


app = App()
app.mainloop()

