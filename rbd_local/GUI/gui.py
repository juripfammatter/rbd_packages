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
    def __init__(self):
        super().__init__()
        # initialize rosbridge
        self.init_rosbridge()

        # GUI
        self.em_stop = True

        customtkinter.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
        customtkinter.set_default_color_theme("dark-blue")  # Themes: "blue" (standard), "green", "dark-blue"
        customtkinter.set_widget_scaling(0.1)  # window geometry dimensions
        customtkinter.set_window_scaling(0.2)  # window geometry dimensions

        title_font = ("Ubuntu Mono", 87)
        text_font = ("Ubuntu Mono", 50)

        self.geometry("1400x1100")
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
                                             fg_color="#202030")

        media_frame.pack(pady=20, padx=20,fill="both", expand=True)


        left_col_grid = customtkinter.CTkFrame(master=media_frame, 
                                            fg_color="#303040")
        
        left_col_grid.grid(row = 0, column = 0, pady=20, padx=20)

        left_col = customtkinter.CTkFrame(master=left_col_grid, 
                                            fg_color="#303040")
        
        left_col.grid(row = 4, column = 1, pady=10, padx=10)



        right_col_grid = customtkinter.CTkFrame(master=media_frame, 
                                            fg_color="#303040")
        
        right_col_grid.grid(row = 0, column = 1, pady=20, padx=20)

        right_col = customtkinter.CTkFrame(master=right_col_grid, 
                                            fg_color="#303040")
        
        right_col.grid(row = 4, column = 1, pady=42, padx=10)


        # Labels
        label_1 = customtkinter.CTkLabel(master=left_col, 
                                         justify=customtkinter.LEFT, 
                                         font = text_font, 
                                         text= "Emergency Stop:")


        label_2 = customtkinter.CTkLabel(master=left_col, 
                                         justify=customtkinter.CENTER, 
                                         font=text_font, 
                                         text = "Current State:")
        
        self.status_text = customtkinter.CTkLabel(master=left_col, 
                                            justify=customtkinter.CENTER, 
                                            font=text_font, 
                                            text_color= "#FFFFFF",
                                            fg_color="#FF0000",
                                            padx = 45,
                                            pady = 13,
                                            corner_radius= 5,
                                            text = "em_stop",
                                            width=600)
        
        #  Queue
        self.queue_1 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "move to (1.0, 0.0)",
                                         width=500)
        
        self.queue_2 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "move to (-1.0, 0.0)")
        
        self.queue_3 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "move to (0.0, 0.0)")
        
        self.queue_4 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "rotate to -180°")
        
        self.queue_5 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "rotate to 0°")
        
        self.queue_6 = customtkinter.CTkLabel(master=right_col, 
                                         justify=customtkinter.LEFT, 
                                         font=text_font, 
                                         text = "empty:")


        # Buttons
        self.em_button = customtkinter.CTkButton(master=left_col, 
                                           command=self.em_callback, 
                                           font=text_font,
                                           border_spacing=30,
                                           hover_color="#505080",
                                           fg_color="#7070D0",
                                           text= "Disable",
                                           width=600)


        label_1.grid(row=0, column=0, padx=20, pady = 50)
        self.em_button.grid(row=1, column=0, padx=20, pady = 50)
        label_2.grid(row=2, column=0, padx=20, pady = 50)
        self.status_text.grid(row=3, column=0, padx=20, pady = 50)

        self.queue_1.grid(row=4, column=0, padx=20, pady = 40)
        self.queue_2.grid(row=3, column=0, padx=20, pady = 40)
        self.queue_3.grid(row=2, column=0, padx=20, pady = 40)
        self.queue_4.grid(row=1, column=0, padx=20, pady = 40)
        self.queue_5.grid(row=0, column=0, padx=20, pady = 40)
        
        
        
        
        


    # deconstructor
    def __del__(self):
        print("Terminating rosbridge connection")
        self.client.terminate()

    def init_rosbridge(self):
        """ FYI: run these first in terminals
        roslaunch rosbridge_server rosbridge_websocket.launch
        rosrun tf2_web_republisher tf2_web_republisher
        """
        # rosbridge client
        self.client= roslibpy.Ros(host='0.0.0.0', port=9090)
        self.client.run();        
        print('rosbridge connection status:', self.client.is_connected)

        # rosbridge service
        self.service = roslibpy.Service(self.client, '/emergency_stop', 'std_srvs/SetBool') #, reconnect_on_close= True
        self.request = roslibpy.ServiceRequest()

        # status listener
        self.status_listener = roslibpy.Topic(self.client, '/rbd_status', 'std_msgs/String')   
        self.status_listener.subscribe(self.status_callback)

        # status listener
        self.pose_listener = roslibpy.Topic(self.client, '/named_poses', 'rbd_msgs/NamedPoses')   
        self.pose_listener.subscribe(self.pose_callback)


    def status_callback(self, message):
        self.status = message['data']
        print(self.status)

        if(self.status == "em_stop"):
            self.status_text.configure(text = self.status, fg_color = "#FF0000")
            self.em_button.configure(text = "Disable")
        else:
            self.status_text.configure(text = self.status, fg_color = "#505080")
            self.em_button.configure(text = "Enable")


    def pose_callback(self, message):
        # print(message['namedPoses'])
        
        queue_text = ['','','','','']
        for i in range(0, len(message['namedPoses'])):
            queue_text[i] = message['namedPoses'][i]
        # print(queue_text)
        
        # configure labels
        self.queue_1.configure(text = queue_text[0])
        self.queue_2.configure(text = queue_text[1])
        self.queue_3.configure(text = queue_text[2])
        self.queue_4.configure(text = queue_text[3])
        self.queue_5.configure(text = queue_text[4])



    def em_callback(self):
        
        if(self.status == "em_stop"):
            #Idle
            print("Disabling emergency stop")

            # call service
            self.request['data'] = False
            self.service.call(self.request, callback= self.cb, timeout= 1)

        else:
            #EmStop
            print("Enabling emergency stop")

            # call service
            self.request['data'] = True
            self.service.call(self.request, callback= self.cb, timeout= 1)


    def run_script(self):
        # Replace "path/to/script.sh" with the actual path to your Bash script
        subprocess.call(["/bin/bash", "path/to/script.sh"])

    #callback function for service call (needed to ensure that service is non-blocking)
    #https://roslibpy.readthedocs.io/en/latest/reference/index.html
    def cb(self, fill):
        print("called")
        


app = App()
app.mainloop()

