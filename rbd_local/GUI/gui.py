# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 18:09:22 2023

@author: jurip
"""


import customtkinter
import subprocess

customtkinter.set_appearance_mode("dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_widget_scaling(0.1)  # window geometry dimensions
customtkinter.set_window_scaling(0.2)  # window geometry dimensions

title_font = ("Ubuntu Mono", 87)
text_font = ("Ubuntu Mono", 50)


app = customtkinter.CTk(fg_color="#101020")
app.geometry("3000x2000")
app.title("CustomTkinter simple_example.py")

def button_callback():
    print("Button click")


# def slider_callback(value):
#     progressbar_1.set(value)
    
def run_script(self):
    # Replace "path/to/script.sh" with the actual path to your Bash script
    subprocess.call(["/bin/bash", "path/to/script.sh"])


frame_1 = customtkinter.CTkFrame(master=app, fg_color="#202030")
frame_1.pack(pady=70, padx=20, fill="both", expand=True)

# label_1 = customtkinter.CTkLabel(master=frame_1, justify=customtkinter.LEFT, font = gui_font, text="RoboDog III")
# label_1.pack(pady=40, padx=30)


title_frame = customtkinter.CTkFrame(master=frame_1, fg_color="#202030")
title_frame.pack(pady=20, padx=20)

title = customtkinter.CTkLabel(master=title_frame, justify=customtkinter.LEFT, font = title_font, text= "RoboDog III User Interface")
title.pack(padx=10, pady = 10)


media_frame = customtkinter.CTkFrame(master=frame_1, fg_color="#303040")
media_frame.pack(pady=20, padx=20,fill="both", expand=True)


grid_frame = customtkinter.CTkFrame(master=media_frame, fg_color="#303040")
grid_frame.grid(row = 2, column = 2, pady=0, padx=0)

label_1 = customtkinter.CTkLabel(master=grid_frame, justify=customtkinter.LEFT, font = text_font, text= "Emergency Stop")
label_1.grid(row=0, column=0, padx=300, pady = 50)

label_2 = customtkinter.CTkLabel(master=grid_frame, justify=customtkinter.CENTER, font=text_font, text = "Current State")
label_2.grid(row=0, column=1, padx=200, pady = 50)

#Add Button
button_1 = customtkinter.CTkButton(master=grid_frame, command=button_callback, font=text_font, text= "Emergency Stop")
button_1.grid(row=1, column=0, padx=200, pady = 10)

# #Create Frame & Label
# # master_frame = customtkinter.CTkFrame(master=app)
# # frame_left = customtkinter.CTkFrame(master=master_frame)

# # # master_frame.grid(row=0, column=0, padx=50, pady=50, sticky="nsew")

# # label_1 = customtkinter.CTkLabel(master=grid_frame, justify=customtkinter.LEFT, font = gui_font)
# # label_1.pack(padx=200, pady = 200)


# # # label_1.pack(pady=10, padx=10)



app.mainloop()

