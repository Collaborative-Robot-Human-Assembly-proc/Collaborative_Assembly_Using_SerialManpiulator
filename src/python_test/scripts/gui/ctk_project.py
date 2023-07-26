#!/usr/bin/env python3

import rospy
import std_msgs


import customtkinter as ctk
import os
from PIL import Image, ImageTk
import subprocess
import signal
import sys
import tkinter as tk
import sensor_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import strftime
from reverse_engineering import ReverseEngineering
from tkinter import messagebox


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control GUI")
        self.geometry(f"{self.winfo_screenwidth()}x{self.winfo_screenheight()}")
        ctk.set_default_color_theme("dark-blue")  # Themes: "blue" (standard), "green", "dark-blue"
        ctk.set_appearance_mode("dark")
        # set grid layout 1x2
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        
        self.run_roscore()

        rospy.init_node('FULLGUI')
        self.n = rospy.Publisher('/hardware',std_msgs.msg.Int16,queue_size=1)


        self.commands_list = []
        self.c_number = 0
        self.file_analyzed = 1
        self.speech_recognition_node = False


        # load images with light and dark mode image
        image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_images")
        self.logo_image = ctk.CTkImage(Image.open(os.path.join(image_path, "180207_ASU-Logo-400-removebg-preview.png")), size=(40, 40))
        
        # self.large_image_home = ctk.CTkImage(Image.open(os.path.join(image_path, "main_menu.png")), size=(800, 150))
        # self.large_image_robot_camera = ctk.CTkImage(Image.open(os.path.join(image_path, "robot_camera.png")), size=(800, 150))
        # self.large_image_environment_camera = ctk.CTkImage(Image.open(os.path.join(image_path, "environment_camera.png")), size=(800, 150))
        # self.large_files_analysis = ctk.CTkImage(Image.open(os.path.join(image_path, "files_analysis.png")), size=(800, 150))
        
        # self.image_icon_image = ctk.CTkImage(Image.open(os.path.join(image_path, "image_icon_light.png")), size=(20, 20))
        
        self.home_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "home_dark.png")),
                                                 dark_image=Image.open(os.path.join(image_path, "home_light.png")), size=(20, 20))
        self.robot_camera_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "robot_icon_black.png")),
                                                 dark_image=Image.open(os.path.join(image_path, "robot_icon_white.png")), size=(20, 20))
        self.environment_camera_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "environment_icon_black.png")),
                                                     dark_image=Image.open(os.path.join(image_path, "environment_icon_white.png")), size=(20, 20))
        
        self.files_analysis_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "files_black.png")),
                                                     dark_image=Image.open(os.path.join(image_path, "files_white.png")), size=(20, 20))
        
        self.info_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "info_icon_black.png")),
                                                     dark_image=Image.open(os.path.join(image_path, "info_icon_white.png")), size=(20, 20))

        self.user_input_image = ctk.CTkImage(light_image=Image.open(os.path.join(image_path, "useri_icon_black.png")),
                                                     dark_image=Image.open(os.path.join(image_path, "useri_icon_white.png")), size=(20, 20))

        # create navigation frame
        self.navigation_frame = ctk.CTkFrame(self, corner_radius=0)
        self.navigation_frame.grid(row=0, column=0, sticky="nsew")
        self.navigation_frame.grid_rowconfigure(7, weight=1)

        self.navigation_frame_label = ctk.CTkLabel(self.navigation_frame, text="  ASU COBOT GUI", image=self.logo_image,
                                                             compound="left", font=ctk.CTkFont(size=15, weight="bold"))
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20)

        self.home_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="Home",
                                                   fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                   image=self.home_image, anchor="w", command=self.home_button_event)
        self.home_button.grid(row=1, column=0, sticky="ew")

        self.robot_camera_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="Robot Camera",
                                                      fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                      image=self.robot_camera_image, anchor="w", command=self.robot_camera_button_event)
        self.robot_camera_button.grid(row=2, column=0, sticky="ew")

        self.environment_camera_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="Environment Camera",
                                                      fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                      image=self.environment_camera_image, anchor="w", command=self.environment_camera_button_event)
        self.environment_camera_button.grid(row=3, column=0, sticky="ew")

        self.files_analysis_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="Files Analysis",
                                                      fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                      image=self.files_analysis_image, anchor="w", command=self.files_analysis_button_event)
        self.files_analysis_button.grid(row=4, column=0, sticky="ew")
        
        self.user_input_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="User Input",
                                                      fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                      image=self.user_input_image, anchor="w", command=self.user_input_button_event)
        self.user_input_button.grid(row=5, column=0, sticky="ew")
        
        self.info_button = ctk.CTkButton(self.navigation_frame, corner_radius=0, height=40, border_spacing=10, text="Info",
                                                      fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                      image=self.info_image, anchor="w", command=self.info_button_event)
        self.info_button.grid(row=6, column=0, sticky="ew")
        

        self.appearance_mode_label = ctk.CTkLabel(self.navigation_frame, text="UI Theme Mode",font=ctk.CTkFont(size=14, weight="bold"))
        self.appearance_mode_label.grid(row=7, column=0, padx=20, pady=(10, 50), sticky="s")

        self.appearance_mode_menu = ctk.CTkOptionMenu(self.navigation_frame, values=["Dark", "Light", "System"],
                                                                command=self.change_appearance_mode_event)
        self.appearance_mode_menu.grid(row=7, column=0, padx=20, pady=20, sticky="s")

        self.appearance_mode_date = ctk.CTkLabel(self.navigation_frame, text=f'{strftime("%a")}, {strftime("%b")} {strftime("%Y")}'
            ,font=ctk.CTkFont(size=14, weight="bold"))
        self.appearance_mode_date.grid(row=7, column=0, padx=20, pady=(10, 120), sticky="s")

        self.appearance_mode_clock = ctk.CTkLabel(self.navigation_frame, text=f'{strftime("%I")}:{strftime("%M")}:{strftime("%S")} {strftime("%p")}'
            ,font=ctk.CTkFont(size=12, weight="bold"))
        self.appearance_mode_clock.grid(row=7, column=0, padx=20, pady=(10, 100), sticky="s")




        # create home frame
        self.home_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.home_frame.grid_columnconfigure(0, weight=1)

        self.home_heading_frame = ctk.CTkFrame(self.home_frame,corner_radius=0,height=80)
        self.home_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.home_heading_frame.grid_rowconfigure(0, weight=1)
        self.home_heading_frame.grid_columnconfigure(0, weight=1)        
        
        self.home_heading_frame_label = ctk.CTkLabel(self.home_heading_frame, text="Robot Control Main Window"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.home_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")


        self.home_interface_frame = ctk.CTkFrame(self.home_frame, corner_radius=50)
        self.home_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.home_interface_frame.grid_columnconfigure(0, weight=0)
        self.home_interface_frame.grid_columnconfigure((1,2,3), weight=10)
        self.home_interface_frame.grid_rowconfigure((0,1,2), weight=0)
        

        self.home_frame_button_1 = ctk.CTkButton(self.home_interface_frame, text="Run Simulation Environment Only",font=ctk.CTkFont(size=15),
            command=lambda: self._demo_command(1))
        self.home_frame_button_1.grid(row=0, column=0, padx=40, pady=(20,0),sticky="we")

        self.home_frame_button_1_terminate = ctk.CTkButton(self.home_interface_frame, text="Close",font=ctk.CTkFont(size=15),
            command=lambda: self._demo_command(2))
        self.home_frame_button_1_terminate.grid(row=0, column=3, padx=40, pady=(20,0),sticky="we")

        self.home_frame_button_2 = ctk.CTkButton(self.home_interface_frame, text="Run Hardware Environment",font=ctk.CTkFont(size=15),
            command=lambda: self._hardware_command(1))
        self.home_frame_button_2.grid(row=1, column=0, padx=40, pady=(20,0),sticky="we")
        
        self.home_frame_button_2_start = ctk.CTkButton(self.home_interface_frame, text="Start Motors",font=ctk.CTkFont(size=15),
            command=lambda: self._motor_command(1))
        self.home_frame_button_2_start.grid(row=1, column=1, padx=40, pady=20,sticky="we")
        
        self.home_frame_button_2_stop = ctk.CTkButton(self.home_interface_frame, text="Stop Motors",font=ctk.CTkFont(size=15),
            command=lambda: self._motor_command(2))
        self.home_frame_button_2_stop.grid(row=1, column=2, padx=40, pady=20,sticky="we")
        
        self.home_frame_button_2_terminate = ctk.CTkButton(self.home_interface_frame, text="Close",font=ctk.CTkFont(size=15),
            command=lambda: self._hardware_command(2))
        self.home_frame_button_2_terminate.grid(row=1, column=3, padx=40, pady=(20,0),sticky="we")

        self.home_frame_button_3 = ctk.CTkButton(self.home_interface_frame, text="Connect to the robot only",font=ctk.CTkFont(size=15),
            command=lambda: self._connection_command(1))
        self.home_frame_button_3.grid(row=2, column=0, padx=40, pady=20,sticky="we")
        
        self.home_frame_button_3_terminate = ctk.CTkButton(self.home_interface_frame, text="Disconnect",font=ctk.CTkFont(size=15),
            command=lambda: self._connection_command(2))
        self.home_frame_button_3_terminate.grid(row=2, column=3, padx=40, pady=20,sticky="we")

        self.home_terminal_frame = ctk.CTkFrame(self.home_frame,corner_radius=50)
        self.home_terminal_frame.grid(row=2, column=0,padx=10, pady=10,sticky="nsew")
        self.home_terminal_frame.grid_rowconfigure((0,1), weight=0)
        self.home_terminal_frame.grid_columnconfigure(0, weight=1)

        self.home_switchbutton_var = ctk.StringVar(value=0)
        self.home_switchbutton = ctk.CTkSwitch(self.home_terminal_frame
            , text="Begin Full Process (Must do file analysis first)", command=self.home_switchbutton_event,
            variable=self.home_switchbutton_var, onvalue=1, offvalue=0)

        self.home_switchbutton.grid(row=0, column=0, padx=40, pady=20,sticky="nw")
        self.home_switchbutton.configure(state='disabled')

        self.home_heading_frame_label_ga = ctk.CTkLabel(self.home_terminal_frame, text="Grid Analyzed")
        self.home_heading_frame_label_ga.grid(row=0, column=0, padx=(40,600), pady=(60,20),sticky="n")

        self.home_terminal = ctk.CTkTextbox(master=self.home_terminal_frame,corner_radius=50)
        self.home_terminal.grid(row=0, column=0,padx=(40,0),pady=(100,20),sticky="nsw")
        self.home_terminal.configure(state='disabled',width=500)
        

        self.home_heading_frame_label_o = ctk.CTkLabel(self.home_terminal_frame, text="Output")
        self.home_heading_frame_label_o.grid(row=0, column=0, padx=(560,40), pady=(60,20),sticky="n")

        self.home_terminal_2 = ctk.CTkTextbox(master=self.home_terminal_frame,corner_radius=50)
        self.home_terminal_2.grid(row=0, column=0,padx=(0,40),pady=(100,20),sticky="nse")
        self.home_terminal_2.configure(state='disabled',width=500)


        self.home_terminal_scrollbar = ctk.CTkScrollbar(self.home_terminal_frame)
        self.home_terminal_scrollbar.grid(row=0, column=0, padx=10, pady=50,sticky="ens")

        self.home_terminal.configure(yscrollcommand=self.home_terminal_scrollbar.set)
        self.home_terminal_2.configure(yscrollcommand=self.home_terminal_scrollbar.set)












        # create second frame
        self.robot_camera_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.robot_camera_frame.grid_columnconfigure(0, weight=1)

        self.robot_camera_heading_frame = ctk.CTkFrame(self.robot_camera_frame,corner_radius=0,height=80)
        self.robot_camera_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.robot_camera_heading_frame.grid_rowconfigure(0, weight=1)
        self.robot_camera_heading_frame.grid_columnconfigure(0, weight=1) 
        
        self.robot_camera_heading_frame_label = ctk.CTkLabel(self.robot_camera_heading_frame, text="Arm Camera Main Window"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.robot_camera_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")

        
        self.robot_camera_interface_frame = ctk.CTkFrame(self.robot_camera_frame, corner_radius=50)
        self.robot_camera_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.robot_camera_interface_frame.grid_rowconfigure((0,1), weight=0)
        self.robot_camera_interface_frame.grid_columnconfigure((0,1), weight=0)

        self.robot_camera_selection_label = ctk.CTkLabel(self.robot_camera_interface_frame, text="Select Image Format"
            ,font=ctk.CTkFont(size=12, weight="bold"))
        self.robot_camera_selection_label.grid(row=0, column=0, padx=40, pady=20, sticky="nw")

        self.robot_camera_selection = ctk.CTkOptionMenu(self.robot_camera_interface_frame
            , values=["No Image", "Parts Image", "Numbers Image"]
            , command=self.robot_camera_preview,width=200)
        self.robot_camera_selection.grid(row=0, column=0, padx=40, pady=50, sticky="nw")















        # create third frame
        self.environment_camera_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.environment_camera_frame.grid_columnconfigure(0, weight=1)

        self.environment_camera_heading_frame = ctk.CTkFrame(self.environment_camera_frame,corner_radius=0,height=80)
        self.environment_camera_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.environment_camera_heading_frame.grid_rowconfigure(0, weight=1)
        self.environment_camera_heading_frame.grid_columnconfigure(0, weight=1) 
        
        self.environment_camera_heading_frame_label = ctk.CTkLabel(self.environment_camera_heading_frame, text="Envrionment Camera Main Window"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.environment_camera_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")

        
        self.environment_camera_interface_frame = ctk.CTkFrame(self.environment_camera_frame, corner_radius=50)
        self.environment_camera_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.environment_camera_interface_frame.grid_rowconfigure((0,1), weight=0)
        self.environment_camera_interface_frame.grid_columnconfigure((0,1), weight=0)

        self.environment_camera_selection_label = ctk.CTkLabel(self.environment_camera_interface_frame, text="Select Image Format"
            ,font=ctk.CTkFont(size=12, weight="bold"))
        self.environment_camera_selection_label.grid(row=0, column=0, padx=40, pady=20, sticky="nw")

        self.environment_camera_selection = ctk.CTkOptionMenu(self.environment_camera_interface_frame
            , values=["No Image", "Mask Image", "Segmentation Image", "Pose Estimation Image"]
            , command=self.environment_camera_preview,width=200)
        self.environment_camera_selection.grid(row=0, column=0, padx=40, pady=50, sticky="nw")









 
        # create fourth frame
        self.files_analysis_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.files_analysis_frame.grid_columnconfigure(0, weight=1)

        self.files_analysis_heading_frame = ctk.CTkFrame(self.files_analysis_frame,corner_radius=0,height=80)
        self.files_analysis_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.files_analysis_heading_frame.grid_rowconfigure(0, weight=1)
        self.files_analysis_heading_frame.grid_columnconfigure(0, weight=1) 
        
        self.files_analysis_heading_frame_label = ctk.CTkLabel(self.files_analysis_heading_frame, text="File Analysis Main Window"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.files_analysis_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")

        
        self.files_analysis_interface_frame = ctk.CTkFrame(self.files_analysis_frame, corner_radius=50)
        self.files_analysis_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.files_analysis_interface_frame.grid_rowconfigure(0, weight=1)
        self.files_analysis_interface_frame.grid_columnconfigure((0,1), weight=1)

        self.l1 = ctk.CTkButton(self.files_analysis_interface_frame, text="File Analysis"
            ,font=ctk.CTkFont(size=12),command=self.file_analysis)
        self.l1.grid(row=0, column=0, padx=40, pady=10,sticky="wn")

        self.scrollable_radiobutton_frame = ScrollableRadiobuttonFrame(master=self.files_analysis_interface_frame
            , command=self.radiobutton_frame_event, item_list='',label_text="Parts List")
        self.scrollable_radiobutton_frame.grid(row=0, column=0, padx=40, pady=50, sticky="nsew")
        self.scrollable_radiobutton_frame.configure(width=400,height=50)



        self.file_analysis_textbox = ctk.CTkTextbox(self.files_analysis_interface_frame,corner_radius=50)
        self.file_analysis_textbox.grid(row=0, column=1, padx=40, pady=50,sticky="nsew")
        self.file_analysis_textbox.configure(width=400,height=50)

        self.file_analysis_textbox.configure(state='disabled')


        self.file_analysis_scrollbar = ctk.CTkScrollbar(self.file_analysis_textbox)
        self.file_analysis_scrollbar.grid(row=0, column=0, padx=0, pady=50,sticky="nse")

        self.file_analysis_textbox.configure(yscrollcommand=self.file_analysis_scrollbar.set)
        

        self.file_analysis_check_var = ctk.StringVar(value=0)
        self.file_analysis_checkbox = ctk.CTkCheckBox(self.files_analysis_interface_frame, 
            text="Show Excel Sheet", command=self.file_analysis_checkbox_event,variable=self.file_analysis_check_var, onvalue=1, offvalue=0)
        self.file_analysis_checkbox.grid(row=0, column=0, padx=40, pady=10,sticky="en")


        self.l2 = ctk.CTkButton(self.files_analysis_interface_frame, 
            text="Send File",font=ctk.CTkFont(size=12),command=self.file_analysis_send)
        self.l2.grid(row=0, column=1, padx=40, pady=10,sticky="ne")
        self.l2.configure(state='disabled')

        self.l3 = ctk.CTkButton(self.files_analysis_interface_frame, 
            text="Send List",font=ctk.CTkFont(size=12),command=self.file_analysis_list_send)
        self.l3.grid(row=0, column=1, padx=200, pady=10,sticky="ne")
        self.l3.configure(state='disabled')

        self.file_analysis_check_var_2 = ctk.StringVar(value=0)
        self.file_analysis_checkbox_2 = ctk.CTkCheckBox(self.files_analysis_interface_frame, 
            text="Show File", command=self.file_analysis_checkbox_2_event,variable=self.file_analysis_check_var_2, onvalue=1, offvalue=0)
        self.file_analysis_checkbox_2.grid(row=0, column=0, padx=40, pady=10,sticky="n")













        # create fifth frame
        self.user_input_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.user_input_frame.grid_columnconfigure(0, weight=1)

        self.user_input_heading_frame = ctk.CTkFrame(self.user_input_frame,corner_radius=0,height=80)
        self.user_input_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.user_input_heading_frame.grid_rowconfigure(0, weight=1)
        self.user_input_heading_frame.grid_columnconfigure(0, weight=1) 
        
        self.user_input_heading_frame_label = ctk.CTkLabel(self.user_input_heading_frame, text="User Input Main Window"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.user_input_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")
        self.user_input_heading_frame_label.grid_rowconfigure(0, weight=1)
        self.user_input_heading_frame_label.grid_columnconfigure(0, weight=1)
        
        self.user_input_interface_frame = ctk.CTkFrame(self.user_input_frame, corner_radius=50)
        self.user_input_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.user_input_interface_frame.grid_rowconfigure((0,1), weight=1)
        self.user_input_interface_frame.grid_columnconfigure((0,1), weight=1)


        self.user_input_entry = ctk.CTkEntry(self.user_input_interface_frame
            ,placeholder_text="Enter Command", font=ctk.CTkFont(size=14,weight='bold'),width=800,height=30)
        self.user_input_entry.grid(row=0, column=0,columnspan=2,padx=40, pady=10, sticky="nw")
        
        self.user_input_entry_button = ctk.CTkButton(self.user_input_interface_frame
            ,text="Enter",font=ctk.CTkFont(size=14,weight='bold'),command=self.user_input_entry_button_event)
        self.user_input_entry_button.grid(row=0, column=1,padx=40, pady=10, sticky="en")

        self.user_input_entry.bind('<Return>',self.user_input_entry_button_event)


        self.user_input_sr_switchbutton_var = ctk.StringVar(value=0)
        self.user_input_sr_switchbutton = ctk.CTkSwitch(self.user_input_interface_frame
            , text="Run Speech Recognition", command=self.user_input_sr_switchbutton_event,
            variable=self.user_input_sr_switchbutton_var, onvalue=1, offvalue=0)

        self.user_input_sr_switchbutton.grid(row=0, column=1,padx=40, pady=40, sticky="en")







        # create sixth frame
        self.info_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.info_frame.grid_columnconfigure(0, weight=1)

        self.info_heading_frame = ctk.CTkFrame(self.info_frame,corner_radius=0,height=80)
        self.info_heading_frame.grid(row=0, column=0,sticky="nsew")
        self.info_heading_frame.grid_rowconfigure(0, weight=1)
        self.info_heading_frame.grid_columnconfigure(0, weight=1) 
        
        self.info_heading_frame_label = ctk.CTkLabel(self.info_heading_frame, text="Information"
            ,font=ctk.CTkFont(size=30,weight='bold'))
        self.info_heading_frame_label.grid(row=0, column=0, padx=0, pady=50,sticky="n")
        self.info_heading_frame_label.grid_rowconfigure(0, weight=1)
        self.info_heading_frame_label.grid_columnconfigure(0, weight=1)
        
        self.info_interface_frame = ctk.CTkFrame(self.info_frame, corner_radius=50)
        self.info_interface_frame.grid(row=1, column=0,padx=10, pady=10, sticky="nsew")
        self.info_interface_frame.grid_rowconfigure(0, weight=0)
        self.info_interface_frame.grid_columnconfigure((0,1), weight=1)

        self.info_interface_frame_textbox = ctk.CTkTextbox(master=self.info_interface_frame,corner_radius=50)
        self.info_interface_frame_textbox.grid(row=0, column=0,padx=(40,40),pady=(20,0),sticky="nesw")
        with open('paragraph.txt', 'r') as file:
            paragraph = file.read()
        self.info_interface_frame_textbox.insert("0.0", paragraph)


        self.info_interface_frame_textbox.configure(state='disabled',width=1050,height=280)



        self.info_interface_frame_powered_img = ctk.CTkImage(Image.open(os.path.join(image_path, "Untitled design__.png")), size=(680, 340))

        self.info_interface_frame_powered = ctk.CTkLabel(self.info_interface_frame, text="Powered By", image=self.info_interface_frame_powered_img,
                                                             compound="right", font=ctk.CTkFont(size=25, weight="bold"))
        self.info_interface_frame_powered.grid(row=1, column=0, padx = 40, pady=0, sticky="nsw")

        # select default frame
        self.select_frame_by_name("Home")


        # updating clock time
        self.clock()



    def select_frame_by_name(self, name):
        # set button color for selected button
        self.home_button.configure(fg_color=("gray75", "gray25") if name == "Home" else "transparent")
        self.robot_camera_button.configure(fg_color=("gray75", "gray25") if name == "Robot Camera" else "transparent")
        self.environment_camera_button.configure(fg_color=("gray75", "gray25") if name == "Environment Camera" else "transparent")
        self.files_analysis_button.configure(fg_color=("gray75", "gray25") if name == "Files Analysis" else "transparent")
        self.user_input_button.configure(fg_color=("gray75", "gray25") if name == "User Input" else "transparent")
        self.info_button.configure(fg_color=("gray75", "gray25") if name == "Info" else "transparent")

        # show selected frame
        if name == "Home":
            self.home_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.home_frame.grid_forget()
        if name == "Robot Camera":
            self.robot_camera_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.robot_camera_frame.grid_forget()
        if name == "Environment Camera":
            self.environment_camera_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.environment_camera_frame.grid_forget()

        if name == "Files Analysis":
            self.files_analysis_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.files_analysis_frame.grid_forget()
        
        if name == "User Input":
            self.user_input_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.user_input_frame.grid_forget()
        
        if name == "Info":
            self.info_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.info_frame.grid_forget()

    def home_button_event(self):
        self.select_frame_by_name("Home")

    def robot_camera_button_event(self):
        self.select_frame_by_name("Robot Camera")

    def environment_camera_button_event(self):
        self.select_frame_by_name("Environment Camera")

    def files_analysis_button_event(self):
        self.select_frame_by_name("Files Analysis")
    
    def user_input_button_event(self):
        self.select_frame_by_name("User Input")
    
    def info_button_event(self):
        self.select_frame_by_name("Info")

    def change_appearance_mode_event(self, new_appearance_mode):
        ctk.set_appearance_mode(new_appearance_mode)


    def clock(self):
        self.appearance_mode_clock.configure(text=f'{strftime("%I")}:{strftime("%M")}:{strftime("%S")} {strftime("%p")}')
        self.after(1000, self.clock)





    def file_analysis(self):
        self.rev = ReverseEngineering()
        self.rev.full_parts_list()
        self.rev.show_draw_and_assembly()




        self.file_analysis_textbox.configure(state='normal')
        self.home_switchbutton.configure(state='normal',text='Begin Full Process')
        my_string = '\n'.join(', '.join(inner_list) for inner_list in self.rev.parts_list)
        text = f'\nParts:\n{my_string}\n' + '-'*50 + '\n'*2

        self.file_analysis_textbox.insert(ctk.END,text=text)

        self.file_analysis_textbox.configure(state='disabled')


        self.l3.configure(state='normal')


        self.scrollable_radiobutton_frame = ScrollableRadiobuttonFrame(master=self.files_analysis_interface_frame
            , command=self.radiobutton_frame_event
            , item_list=[f'{i}' for i in self.rev.names_list],label_text="Parts List",width=400)
        self.scrollable_radiobutton_frame.grid(row=0, column=0, padx=40, pady=(50,0), sticky="nsew")
        self.scrollable_radiobutton_frame.configure(width=400,height=50)

    def file_analysis_checkbox_event(self):
        pass

    def file_analysis_checkbox_2_event(self):
        pass 

    def file_analysis_send(self):
        try:
            p = rospy.Publisher('/one_file_analysed',std_msgs.msg.String,queue_size=1)
            message = std_msgs.msg.String()
            message = f'{self.rev.part}|{self.rev.diameter}'
            print(f'sending message: {message}')
            p.publish(message)
        except:
            pass
    
    def file_analysis_list_send(self):
        p = rospy.Publisher('/list_file_analyzed',std_msgs.msg.String,queue_size=1)
        message = std_msgs.msg.String()

        my_string = '\n'.join(', '.join(inner_list) for inner_list in self.rev.parts_list)
        message = my_string
        print(f'sending message: {my_string}')
        p.publish(message)



    def radiobutton_frame_event(self):
        idx = int(self.scrollable_radiobutton_frame.get_checked_item().split('.')[0])
        print(f"radiobutton frame modified: {idx}")
        self.rev.parameters(self.rev.ipts_list[idx-1])
        self.file_analysis_textbox.configure(state='normal')
        part_description =\
        f'\n- {self.scrollable_radiobutton_frame.get_checked_item()}\nPart: {self.rev.part}\tDiameter: {self.rev.diameter}\n'
        self.file_analysis_textbox.insert(ctk.END,text=part_description)
        # self.file_analysis_textbox.insert(ctk.END,text=self.rev.df)
        self.file_analysis_textbox.configure(state='disabled')
        
        if int(self.file_analysis_check_var.get()):
            subprocess.Popen(['libreoffice','output.xlsx'])


        if int(self.file_analysis_check_var_2.get()):
            self.rev.show_part(self.rev.ipts_list[idx-1])
        
        self.l2.configure(state='normal')


    def user_input_entry_button_event(self,event=None):
        input_command = f'{self.user_input_entry.get()}'
        print(input_command)
        input_command = input_command.split()
        print(input_command)
        subprocess.Popen(input_command)


    def user_input_sr_switchbutton_event(self):
        if int(self.user_input_sr_switchbutton_var.get()):
            subprocess.Popen(['python3','speech_recognition.py'])

        elif int(self.user_input_sr_switchbutton_var.get()) == 0:
            subprocess.Popen(['rosnode','kill','/speech_recognition_commander_node'])



    def home_switchbutton_event(self):
        if int(self.home_switchbutton_var.get()):
            subprocess.Popen(['python3','full_pro_2.py'])
            
            self.home_terminal_2.insert(ctk.END,text='-'*50+f'\nOUTPUT --------\n')
            self.home_terminal_2.configure(state='disabled')


            rospy.Subscriber('camera_list',std_msgs.msg.String,self.camera_list_callback)
            rospy.Subscriber('output',std_msgs.msg.String,self.output_callback)

            pass

        elif int(self.home_switchbutton_var.get()) == 0:
            subprocess.Popen(['rosnode','kill','tracker'])
            self.home_terminal.configure(state='normal')
            self.home_terminal_2.configure(state='normal')

            self.home_terminal.delete('1.0', ctk.END)
            self.home_terminal_2.delete('1.0', ctk.END)

            self.home_terminal.configure(state='disabled')
            self.home_terminal_2.configure(state='disabled')

            pass


    def camera_list_callback(self,camera_list):

        self.home_terminal.configure(state='normal')
        text = f'\nANALYSIS DONE --------\n{camera_list.data}\n' + '-'*50 + '\n'*2
        self.home_terminal.insert(ctk.END,text=text)
        self.home_terminal.configure(state='disabled')


    def output_callback(self,data):
        self.home_terminal_2.configure(state='normal')
        text = f'\n- {data.data}\n'
        self.home_terminal_2.insert(ctk.END,text=text)
        self.home_terminal_2.configure(state='disabled')


    def robot_camera_preview(self,topic):


        if topic == 'Parts Image':
            topic_name = 'parts_topic'
        
        elif topic == 'Numbers Image':
            topic_name = 'numbers_topic'
        
        else:
            topic_name = False
        
        self.video_app = RobotVideo(self, self.robot_camera_interface_frame,topic_name)

    def environment_camera_preview(self,topic):

        if topic == "Mask Image":
            topic_name = 'mask_pub'
        
        elif topic == "Segmentation Image":
            topic_name = 'seg_result'
        
        elif topic == "Pose Estimation Image":
            topic_name = 'pose_result'
        
        else:
            topic_name = False
        
        self.video_app = EnvrionmentVideo(self, self.environment_camera_interface_frame,topic_name)



    def _demo_command(self,command):

        if command == 1:

            process_name = ['roslaunch','robot2_moveit_config','demo.launch']
            separator = ' '
            p_name = separator.join(process_name)
            process = subprocess.Popen(process_name)
            self.home_frame_button_1.configure(state="disabled",fg_color='transparent')
            self.home_frame_button_2.configure(state="disabled")
            self.home_frame_button_3.configure(state="disabled")
            # self.home_terminal.configure(state='normal')
            # self.c_number+=1
            # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} opened')
            # self.home_terminal.configure(state='disabled')
            self.commands_list.append([process_name,process])


        elif command == 2:
            process_name = ['roslaunch','robot2_moveit_config','demo.launch']
            for index, sublist in enumerate(self.commands_list):
                for element in sublist:
                    if element == process_name:
                        separator = ' '
                        p_name = separator.join(process_name)
                        self.home_frame_button_1.configure(state='normal',fg_color='darkblue')
                        self.home_frame_button_2.configure(state='normal')
                        self.home_frame_button_3.configure(state='normal')
                        # self.home_terminal.configure(state='normal')
                        # self.c_number+=1
                        # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} closed.')
                        # self.home_terminal.configure(state='disabled')
                        self.commands_list[index][1].send_signal(signal.SIGTERM)
                        self.commands_list.pop(index)
                        break

    def _hardware_command(self,command):
        if command == 1:

            process_name = ['roslaunch','robot2_moveit_config','demo_hw.launch']
            self.n.publish(1)
            separator = ' '
            p_name = separator.join(process_name)
         
            # process = subprocess.Popen(process_name)
            self.home_frame_button_1.configure(state="disabled")
            self.home_frame_button_2.configure(state="disabled",fg_color='transparent')
            self.home_frame_button_3.configure(state="disabled")
            # self.home_terminal.configure(state='normal')
            # self.c_number+=1
            # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} opened')
            # self.home_terminal.configure(state='disabled')
            self.commands_list.append([process_name,p_name])

        elif command == 2:
            process_name = ['roslaunch','robot2_moveit_config','demo_hw.launch']

            for index, sublist in enumerate(self.commands_list):
                for element in sublist:
                    if element == process_name:
                        print(self.commands_list[index][1])

                        separator = ' '
                        p_name = separator.join(process_name)
                        self.home_frame_button_1.configure(state='normal')
                        self.home_frame_button_2.configure(state='normal',fg_color='darkblue')
                        self.home_frame_button_3.configure(state='normal')
                        # self.home_terminal.configure(state='normal')
                        # self.c_number+=1
                        # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} closed.')
                        # self.home_terminal.configure(state='disabled')
                        # self.commands_list[index][1].send_signal(signal.SIGTERM)
                        # print(self.commands_list[index][1])
                        self.commands_list.pop(index)
                        break
            self.n.publish(0)



    def _connection_command(self,command):

        if command == 1:

            process_name = ['roslaunch','abb_robot_bringup_examples','ex2_rws_and_egm_6axis_robot.launch']
            separator = ' '
            p_name = separator.join(process_name)
            process = subprocess.Popen(process_name)
            self.home_frame_button_1.configure(state="disabled")
            self.home_frame_button_2.configure(state="disabled")
            self.home_frame_button_3.configure(state="disabled",fg_color='transparent')
            # self.home_terminal.configure(state='normal')
            # self.c_number+=1
            # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} opened')
            # self.home_terminal.configure(state='disabled')
            self.commands_list.append([process_name,process])


        elif command == 2:
            process_name = ['roslaunch','abb_robot_bringup_examples','ex2_rws_and_egm_6axis_robot.launch']
            for index, sublist in enumerate(self.commands_list):
                for element in sublist:
                    if element == process_name:
                        print(self.commands_list[index][1])

                        separator = ' '
                        p_name = separator.join(process_name)
                        self.home_frame_button_1.configure(state='normal')
                        self.home_frame_button_2.configure(state='normal')
                        self.home_frame_button_3.configure(state='normal',fg_color='darkblue')
                        # self.home_terminal.configure(state='normal')
                        # self.c_number+=1
                        # self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {p_name} closed.')
                        # self.home_terminal.configure(state='disabled')
                        self.commands_list[index][1].send_signal(signal.SIGTERM)
                        self.commands_list.pop(index)
                        break



    def _motor_command(self,command):
        if command == 1:
            process = 'rosrun python_test start_robot.bash'

        elif command == 2:
            process = 'rosrun python_test stop_robot.bash'
        
        self.home_terminal.configure(state='normal')
        self.c_number+=1
        self.home_terminal.insert(ctk.END, text=f'\n{self.c_number}. {process} running...')
        self.home_terminal.configure(state='disabled')        
        subprocess.Popen(process, shell=True)


    def on_window_close(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            try:
             
                try:
                    os.kill(self.roscore_process[2], signal.SIGTERM) #or signal.SIGKILL

                except:
                    os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)  # Send the signal to all the process groups
            
            except:
                pass

            self.destroy()

    def run_roscore(self):
                # Check if roscore is already running
            try:
                self.roscore_process = rospy.get_master().getPid()


            except ConnectionRefusedError:
                # roscore is not running, start it
                self.roscore_process = subprocess.Popen('roscore', shell=True)

class RobotVideo:
    def __init__(self, root, video_frame,topic_name):
        self.root = root
        self.video_frame = video_frame
        self.w = self.root.winfo_screenwidth()//2
        self.h = self.root.winfo_screenheight()//2

        self.video = ctk.CTkLabel(self.video_frame,image=None,text='')
        self.video.grid(row=1, column=1, padx=0, pady=0,sticky="nsew")


        # Initialize the video capture
        self.video_capture = None

        # Initialize ROS

        # Create a CvBridge object
        self.bridge = CvBridge()
        self.topic_name = topic_name
        # Subscribe to the camera topic
        
        if topic_name is not False:
            # self.video_label.destroy()
            self.subscriber = rospy.Subscriber(topic_name, sensor_msgs.msg.Image, self.video_callback)

        else:
        
            try:
                self.subscriber.unregister()
        
            except AttributeError:
                pass

            self.video.configure(text='No Image',font=ctk.CTkFont(size=30,weight='bold'))

            # self.video_label = ctk.CTkLabel(self.video_frame,image=None,text='No Image',font=ctk.CTkFont(size=30,weight='bold'))
            # self.video_label.grid(row=1, column=1, padx=0, pady=10,sticky="n")




    def video_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized_frame = cv2.resize(cv_image, (self.w,self.h))

            # Resize the image to fit the label
            pil_image = Image.fromarray(resized_frame)

            # Create an ImageTk object from the RGB frame
            img_tk = ImageTk.PhotoImage(pil_image)

            # Update the video label with the new frame
            self.video.configure(image=img_tk)
            self.video.image = img_tk  # Keep a reference to prevent garbage collection


        
        except Exception as e:
            rospy.logerr(e)


class EnvrionmentVideo:
    def __init__(self, root, video_frame,topic_name):
        self.root = root
        self.video_frame = video_frame
        self.w = self.root.winfo_screenwidth()//2
        self.h = self.root.winfo_screenheight()//2

        self.video = ctk.CTkLabel(self.video_frame,image=None,text='')
        self.video.grid(row=1, column=1, padx=0, pady=0,sticky="nsew")


        # Initialize the video capture
        self.video_capture = None

        # Initialize ROS

        # Create a CvBridge object
        self.bridge = CvBridge()
        self.topic_name = topic_name
        # Subscribe to the camera topic
        
        if topic_name is not False:
            # self.video_label.destroy()
            self.subscriber = rospy.Subscriber(topic_name, sensor_msgs.msg.Image, self.video_callback)

        else:
        
            try:
                self.subscriber.unregister()
        
            except AttributeError:
                pass

            self.video.configure(text='No Image',font=ctk.CTkFont(size=30,weight='bold'))

            # self.video_label = ctk.CTkLabel(self.video_frame,image=None,text='No Image',font=ctk.CTkFont(size=30,weight='bold'))
            # self.video_label.grid(row=1, column=1, padx=0, pady=10,sticky="n")




    def video_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            resized_frame = cv2.resize(cv_image, (self.w,self.h))

            # Resize the image to fit the label
            pil_image = Image.fromarray(resized_frame)

            # Create an ImageTk object from the RGB frame
            img_tk = ImageTk.PhotoImage(pil_image)

            # Update the video label with the new frame
            self.video.configure(image=img_tk)
            self.video.image = img_tk  # Keep a reference to prevent garbage collection


        
        except Exception as e:
            rospy.logerr(e)


class ScrollableRadiobuttonFrame(ctk.CTkScrollableFrame):
    def __init__(self, master, item_list, command=None, **kwargs):
        super().__init__(master, **kwargs)

        self.command = command
        self.radiobutton_variable = ctk.StringVar()
        self.radiobutton_list = []
        for i, item in enumerate(item_list):
            self.add_item(item)

    def add_item(self, item):
        radiobutton = ctk.CTkRadioButton(self, text=item, value=item, variable=self.radiobutton_variable)
        if self.command is not None:
            radiobutton.configure(command=self.command)
        radiobutton.grid(row=len(self.radiobutton_list), column=0, pady=(0, 10))
        radiobutton.configure(height=30,width=200)
        self.radiobutton_list.append(radiobutton)

    def remove_item(self, item):
        for radiobutton in self.radiobutton_list:
            if item == radiobutton.cget("text"):
                radiobutton.destroy()
                self.radiobutton_list.remove(radiobutton)
                return

    def get_checked_item(self):
        return self.radiobutton_variable.get()


if __name__ == "__main__":
    app = App()
    print(app.roscore_process)
    app.protocol("WM_DELETE_WINDOW", app.on_window_close)
    app.mainloop()



