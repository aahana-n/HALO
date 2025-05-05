import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import cv2.aruco as aruco
import threading

def load_patient_details():
    patient_details[1] = "Aruco ID: 1\nName: Patient A\nAge: 45\nPrescribed Medicines:\n- Paracetamol\n- Ibuprofen\n- Amoxicillin"
    patient_details[2] = "Aruco ID: 2\nName: Patient B\nAge: 60\nPrescribed Medicines:\n- Metformin\n- Atorvastatin\n- Amlodipine"
    patient_details[3] = "Aruco ID: 3\nName: Patient C\nAge: 35\nPrescribed Medicines:\n- Omeprazole\n- Levothyroxine\n- Loratadine"

def update_gui(aruco_id):
    if aruco_id == 1:
        desc_text_box.delete("1.0", tk.END)
        desc_text_box.insert(tk.END, patient_details.get(1, "No details available."))
    elif aruco_id == 2:
        middle_text_box.delete("1.0", tk.END)
        middle_text_box.insert(tk.END, patient_details.get(2, "No details available."))
    elif aruco_id == 3:
        code_text_box.delete("1.0", tk.END)
        code_text_box.insert(tk.END, patient_details.get(3, "No details available."))

def clear_patient_a():
    desc_text_box.delete("1.0", tk.END)

def clear_patient_b():
    middle_text_box.delete("1.0", tk.END)

def clear_patient_c():
    code_text_box.delete("1.0", tk.END)

def detect_aruco():
    #cap = cv2.VideoCapture("http://192.168.97.40:4747/video?640x480")
    cap = cv2.VideoCapture("http://192.168.170.2:4747/video?640x480")
    #cap = cv2.VideoCapture(1)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for id in ids.flatten():
                update_gui(id)
        
        aruco.drawDetectedMarkers(frame, corners, ids)
        #cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# GUI Code
root = tk.Tk()
root.title("TEAM METRON")
root.configure(bg='#e6e6fa')

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)

patient_details = {}
load_patient_details()

# RVCE logo
image_path = "logo.jpeg"
image = Image.open(image_path)
photo = ImageTk.PhotoImage(image.resize((200, 84), Image.LANCZOS))

image_label = tk.Label(root, image=photo, bg='#e6e6fa')
image_label.place(y=20, relx=0)

tagline = tk.Label(root, text="Go, Change the World!", font=("Times New Roman", 12, "italic", "bold"), bg='#e6e6fa', fg='black')
tagline.place(x=1100, y=35)

clg_name = tk.Label(root, text="R V College of Engineering", font=("Times New Roman", 35, "bold"), bg='#e6e6fa', fg='black')
clg_name.place(relx=0.5, y=50, anchor="center")

course_name = tk.Label(root, text="3rd SEMESTER EXPERIENTIAL LEARNING", font=("Times New Roman", 18, "bold"), bg='#e6e6fa', fg='black')
course_name.place(relx=0.5, y=100, anchor="center")

explrn = tk.Label(root, text="HALO: Hospital Autonomous Logistics Operations", font=("Times New Roman", 20, "bold"), bg='#e6e6fa', fg='black')
explrn.place(relx=0.5, y=130, anchor="center")

title = tk.Label(root, text="Patient Medicine Details", font=("Times New Roman", 17, "bold"), bg='#e6e6fa', fg='black')
title.place(relx=0.5, y=170, anchor="center")

names = tk.Label(root, text="BY: SUDHANSHU H, SHISHIR V, SURYANSH TRIPATHI, AAHANA NISCHAL", font=("Times New Roman", 12, "bold"), bg='#e6e6fa', fg='black')
names.place(relx=0.5, rely=0.95, anchor="center")

# Patient A details section
description_title = tk.Label(root, text="Patient A Details", font=("Times New Roman", 16, "bold"), bg='#e6e6fa', fg='black')
description_title.place(relx=0.17, rely=0.30, anchor="center")

desc_text_frame = tk.Frame(root, bg='#e8f1fd', bd=1)
desc_text_frame.place(relx=0.17, rely=0.6, anchor="center", width=400, height=400)

desc_text_box = tk.Text(desc_text_frame, wrap=tk.NONE, bg='white', fg='black', font=("Times New Roman", 16))
desc_text_box.pack(fill=tk.BOTH, expand=True)

# Done button for Patient A
done_button_a = tk.Button(root, text="Done", font=("Times New Roman", 12, "bold"), bg='green', fg='black', command=clear_patient_a)
done_button_a.place(relx=0.17, rely=0.82, anchor="center")

# Patient B details section
code_title = tk.Label(root, text="Patient B Details", font=("Times New Roman", 16, "bold"), bg='#e6e6fa', fg='black')
code_title.place(relx=0.50, rely=0.30, anchor="center")

middle_text_frame = tk.Frame(root, bg='#e8f1fd', bd=1)
middle_text_frame.place(relx=0.5, rely=0.6, anchor="center", width=400, height=400)

middle_text_box = tk.Text(middle_text_frame, wrap=tk.NONE, bg='white', fg='black', font=("Times New Roman", 16))
middle_text_box.pack(fill=tk.BOTH, expand=True)

# Done button for Patient B
done_button_b = tk.Button(root, text="Done", font=("Times New Roman", 12, "bold"), bg='green', fg='black', command=clear_patient_b)
done_button_b.place(relx=0.5, rely=0.82, anchor="center")

# Patient C details section
code_title = tk.Label(root, text="Patient C Details", font=("Times New Roman", 16, "bold"), bg='#e6e6fa', fg='black')
code_title.place(relx=0.82, rely=0.30, anchor="center")

code_text_frame = tk.Frame(root, bg='#e8f1fd', bd=1)
code_text_frame.place(relx=0.82, rely=0.6, anchor="center", width=400, height=400)

code_text_box = tk.Text(code_text_frame, wrap=tk.NONE, bg='white', fg='black', font=("Times New Roman", 16))
code_text_box.pack(fill=tk.BOTH, expand=True)

# Done button for Patient C
done_button_c = tk.Button(root, text="Done", font=("Times New Roman", 12, "bold"), bg='green', fg='black', command=clear_patient_c)
done_button_c.place(relx=0.82, rely=0.82, anchor="center")

# Start ArUco Detection in a separate thread
aruco_thread = threading.Thread(target=detect_aruco, daemon=True)
aruco_thread.start()

root.mainloop()