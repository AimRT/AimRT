# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

import tkinter as tk
from tkinter import ttk
import threading
import http.client
import urllib.parse
import json
import time


class URLApp:
    def __init__(self, root):
        self.root = root
        self.root.title("App")

        self.url_entry = tk.Entry(root, width=100)
        self.url_entry.insert(
            0, "http://127.0.0.1:50080/rpc/aimrt.protocols.time_manipulator_plugin.TimeManipulatorService/SetTimeRatio")
        self.url_entry.pack()

        self.executor_name_entry = tk.Entry(root, width=100)
        self.executor_name_entry.insert(0, "time_schedule_executor")
        self.executor_name_entry.pack()

        self.discrete_values = [0, 1 / 8, 1 / 4, 1 / 2, 1, 2, 4, 8]
        self.slider_value = tk.DoubleVar()
        self.slider = ttk.Scale(root, from_=0, to=len(self.discrete_values) - 1,
                                orient='horizontal', length=200,
                                variable=self.slider_value, command=self.update_slider_value)
        self.slider.pack()

        self.create_scale_labels(root)

        self.current_value_label = tk.Label(root, text="Current Value: 1")
        self.current_value_label.pack()

        self.toggle_button = ttk.Checkbutton(root, text="Start", command=self.toggle)
        self.toggle_button.pack()

        self.locked = False
        self.last_slider_value = None

    def create_scale_labels(self, root):
        scale_frame = tk.Frame(root)
        scale_frame.pack()
        for i, value in enumerate(self.discrete_values):
            tk.Label(scale_frame, text=str(value)).grid(row=0, column=i)

    def update_slider_value(self, event=None):
        index = round(self.slider_value.get())
        self.slider_value.set(index)
        self.current_value_label.config(text=f"Current Value: {self.discrete_values[index]}")
        return self.discrete_values[index]

    def toggle(self):
        self.locked = not self.locked
        self.url_entry.config(state='disabled' if self.locked else 'normal')
        self.executor_name_entry.config(state='disabled' if self.locked else 'normal')

        if self.locked:
            self.start_thread()

    def start_thread(self):
        thread = threading.Thread(target=self.monitor_slider)
        thread.daemon = True
        thread.start()

    def monitor_slider(self):
        while self.locked:
            current_value = self.slider_value.get()
            if current_value != self.last_slider_value:
                self.last_slider_value = current_value
                self.send_request(current_value)
            time.sleep(0.5)

    def send_request(self, value):
        url = self.url_entry.get()
        executor_name = self.executor_name_entry.get()
        index = round(value)
        if url:
            try:
                parsed_url = urllib.parse.urlparse(url)
                conn = http.client.HTTPConnection(parsed_url.netloc)
                headers = {'content-type': 'application/json'}
                json_data = json.dumps({'executor_name': executor_name, 'time_ratio': self.discrete_values[index]})
                conn.request("POST", parsed_url.path, json_data, headers)
                response = conn.getresponse()
                print(
                    f"Sent POST request to {url} with json_data {json_data}, response: {response.status} {response.reason}")
                conn.close()
            except Exception as e:
                print(f"Error sending request: {}".format(e))


if __name__ == "__main__":
    root = tk.Tk()
    app = URLApp(root)
    root.mainloop()
