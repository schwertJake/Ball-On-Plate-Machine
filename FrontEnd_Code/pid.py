from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import StringProperty
from kivy.clock import Clock
from threading import Thread
import serial
from queue import Queue
from kivy.garden.graph import MeshLinePlot
from kivy.core.window import Window
from kivy.config import Config

class pid(FloatLayout):
    # ----- Real Time Plotting List -----
    plot_1_list = []
    plot_2_list = []
    plot_3_list = []
    setpoint_list = []
    actual_list = []

    # ----- System Control Variables -----
    lit_button = None
    coms_on = False
    serial_read = False

    # ----- Tuning Variables -----
    p_val = 3.5
    i_val = 0
    d_val = 9

    # ---------- Init ----------
    # Initialize Kivy window, setup realtime
    # graphs, schedule update intervals, and
    # initialize the queue
    def __init__(self):
        super(pid, self).__init__()
        Window.clearcolor = (.14, .18, .2, 1)

        # Initialize but don't open serial coms
        self.ser = serial.Serial()
        self.ser.port = 'COM8'

        # Create Mesh Line Plots
        self.plot = MeshLinePlot()
        self.plot2 = MeshLinePlot()
        self.plot3 = MeshLinePlot()
        self.setpoint_plot = MeshLinePlot()
        self.act_pos_plot = MeshLinePlot()

        # Add line plots to graph
        self.ids.graph_1.add_plot(self.plot)
        self.ids.graph_2.add_plot(self.plot2)
        self.ids.graph_3.add_plot(self.plot3)
        self.ids.pos_graph.add_plot(self.setpoint_plot)
        self.ids.pos_graph.add_plot(self.act_pos_plot)

        # Assign colors to line plots
        self.plot.color = [0, 1, 0, 1]
        self.plot2.color = [1,0,0,1]
        self.plot3.color = [0,0,1,1]
        self.setpoint_plot.color = [0,1,0,1]
        self.act_pos_plot.color = [1,0,0,1]

        # Schedule Intervals
        self.graph_clock = Clock.schedule_interval(self.get_plot_points, (1/10))
        self.graph_clock.cancel()
        self.slider_clock = Clock.schedule_interval(self.pid_listener, (1 / 30))

        # initialize a que to store incoming serial coms
        self.q = Queue()
    # ---------- End Init ----------


    # ---------- PID Listener ----------
    # Checks to see if slider PID values have changed
    # If they have, send new value via serial
    def pid_listener(self, other):
        # Check P Value:
        if(self.p_val != float(self.ids.p_slider.value)):
            self.send_serial((",".join(["p", str(self.ids.p_slider.value)])), "")
            self.p_val = self.ids.p_slider.value

        # Check I Value:
        if(self.i_val != float(self.ids.i_slider.value)):
            self.send_serial((",".join(["i", str(self.ids.i_slider.value)])), "")
            self.i_val = self.ids.i_slider.value

        # Check D Value:
        if(self.d_val != float(self.ids.d_slider.value)):
            self.send_serial((",".join(["d", str(self.ids.d_slider.value)])), "")
            self.d_val = self.ids.d_slider.value
    # ---------- End PID Listener ----------


    # ---------- Coms Btn Press ----------
    # Turns serial communication on/off
    # also changes coms button color/text
    def coms_btn_press(self):
        # If coms are being turned off:
        if self.coms_on:
            self.coms_on = False
            self.ids.coms_btn.background_color = [1,.73,0,1]
            self.graph_clock.cancel()
            self.ids.coms_btn.text = "Begin Coms"
            self.serial_read = False
        # If coms are being turned on:
        else:
            self.coms_on = True
            self.ids.coms_btn.background_color = [0,.8,.42,1]
            self.graph_clock()
            self.ids.coms_btn.text = "End Coms"
            self.serial_read = True
            get_serial_thread = Thread(target=self.get_serial)
            get_serial_thread.daemon = True
            get_serial_thread.start()
    # ---------- Coms Btn Press ----------


    # ---------- Reset All ----------
    # Resets to starter PID parameters
    # and centers values
    def reset_all(self):
        # Reset Slider Values:
        self.ids.p_slider.value = 3.5
        self.ids.i_slider.value = 0
        self.ids.d_slider.value = 9

        # Pull ball to the center:
        self.send_serial(0, "PASS")
    # ---------- End Reset All ----------


    # ---------- Get Plot Points ----------
    # Clock scheduled method that checks queue
    # for new data points, and adds it to the
    # real time graphs
    def get_plot_points(self, other):
        # Reset error plots every 2000 data points
        if (len(self.plot_1_list) > 2000):
            self.plot_1_list = []
            self.plot_2_list = []
            self.plot_3_list = []

        # Reset position plots every 1000 data points
        if (len(self.actual_list)) > 1000:
            self.actual_list = []
            self.setpoint_list = []

        # If the queue has data, append it to the plot lists
        while not self.q.empty():
            temp_vals = self.q.get()
            self.plot_1_list.append(float(temp_vals[0]))
            self.plot_2_list.append(float(temp_vals[1]))
            self.plot_3_list.append(float(temp_vals[2]))
            self.setpoint_list.append((float(temp_vals[3]),float(temp_vals[4])))
            self.actual_list.append((float(temp_vals[5]), float(temp_vals[6])))

        # Append plot lists to mesh line plots
        self.plot.points = [(i, j) for i, j in enumerate(self.plot_1_list)]
        self.plot2.points = [(i, j) for i, j in enumerate(self.plot_2_list)]
        self.plot3.points = [(i, j) for i, j in enumerate(self.plot_3_list)]
        self.act_pos_plot.points = list(self.actual_list)
        self.setpoint_plot.points = self.setpoint_list[:]
    # ---------- End Get Plot Points ----------

    # ---------- Get Serial (Threaded) ----------
    # Grabs a line from serial comms
    # Should be a valued pair from arduino
    # Splits at comma and puts in queue
    def get_serial(self):
        try: # Try and open serial port
            if not self.ser.is_open:
                self.ser.open()

        except: # Serial Port isn't connected
            print("No Serial Found!")

        else: # If serial port is opened, start scanning for data
            while self.serial_read:
                # Data is sent as byte stream, we need to decode:
                new_info = self.ser.readline().decode('utf-8')[:-2].split(',')
                # Put new data in the queue for the GUI to display
                self.q.put(new_info)
    # ---------- End Get Serial ----------

    # ---------- Send Serial ----------
    # Sends commands over serial to change
    # machine's behaviour or tunings
    def send_serial(self, num, text, btn_id):
        try: # try to open serial and write command
            if not self.ser.is_open:
                self.ser.open()
            self.ser.write(str(num).encode('utf-8'))
            print(num, instr)

        except: # serial port isn't connected
            print("Serial Not Found")
            print(text)

        finally: # change button colors to show which is selected
            btn_id.background_color = [0,.8,.42,1]
            try:
                self.lit_button.background_color = [(192 / 255), (57 / 255), (43 / 255), 1]
            except: pass # exception will only be thrown on first button click
            self.lit_button = btn_id
    # ---------- End Send Serial ----------

    # ---------- Toggle Tabs ----------
    # Switches between position and error graphs
    def toggle_tabs(self):
        if self.ids.tab_panel.current_tab == self.ids.error_tab:
            self.ids.tab_panel.switch_to(self.ids.pos_tab)
        else: self.ids.tab_panel.switch_to(self.ids.error_tab)
    # ---------- End Toggle Tabs ----------

class pidApp(App):
    def build(self):
        return pid()

if __name__ == '__main__':
    mainApp = pidApp()
    mainApp.run()