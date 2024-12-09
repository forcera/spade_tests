# Copyright (c) 2024 FORCERA, LDA
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

import datetime
from datetime import timedelta
import numpy as np
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import os
import olympe
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
import json

class position_register(olympe.EventListener):
    default_queue_size = 500 #default queue size for register methods
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.drone_obj = args[0]
        self.init_lat = self.drone_obj.get_state(olympe.messages.ardrone3.PilotingState.GpsLocationChanged)['latitude']
        self.init_long = self.drone_obj.get_state(olympe.messages.ardrone3.PilotingState.GpsLocationChanged)['longitude']
        self.last_update = datetime.datetime.now()
        
        load_dotenv()
        self.mqtt_broker = {
            "host": os.getenv('FRAMEWORK_LOCAL_MQTT_BROKER'),
            "keepalive": 60,
            "port": int(os.getenv('FRAMEWORK_MQTT_PORT')),  # broker port
            "telemetry_topic": os.getenv('FRAMEWORK_DRONE_MQTT_TOPIC'),  # mqtt topic
            "camera_topic": os.getenv('FRAMEWORK_DRONE_CAMERA_TOPIC')  # camera info topic
        }
        self.mqtt_client = mqtt.Client()  #mqtt client
        self.Ts = float(os.getenv('FRAMEWORK_TELEMETRY_SAMPLING_TIME')) #sampling time

    def haversine(self, curr_lat, curr_long):
        '''
        :param curr_lat: current registered latitude in degrees
        :param curr_long: current registered longitude in degrees
        :return: the distante between the current sample and the first sample in meters
        '''

        r = 6371 #earth radius in km
        d_lat = np.deg2rad(curr_lat) - np.deg2rad(self.init_lat) #latitude diff in rad
        d_long = np.deg2rad(curr_long) - np.deg2rad(self.init_long) #longitude diff in rad
        root_part = (np.sin(d_lat/2)**2) + np.cos(np.deg2rad(self.init_lat))*np.cos(np.deg2rad(curr_lat))*np.sin(d_long/2)**2
        out_part = 2*r*np.arcsin(np.sqrt(root_part))
        return out_part*1000

    #handle attitude, altitude and position messages and register the data
    @olympe.listen_event(olympe.messages.ardrone3.PilotingState.FlyingStateChanged(_policy="wait")
        | olympe.messages.ardrone3.PilotingState.AttitudeChanged(_policy="wait")
        | olympe.messages.ardrone3.PilotingState.AltitudeAboveGroundChanged(_policy="wait")
        | olympe.messages.ardrone3.PilotingState.GpsLocationChanged(_policy="wait"))
    def onEventChanged(self, event, scheduler):
        '''
        :param event: object of the event registered as changed
        :param scheduler: scheduler object of the olympe SDK
        :return: publishes the acquired telemetry data given the sampling rate self.Ts
        '''
        curr_datetime_form = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # convert datetime object to str
        last_update_form = self.last_update.strftime('%Y-%m-%d %H:%M:%S')  # convert datetime object to str
        if curr_datetime_form != last_update_form: #avoid registering the same sample more than one time
            att_dict = self.drone_obj.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) #attitude data dictionary
            pos_dict = self.drone_obj.get_state(olympe.messages.ardrone3.PilotingState.GpsLocationChanged) #position data dictionary
            speed_dict = self.drone_obj.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged) #speed data dictionary

            #camera data dictionary might not be initialized
            try:
                camera_dict = self.drone_obj.get_state(olympe.messages.camera2.Event.State)
            except:
                camera_dict = {'zoom': '', 'active': False}

            event_message = {'time': datetime.datetime.now(),
                             'latitude': pos_dict['latitude'],
                             'longitude': pos_dict['longitude'],
                             'altitude': pos_dict['altitude'],
                             'roll': np.round(att_dict['roll'],4),
                             'pitch': np.round(att_dict['pitch'],4),
                             'yaw': np.round(att_dict['yaw'],4),
                             'speedX': np.round(speed_dict['speedX'],4),
                             'speedY': np.round(speed_dict['speedY'],4),
                             'speedZ': np.round(speed_dict['speedZ'],4),
                             'camera_status': camera_dict['active']}

            # Handle the message that will be sent to the camera_info topic
            camera_dict.update({'time': event_message['time']})  # add the timestamp to the camera dictionary
            camera_dict.move_to_end('time', last=False)  # rearange the timestamp as the first entry

            # Manage the controlled sampling time to publish telemetry data
            delta_t = datetime.datetime.now() - self.last_update  # diff. between current and last update
            if delta_t >= datetime.timedelta(seconds=self.Ts):
                self.mqtt_publisher(event_message, self.mqtt_broker["telemetry_topic"])  # sending telemetry mqtt message
                self.mqtt_publisher(camera_dict, self.mqtt_broker["camera_topic"])  # sending camera info mqtt message
                self.last_update = datetime.datetime.now()  # update the last_update published variable

    def json_serial(self, obj):  # function to handle datetime data entries (convert to json serializable)
        '''
        :param obj: json dictionary with the output data of the simulator
        :return: deals with timestamp data format to be serialized
        '''
        if isinstance(obj, datetime.datetime):
            return obj.isoformat()

    def mqtt_publisher(self, event_message, topic):
        '''
        :param event_message: telemetry message in json-like format to be published in the given topic
        :param topic: MQTT topic to which the message will be sent
        '''
        #Handling connection issues
        if self.mqtt_client.connect(host=self.mqtt_broker["host"],
                                    port=self.mqtt_broker["port"],
                                    keepalive=self.mqtt_broker["keepalive"]) != 0:
            raise TypeError("[drone_utils] Could not connect to the MQTT broker!")

        payload = json.dumps(event_message, default=self.json_serial) #event message to be sent via mqtt
        self.mqtt_client.publish(topic, payload, qos=1) #send the message at least one time
        self.mqtt_client.disconnect() #disconnect from the broker

class data_acquisition:
    def __init__(self, drone_obj):
        '''
        :param drone_obj: object of the anafi drone
        :param time_init: initial time of acquisition in [s]
        '''
        self.mission_data = drone_obj #class with the olympe drone info

        if self.mission_data.drone.connect(retry=5):
            print('[drone_utils] Connected to the drone!')
        else:
            raise TypeError("[drone_utils] Connection to the drone failed!")

        #Handle SkyController connections
        self.hardware_info = os.getenv('FRAMEWORK_CONNECT_HW')
        self.hardware_info = self.hardware_info.lower()
        print(self.hardware_info)
        if self.hardware_info == 'skycontroller':
            self.mission_data.drone(setPilotingSource(source="Controller")).wait()

        #Telemetry data setup
        self.position_event_register = position_register(self.mission_data.drone)

    def begin_register(self, flight_flag_event):
        '''
        :param flight_flag_event: threading event to monitor when the drone lands
        '''
        self.position_event_register.subscribe()  #subscribe to the event listener

        while True:
            offline_flag = flight_flag_event.is_set()
            if offline_flag:
                #self.position_event_register.unsubscribe()  # unsubscribe to the event messages
                if self.mission_data.drone.disconnect():
                    print('[drone_utils] Disconnected and mission complete!')
                    break
                else:
                    raise TypeError("[drone_utils] Disconnection to the drone failed!")

class flight_control:
    def __init__(self, drone_obj, move_by=1):
        '''
        :param drone_obj: object of the anafi drone
        :param moveby: number of loops in the flight path
        '''
        self.mission_data = drone_obj  # class with the olympe drone info

        if self.mission_data.drone.connect(retry=5):
            print('[drone_utils.flight_control] Connected to the drone!')
        else:
            raise TypeError("[drone_utils.flight_control] Connection to the drone failed!")

        self.move_by = move_by

    def drone_hover(self):
        print('[drone_utils.flight_control] Drone taking off...')
        self.mission_data.drone(olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="hovering", _policy="check")
                                    | olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="flying", _policy="check")
                                    | (olympe.messages.ardrone3.GPSSettingsState.GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                                       >> (olympe.messages.ardrone3.Piloting.TakeOff(_no_expect=True) & olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="hovering", _timeout=10,
                                                                                         _policy="check_wait")))).wait()
        print('[drone_utils.flight_control] Drone took off!')

    def flight_path(self):
        print('[drone_utils.flight_control] Running pre-defined drone route...')
        self.mission_data.drone(olympe.messages.ardrone3.Piloting.moveBy(self.move_by, 0, 0, 0)).wait().success()
        self.mission_data.drone(olympe.messages.ardrone3.Piloting.Landing() >> olympe.messages.ardrone3.PilotingState.FlyingStateChanged(state="landed")).wait().success()

    def run_flight(self, flight_flag_event):
        self.drone_hover() #hover the drone
        self.flight_path()
        flight_flag_event.set()  #set the event to communicate other trheads to go offline
        print('[drone_utils.flight_control] Flight routine complete!')
