# Copyright (c) 2024 FORCERA, LDA
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

import drone_utils
import rtsp_utils
import threading
import time

class module_init:
    def __init__(self, drone_obj, rtsp_port, free_flight=False):
        '''
        :param drone_ip: object of the anafi drone
        :param rtsp_port: the name to be defined for the rtsp server
        :param free_flight: flag to start threads related to pre-defined route of the drone
        '''
        self.mission_data = drone_obj #object with the drone object
        self.rtsp_port = rtsp_port #port of the original rtsp stream
        self.free_flight = free_flight #define if the drone will follow a pre-defined route or not

        #drone utils obj to acquire telemetry data
        self.data_acq_obj = drone_utils.data_acquisition(self.mission_data)

        if not self.free_flight:
            self.flight_obj = drone_utils.flight_control(self.mission_data, move_by=4)
            self.flight_thread = None #thread to process the flight commands of the drone

        self.publish_thread = None #thread to process the publishing of telemetry data

        #RTSP stream processing setup
        self.stream_obj = rtsp_utils.rtsp_processing(self.data_acq_obj.mission_data, self.rtsp_port) #open the rtsp url and start processing
        self.stream_thread = None #thread to process all the stream information coming from the rtsp
        self.telemetry_thread = None #thread to process all incoming telemetry data in a queue

    def run_threads(self):
        '''
        :return: runs the threads
        '''
        stream_event = threading.Event() #create the event to manage thread condition
        print('[rtsp_utils] Setting up the stream...')
        time.sleep(30) #wait 30 seconds before writing to redis to set up the stream side without delay

        #Define the threads
        self.stream_thread = threading.Thread(target=self.stream_obj.run_stream, args=(stream_event,)) #define the target for the thread
        self.telemetry_thread = threading.Thread(target=self.stream_obj.write_telemetry_data)
        self.publish_thread = threading.Thread(target=self.data_acq_obj.begin_register, args=(stream_event,)) #define the target for the second thread

        if not self.free_flight:
            self.flight_thread = threading.Thread(target=self.flight_obj.run_flight, args=(stream_event,))

        #Start the threads
        self.stream_thread.start() #start the stream thread
        self.telemetry_thread.start() #start the telemetry data read thread
        self.publish_thread.start() #start the publish thread

        if not self.free_flight:
            self.flight_thread.start() #start the flight thread

        #Wait for the threads to end
        if not self.free_flight:
            self.flight_thread.join() #wait for the flight thread to end

        self.publish_thread.join() #wait for the publish thread to end
        self.telemetry_thread.join() #wait for the telemetry thread to end
        self.stream_thread.join() #wait for the stream thread to end
