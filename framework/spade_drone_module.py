# Copyright (c) 2024 FORCERA, LDA
# 
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0

import module_utils
import os
import olympe
from dotenv import load_dotenv
from distutils.util import strtobool

class hardware_data:
    def __init__(self, hardware_ip):
        '''
        :param drone_ip: the IP of the anafi drone in string format
        '''
        olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})
        self.HW_IP = hardware_ip
        print(hardware_ip)
        self.hardware_info = os.getenv('FRAMEWORK_CONNECT_HW')
        self.hardware_info = self.hardware_info.lower()
        if self.hardware_info == 'drone':
            self.drone = olympe.Drone(self.HW_IP)
        elif self.hardware_info == 'skycontroller':
            self.drone = olympe.SkyController(self.HW_IP)

load_dotenv()
hardware_ip = os.getenv('FRAMEWORK_HW_IP')
free_flight_flag = os.getenv('FRAMEWORK_FREE_FLIGHT_FLAG')
mission_obj = module_utils.module_init(hardware_data(hardware_ip), 554, free_flight=bool(strtobool(free_flight_flag)))
mission_obj.run_threads()
