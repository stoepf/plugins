#!/usr/bin/env python
#########################################################################
# Copyright 2014 Marcel Tiews marcel.tiews@gmail.com
# Modified 2014-2017 by René Jahncke aka Tom-Bom-badil @ github.com
# Modified 2018 Stefan Hauf sth@online.de
#########################################################################
# Helios-TCP-Plugin for SmartHome.py. http://mknx.github.io/smarthome/
#
# This plugin is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This plugin is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this plugin. If not, see <http://www.gnu.org/licenses/>.
#########################################################################

import sys
import serial
import logging
import socket
import threading
import struct
import time
import datetime
import array
import re
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from lib.model.smartplugin import SmartPlugin

SLAVE_ID = 180
FIRST_REGISTER_ADDR = 0x01

CONST_MAP_VARIABLES_TO_ID = {
#        "power_state"     : {"variable" : 'v, 'type': 'bit',          'bitposition':  0, 'read': True, 'write': True  },
#        "max_fanspeed"    : {"varid" : 0xA5, 'type': 'fanspeed',     'bitposition': -1, 'read': True, 'write': True  },
#        "min_fanspeed"    : {"varid" : 0xA9, 'type': 'fanspeed',     'bitposition': -1, 'read': True, 'write': True  },
#        "bypass_temp"     : {"varid" : 0xAF, 'type': 'temperature',  'bitposition': -1, 'read': True, 'write': True  },
#        "bypass_disabled" : {"varid" : 0xA3, 'type': 'bit',          'bitposition':  3, 'read': True, 'write': True  },
#        "heating_on_off"  : {"varid" : 0x70, 'type': 'bit',          'bitposition':  7, 'read': True, 'write': True  },
#        "heating_setpoint": {"varid" : 0xA7, 'type': 'temperature',  'bitposition': -1, 'read': True, 'write': True  },
#        "frost_stop"      : {"varid" : 0xA8, 'type': 'temperature',  'bitposition': -1, 'read': True, 'write': True  },
#        "cell_defrost"    : {"varid" : 0xB2, 'type': 'dec',          'bitposition': -1, 'read': True, 'write': True  },
#        "boost_mode"      : {"varid" : 0xAA, 'type': 'bit',          'bitposition':  5, 'read': True, 'write': True  },
#        "boost_on"        : {"varid" : 0x71, 'type': 'bit',          'bitposition':  5, 'read': True, 'write': True  },
#        "boost_status"    : {"varid" : 0x71, 'type': 'bit',          'bitposition':  6, 'read': True, 'write': False },
#        "boost_remaining" : {"varid" : 0x79, 'type': 'dec',          'bitposition': -1, 'read': True, 'write': False },
#        "fan_in_on_off"   : {"varid" : 0x08, 'type': 'bit',          'bitposition':  3, 'read': True, 'write': True  },
#        "fan_in_percent"  : {"varid" : 0xB0, 'type': 'dec',          'bitposition': -1, 'read': True, 'write': True  },        
#        "fan_out_on_off"  : {"varid" : 0x08, 'type': 'bit',          'bitposition':  5, 'read': True, 'write': True  },
#        "fan_out_percent" : {"varid" : 0xB1, 'type': 'dec',          'bitposition': -1, 'read': True, 'write': True  },   
#        "clean_filter"    : {"varid" : 0xAB, 'type': 'dec',          'bitposition': -1, 'read': True, 'write': True  },
        "fanspeed"        : {"variable" : 'v00102', 'type': 'dec', 'pos': 0, 'count': 5, 'read': True, 'write': True  },
        "outside_temp"    : {"variable" : 'v00104', 'type': 'dec', 'pos': 0, 'count': 8, 'read': True, 'write': False },
        "incoming_temp"   : {"variable" : 'v00105', 'type': 'dec', 'pos': 0, 'count': 8, 'read': True, 'write': False },
        "exhaust_temp"    : {"variable" : 'v00106', 'type': 'dec', 'pos': 0, 'count': 8, 'read': True, 'write': False },
        "inside_temp"     : {"variable" : 'v00107', 'type': 'dec', 'pos': 0, 'count': 8, 'read': True, 'write': False },
        "device_error"    : {"variable" : 'v01123', 'type': 'dec', 'pos': 0, 'count': 9, 'read': True, 'write': False }
    }

class HeliosException(Exception):
    pass


class HeliosBase(SmartPlugin):

    PLUGIN_VERSION = "0.0.1"
    ALLOW_MULTIINSTANCE = False

    def __init__(self, ip='192.168.87.6'):
        self.logger = logging.getLogger(__name__)
        self._ip = ip
        self._is_connected = False
        self._port = False
        self._lock = threading.Lock()
     
    def connect(self):
        if self._is_connected and self._port:
            return True
            
        try:
            self.logger.debug("Helios: Connecting...")
            self._port = ModbusTcpClient(self._ip, 502)
            self._is_connected = True
            return True
        except:
            self.logger.error("Helios: Could not open {0}.".format(self._ip))
            return False
        
    def disconnect(self):
        if self._is_connected and self._port:
            self.logger.debug("HeliosBase: Disconnecting...")
            self._port.close()
            self._is_connected = False
            
    def _convertFromRawValue(self, varname, rawvalue):
        value = None
        vardef = CONST_MAP_VARIABLES_TO_ID[varname]
        
        if vardef["type"] == "bit":
            value = rawvalue >> vardef["pos"] & 0x01
        elif vardef["type"] == "dec": #  decimal value
            value = rawvalue
                   
        return value        

    def _convertFromValue(self, varname, value, prevvalue):
        rawvalue = None
        vardef = CONST_MAP_VARIABLES_TO_ID[varname]
        
        if vardef["type"] == "bit":
            # for bits we have to keep the other bits of the byte (previous value)
            if value in (True,1,"true","True","1","On","on"):
                rawvalue = prevvalue | (1 << vardef["pos"])
            else:
                rawvalue = prevvalue & ~(1 << vardef["pos"])
        elif vardef["type"] == "dec": #  decimal value
            rawvalue = int(value)
            
        return rawvalue        
        
    def writeValue(self,varname, value):
        if CONST_MAP_VARIABLES_TO_ID[varname]["write"] != True:
            self.logger.error("Helios: Variable {0} may not be written!".format(varname))
            return False 
        success = False
        
        self._lock.acquire()
        try:
            # if we have got to write a single bit, we need the current (byte) value to
            # reproduce the other bits...
#            if CONST_MAP_VARIABLES_TO_ID[varname]["type"] == "bit":
#                currentval = None
#                # Send poll request
#                # Read response
#                #    CONST_MAP_VARIABLES_TO_ID[varname]["variable"]
#                value = value
#                if currentval == None:
#                    self.logger.error("Helios: Sending value to ventilation system failed. Can not read current variable value '{0}'."
#                        .format(varname))
#                    return False
#                rawvalue = self._convertFromValue(varname, value, currentval)
#            else:    
#                #rawvalue = self._convertFromValue(varname, value, None)
            rawvalue = str(value)

            # send the new value    
            if rawvalue is not None:
                # Writing value
                builder = BinaryPayloadBuilder()
                builder.add_string(CONST_MAP_VARIABLES_TO_ID[varname]["variable"] + '=' + rawvalue + '\0')
                payload = builder.build()
                self._port.write_registers(FIRST_REGISTER_ADDR, payload,
                                   skip_encode=True, unit=SLAVE_ID)
                success = True
                
            else:
                self.logger.error("Helios: Sending value to ventilation system failed. Can not convert value '{0}' for variable '{1}'."
                   .format(value,varname))
                success = False
        except Exception as e:
                self.logger.error("Helios: Exception in writeValue() occurred: {0}".format(e))
        finally:
            self._lock.release()
   
        return success
            
    def readValue(self,varname):
        if CONST_MAP_VARIABLES_TO_ID[varname]["read"] != True:
            self.logger.error("Variable {0} may not be read!".format(varname))
            return False
        value = None
        
        self._lock.acquire()
        try:
            self.logger.debug("Helios: Reading value: {0}".format(varname)) 
            # Send poll request
            builder = BinaryPayloadBuilder()
            builder.add_string(CONST_MAP_VARIABLES_TO_ID[varname]["variable"] + '\0')
            payload = builder.build()
            self._port.write_registers(FIRST_REGISTER_ADDR, payload,
                                   skip_encode=True, unit=SLAVE_ID)
            # Read response
            rtr = CONST_MAP_VARIABLES_TO_ID[varname]["count"] + 3
            if rtr < 8:
                rtr = 8
            result = self._port.read_holding_registers(
                FIRST_REGISTER_ADDR, rtr, unit=SLAVE_ID)
            output = BinaryPayloadDecoder.fromRegisters(
                result.registers).decode_string(rtr)
            output = re.sub(u'([\x00])', "", output.decode('utf-8'))
            reg, value = output.split('=')

            if value is not None:
                raw_value = value
                value = self._convertFromRawValue(varname,value)
                self.logger.debug("Value for {0} ({1}) received: {2} --> converted = {3}"
                    .format(varname, CONST_MAP_VARIABLES_TO_ID[varname]["variable"],
                    raw_value, value)
                ) 
            else:   # logging in debug only, so we stop spamming log file (noise on the bus seems to be normal)
                self.logger.debug("Helios: No valid value for '{0}' from ventilation system received."
                    .format(varname)
                ) 
        except Exception as e:
                self.logger.error("Helios: Exception in readValue() occurred: {0}".format(e))
        finally:
            self._lock.release()
   
        return value

    
class HeliosTCP(HeliosBase): 
    _items = {}
    
    def __init__(self, smarthome, ip, cycle=300):
        HeliosBase.__init__(self, ip)
        self._sh = smarthome
        self._cycle = int(cycle)
        self._alive = False
        
    def run(self):
        self.connect()
        self._alive = True
        self._sh.scheduler.add('Helios', self._update, cycle=self._cycle)

    def stop(self):
        self.disconnect()
        self._alive = False

    def parse_item(self, item):
        if 'helios_var' in item.conf:
            varname = item.conf['helios_var']
            if varname in CONST_MAP_VARIABLES_TO_ID.keys():
                self._items[varname] = item
                return self.update_item
            else:
                self.logger.warning("Helios: Ignoring unknown variable '{0}'".format(varname))
        
    def update_item(self, item, caller=None, source=None, dest=None):
        if caller != 'HeliosTCP':
            self.writeValue(item.conf['helios_var'], item()) 
        
    def _update(self):
        self.logger.debug("Helios: Updating values")
        for var in self._items.keys():
            val = self.readValue(var)
            if val != None:
                self._items[var](val,"HeliosTCP")

   
def main():
    import argparse 
    
    parser = argparse.ArgumentParser(
    description="Helios ventilation system commandline interface.",
    epilog="Without arguments all readable values using default ip will be retrieved.",
    argument_default=argparse.SUPPRESS)
    parser.add_argument("-t", "--ip", dest="port", default="192.168.87.6", help="IP of device to use")
    parser.add_argument("-r", "--read", dest="read_var", help="Read variables from ventilation system")
    parser.add_argument("-w", "--write", dest="write_var", help="Write variable to ventilation system")
    parser.add_argument("-v", "--value", dest="value", help="Value to write (required with option -v)")
    parser.add_argument("-d", "--debug", dest="enable_debug", action="store_true", help="Prints debug statements.")
    args = vars(parser.parse_args())
 
    if "write_var" in args.keys() and "value" not in args.keys():
        parser.print_usage()
        return

    logger.setLevel(logging.DEBUG)

    try:
        helios = HeliosBase(args["port"])
        helios.connect()
        if not helios._is_connected:
            raise Exception("Not connected")
        
        if "read_var" in args.keys():
            print("{0} = {1}".format(args["read_var"],helios.readValue(args["read_var"])))
        elif "write_var" in args.keys():
            helios.writeValue(args["write_var"],args["value"])
        else:
            for var in CONST_MAP_VARIABLES_TO_ID.keys():
                print("{0} = {1}".format(var,helios.readValue(var)))
    except Exception as e:
        print("Exception: {0}".format(e))
        return 1
    finally:
        if helios:
            helios.disconnect()

if __name__ == "__main__":
    sys.exit(main())        
