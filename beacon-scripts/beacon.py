#! /usr/local/bin/python2
#-*- coding: utf-8 -*-

## Copyright 2011, 2012 Philip Crump M0DNY <pdc1g09@ecs.soton.ac.uk>

## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.

## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.

## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.


## Callsign of beacon <== CHANGE THIS
CALLSIGN = 'M0DNY-1'

# The beacon is only intended to reach an i-Gate, so a very limited path is needed
PATH = 'APRX22 via WIDE1-1 WIDE2-1'
#PATH = 'BEACON via WIDE1-1 WIDE2-1' # Suitable for remote areas

SYMBOL_TABLE = '/' # Primary Symbol Table
#SYMBOL = 'l' # Laptop
SYMBOL = '>' # Car
#SYMBOL = 'U' # Bus

## APRS Comment <== CHANGE THIS
COMMENT = 'www.thecraag.com'
COMMENT_PERIOD = 20 # Period of sending comment in minutes

BEACON_PERIOD = 2 # Period of sending position beacon in minutes

## Adds a timestamp to location data, only useful in very high network latency
## or low GPS signal environments.
TIMESTAMP = False # Not yet implemented

## Only valid for short (compressed) packets, this disables course/speed in favour of
## altitude reporting.
ALTITUDE = False # WARNING: Currently bugged. (Value reported is too large.)

## This outputs to file (for aprx beacon) instead of calling system 'beacon'
APRX = True # Standalone beacon by default
APRX_PATH = '/tmp/beacon.txt'

## Log to file - NOT YET IMPLEMENTED
## This will provide a log of all happennings (beacons, loss of gps, regain of gps,..)
##LOG_PATH = './beacon_log.txt'

## This enables extra stdout outputs for bug-hunting.
DEBUG = False

# Make sure the comment is sent 'at least' once every comment_period
REAL_COMMENT_PERIOD = (COMMENT_PERIOD - BEACON_PERIOD) + 1 # add 1 minute

import gps, os, time, math, threading, re

# Function to encode (shorten) the lat,lon values in the packets
def latlon_encode(value):
   div1 = math.floor(value/math.pow(91,3))
   remainder1 = math.fmod(value, math.pow(91,3))
   div2 = math.floor(remainder1/math.pow(91,2))
   remainder2 = math.fmod(remainder1, math.pow(91,2))
   div3 = math.floor(remainder2/91)
   remainder3 = math.fmod(remainder2, 91)
   return chr(math.trunc(div1+33)) + chr(math.trunc(div2+33)) + chr(math.trunc(div3+33)) + chr(math.trunc(remainder3+33))

# Thread that keeps a connection to gpsd, holds current GPS state dictionary
class GpsPoller(threading.Thread):
   def __init__(self):
       threading.Thread.__init__(self)
       self.session = gps.gps(mode=gps.WATCH_ENABLE)
       self.current_value = None
       self.stopped = False

   def stop(self): # Stop the thread at the next opportunity
       self.stopped = True

   def get_current_value(self): # Return GPS state dictionary
       return self.current_value

   def run(self):
       try:
            while not self.stopped:
                self.current_value = self.session.next() # Set GPS state dictionary
                time.sleep(0.2) # Pauses, polls gpsd 5x per second
       except StopIteration:
            pass

# Main beacon thread, gets position from GpsPoller
class Beaconer(threading.Thread):
   def __init__(self):
       threading.Thread.__init__(self)
       #logfile = open(LOG_PATH, 'a')
       #logfile.write('Log Opened.')
       self.status = "Starting"
       self.stopped = False
       self.last_lat = 0
       self.last_lon = 0
       self.no_gps_timer = 0
       self.beacon_timer = 0
       self.comment_timer = 0
       self.gps_status = ''
       self.last_beacon = '' # DEBUG
       self.beacon_period = BEACON_PERIOD*60 # convert to seconds
       self.comment_period = REAL_COMMENT_PERIOD*60 # convert to seconds

   def stop(self): # Stop the thread at the next opportunity
       self.stopped = True
       #logfile.write('Log Closed.')
       #logfile.close()

   def get_last_beacon(self): # Provides a timer since last beacon sent
       return self.beacon_timer
       
   def get_last_comment(self): # Provides a timer since last comment sent
       return self.comment_timer
       
   def get_last_fix(self): # Provides a timer since the last known 3D fix
       return self.no_gps_timer
       
   def get_beacon_period(self): # Returns the beacon interval in seconds
       return self.beacon_period
       
   def get_comment_period(self): # Returns the comment interval in seconds
       return self.comment_period
       
   def get_status(self): # Returns the current gps fix status
       return self.status
       
   def get_lat(self): # Returns last known latitude
       return self.last_lat
       
   def get_lon(self): # Returns last known longitude
       return self.last_lon
       
   def get_gps_debug(self): # Returns full gps object
       return self.gps_status
       
   def get_beacon_debug(self): # Returns last aprs string sent 
      return self.last_beacon
       
   def runbeacon(self):
       if APRX: # APRX - Always send comment
          beacon_string = self.short_beacon()+COMMENT
          self.comment_timer = time.time()
       elif math.trunc(time.time() - self.comment_timer)>=self.comment_period:
          beacon_string = self.short_beacon()+COMMENT
          self.comment_timer = time.time()
       else:
          beacon_string = self.short_beacon()
       self.last_beacon = beacon_string
       if APRX:
          self.save_beacon(beacon_string)
       else:
          self.send_beacon(beacon_string)
       self.beacon_timer = time.time()
       
   def send_beacon(self,aprs_string): # Sends beacon using system 'beacon'
       system_string = "/usr/sbin/beacon -c "+CALLSIGN+" -d '"+PATH+"' -s sm0 "+re.escape(aprs_string)
       os.system(system_string)
       #logfile.write('Beacon sent: ', aprs_string)

   def save_beacon(self,aprs_string): # Used for aprx, saves to file
       fout = open(APRX_PATH, "w")
       fout.truncate()
       fout.write(aprs_string)
       fout.close()
       #logfile.write('Beacon saved: ', aprs_string)

   def short_beacon(self): # Compressed beacon format
      if TIMESTAMP:
         aprs_prefix = '/' # According to the APRS spec (Position, timestamp, no messaging)
      else:
         aprs_prefix = '!' # According to the APRS spec (Position, no timestamp, no messaging)
      ## Sort out time strings
      utctime = str(self.gps_status['time'])
      day = utctime[8::10]
      hour = utctime[11:13]
      minute = utctime[14:16]
      second = utctime[17:19]
      ## Sort out Latitude and Longitude
      lat_bytes = latlon_encode(380926*(90-self.gps_status['lat']))
      lon_bytes = latlon_encode(190463*(180+self.gps_status['lon']))
      short_packet_string = aprs_prefix + SYMBOL_TABLE
      if TIMESTAMP:
         short_packet_string += hour + minute + second + 'h'
      short_packet_string += lat_bytes + lon_bytes + SYMBOL
      if ALTITUDE:
         type_byte = chr(33+int('00110010',2)) # Set to GGA to enable altitude
         alt_value = math.log(math.trunc(self.gps_status['alt']*3.28))/math.log(1.002)
         alt_div = math.floor(alt_value/91)
         alt_remainder = math.fmod(math.trunc(alt_value), 91)
         alt_bytes = chr(33+math.trunc(alt_div)) + chr(33+math.trunc(alt_remainder))
         short_packet_string += alt_bytes + type_byte
      else:
         type_byte = chr(33+int('00111010',2)) # Set to RMC to enable course/speed
         course_byte = chr(33+math.trunc(self.gps_status['track']/4))
         speed_byte = chr(33+math.trunc(math.log((self.gps_status['speed']*1.9438)+1)/math.log(1.08)))
         short_packet_string += course_byte + speed_byte + type_byte
      return short_packet_string

   def run(self):
       try:
            time.sleep(0.3) # Allow gpsd connection time to return status
            while not self.stopped:
                self.gps_status = gpsp.get_current_value()
                if self.gps_status['class']=='TPV': # Do we have a GPS fix?
                   if self.gps_status['mode']==3: # Do we have 3D fix? (Not available unless in TPV)
                      if self.status != "3D": # New to this state
                         self.status = "3D"
                         #logfile.write('3D Fix!')
                      self.no_gps_timer = 0
                      self.last_lat = self.gps_status['lat']
                      self.last_lon = self.gps_status['lon']
                      if APRX:
                         self.runbeacon()
                      elif math.trunc(time.time() - self.beacon_timer)>=self.beacon_period:
                         self.runbeacon()
                   else: # Only 2D GPS fix
                      if self.status != "2D": # New to this state
                         self.status = "2D"
                         #logfile.write('2D Fix.')
                      if self.no_gps_timer==0: self.no_gps_timer = time.time()
                elif self.gps_status['class']=='DEVICE' or self.gps_status['class']=='WATCH': # Check for 'No Device'
                   if self.status != "No Device": # New to this state
                      self.status = "No Device"
                      #logfile.write('No Device.')
                   if self.no_gps_timer==0: self.no_gps_timer = time.time()
                else: # No GPS fix
                   if self.status != "No Fix": # New to this state
                      self.status = "No Fix"
                      #logfile.write('No Fix.')
                   if self.no_gps_timer==0: self.no_gps_timer = time.time()
                time.sleep(1)
       except StopIteration:
            pass

## Main foreground process, just draws to the screen, grabs info from Beaconer thread
if __name__ == "__main__":
   try:
      gpsp = GpsPoller()
      gpsp.start()
      shout = Beaconer()
      shout.start()
      while 1:
         os.system('clear') # Clear the screen
         # Now draw screen depending on GPS status
         if shout.get_status()=='3D':
            print 'Got 3D Fix!'
            print 'Lat: ', shout.get_lat()
            print 'Lon: ', shout.get_lon()
         elif shout.get_status()=='2D': # Only 2D GPS fix
            print 'Got 2D, really want 3D fix..'
         elif shout.get_status()=='No Device':
            print 'GPS Device not present'
         elif shout.get_status()=='No Fix': 
            print 'Waiting for fix..'
         elif shout.get_status()=='Starting': # Beacon thread not yet initialised
            print 'Beacon starting up..'
         else:
            print shout.get_status() # Provides debug incase status isn't recognised
         print ''
         if shout.get_last_beacon()==0: # No beacon yet sent
            print 'No beacon yet sent.'
         else:
            if APRX:
               print APRX_PATH, ' updated ', math.trunc(time.time() - shout.get_last_beacon()), 'seconds ago.'
            else:
               print 'Beacon timer: ', math.trunc(time.time() - shout.get_last_beacon()), '/', shout.get_beacon_period(), ' seconds.'
         print ''
         print 'Current comment: ', COMMENT
         if shout.get_last_comment()==0: # No comment yet sent
            print 'No comment yet sent'
         else:
            if APRX==False: ## Can I do this with '!APRS' ?
               print 'Comment timer: ', math.trunc((time.time() - shout.get_last_comment())/60), '/', shout.get_comment_period()/60, ' minutes.'
         if DEBUG:
            print shout.get_beacon_debug()
            print shout.get_last_beacon()
            print shout.get_gps_debug()
         time.sleep(1) # Give the CPU some time to breathe
   except KeyboardInterrupt: # Catch Ctrl+C, stop both threads
      shout.stop()
      gpsp.stop()
      pass
