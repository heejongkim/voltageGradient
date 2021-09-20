## last updated: 2021.09.18 - 2
import warnings
import re
import win32com.client as win32  # pip install pywin32
import serial                    # pip install pyserial=3.5
import serial.tools.list_ports
import datetime
from datetime import timedelta
from time import sleep
import time
import pandas as pd
import csv
import os
from apscheduler.schedulers.background import BackgroundScheduler
# pip install apscheduler=3.7.0

def getPowerSupplyOutputLine(serial_info):
    with serial.Serial(serial_info['port'], serial_info['baudRate']) as ser:
        # add time mark in front
        return('\t'.join([str(datetime.datetime.now()), ser.readline().decode('ascii')]))

def setPowerSupplyVoltage(serial_info, voltage):
    with serial.Serial(serial_info['port'], serial_info['baudRate']) as ser:
        ser.write(("V="+str(voltage)).encode('ascii'))
    
def setPowerSupplyCurrent(serial_info, current):
    with serial.Serial(serial_info['port'], serial_info['baudRate']) as ser:
        ser.write(("I="+str(current)).encode('ascii'))
    
def setPowerSupplyPolarity(serial_info, polarity):
    with serial.Serial(serial_info['port'], serial_info['baudRate']) as ser:
        ser.write(("P="+str(polarity)).encode('ascii'))
    
def scheduleStepVoltage(serial_info, scheduler, volt_gradient_table, gradient_log_filename):
    # get the start time of acquisition (RT=0)
    now = datetime.datetime.utcnow()

    # schedule voltage, currrent, and polarity based on the table
    for index, row in volt_gradient_table.iterrows():
        #print(row["time(min)"], row["voltage(V)"], row["current(A)"])
        # set the RT point
        #run_at = now + timedelta(minutes= float(row["time(min)"]))
        # set proper type for each value
        current = round(float(row["current(A)"]),2)
        voltage = round(float(row["voltage(V)"]),2)
        polarity = int(row["polarity"])
        # schedule the current, voltage, and polarity with RT point
        scheduler.add_job(setPowerSupplyVoltage, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=3.0), args = [serial_info, voltage])
        scheduler.add_job(setPowerSupplyCurrent, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=6.0), args = [serial_info, current])
        scheduler.add_job(setPowerSupplyPolarity, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=9.0), args = [serial_info, polarity])
        # schedule the log write to prevent blocking the port
        for log_time in range(12, 24, 2):
            scheduler.add_job(writeGradientLog, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=log_time), args = [serial_info, gradient_log_filename])


def readGradientTable(volt_gradient_filename):
    # get the voltage scheme from VOLT csv file
    volt_gradient_table = pd.read_csv(volt_gradient_filename, skipinitialspace=True)
    volt_gradient_table = volt_gradient_table.apply(pd.to_numeric,downcast='float') # change all to float
    
    return(volt_gradient_table)

def writeGradientLog(serial_info, gradient_log_filename):
    with open(gradient_log_filename,"a") as f:
        writer = csv.writer(f)
        writer.writerow([getPowerSupplyOutputLine(serial_info)])

def arduinoPortDetect():
    # auto detection of arduino port -- https://stackoverflow.com/questions/24214643/python-to-automatically-select-serial-ports-for-arduino
    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'Arduino' in p.description  # may need tweaking to match new arduinos
    ]
    if not arduino_ports:
        raise IOError("No Arduino found")
    if len(arduino_ports) > 1:
        warnings.warn('Multiple Arduinos found - using the first')

    return(arduino_ports[0])

def extractVoltageSchemeFilename(filename):
    split_raw_filename_by_sp_chars = re.split(r'[`\-=~!@#$%^&*()_+\[\]{};\'\\:"|<,./<>?]', filename)  # https://stackoverflow.com/questions/21023901/how-to-split-at-all-special-characters-with-re-split
    volt_string = [s for s in split_raw_filename_by_sp_chars if "volt" in s]  # https://stackoverflow.com/questions/4843158/check-if-a-python-list-item-contains-a-string-inside-another-string
    print(volt_string)
    if volt_string:
        if os.path.isfile(volt_string[0] + ".csv"):
            return(volt_string[0] + ".csv")
        else:
            return([]) # handling the case where it's specified but doesn't exist
    else:
        return(volt_string) # return empty list

### INITIALIZATION
# PowerSupply Communication Info
serial_info = {"port": arduinoPortDetect(), "baudRate": 9600}

# xcalibur connection
q = win32.Dispatch("AcqServer.AcqInterfaceSupport1")

# scheduler start
scheduler = BackgroundScheduler(daemon=True, timezone="utc")
scheduler.start()

while True:
    # printing current power supply status while it's idle
    print(getPowerSupplyOutputLine(serial_info))
    # when acquisition starts:
    if(q.GetRunManagerStatusFromCom == u"Acquiring"):
        # Make sure that Power Supply is enabled
        print("Voltage supply Enabled ...")
        with serial.Serial(serial_info['port'], serial_info['baudRate']) as ser:
            ser.write(("R=1").encode('ascii'))
        # Acquisition Start from here
        print("Acquisition Starts ...")
        # retrieve raw filename from xcalibur
        raw_filename = q.GetMangledRawFilename
        # retrieve voltage scheme
        volt_gradient_filename = extractVoltageSchemeFilename(filename = raw_filename)
        ### probably need to put non voltage gradient exception handling here
        if volt_gradient_filename:
            # set log filename
            gradient_log_filename = raw_filename[:-4] + ".csv"
            # read voltage scheme into table - assuming the scheme file is located under the same location where this script is located
            volt_gradient_table = readGradientTable(volt_gradient_filename)
            print(volt_gradient_table)
            # schedule the gradient - nonblocking background scheduling
            scheduleStepVoltage(serial_info, scheduler, volt_gradient_table, gradient_log_filename)
            # print schedule
            scheduler.print_jobs()
        else: # if filename is empty
            print("No Voltage scheme in the filename ...")

        # Wait until the acquisition is done
        while(q.GetRunManagerStatusFromCom == u"Acquiring"):
            sleep(5)
            print("Acquisition ...", volt_gradient_filename)
        sleep(1)
        # ensure power set to zero & set polarity to normal
        setPowerSupplyVoltage(serial_info, voltage = 0)
        setPowerSupplyCurrent(serial_info, current = 0)
        setPowerSupplyPolarity(serial_info, polarity = 1)
        print("Acquisition Ends ...")
        sleep(1)
        
