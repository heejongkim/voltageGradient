## last updated: 2021.04.14

import win32com.client as win32
import serial
import datetime
from datetime import timedelta
from time import sleep
import time
import pandas as pd
import csv
import os
from apscheduler.schedulers.background import BackgroundScheduler

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
        scheduler.add_job(setPowerSupplyCurrent, 'date', run_date = now + timedelta(minutes= float(row["time(min)"])), args = [serial_info, current])
        scheduler.add_job(setPowerSupplyVoltage, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=1), args = [serial_info, voltage])
        scheduler.add_job(setPowerSupplyPolarity, 'date', run_date = now + timedelta(minutes= float(row["time(min)"]), seconds=2), args = [serial_info, polarity])
        # schedule teh log write to prevent blocking the port
        for log_time in range(4, 10, 2):
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

### INITIALIZATION
# PowerSupply Communication Info
serial_info = {"port": "COM28", "baudRate": 9600}

# xcalibur connection
q = win32.Dispatch("AcqServer.AcqInterfaceSupport1")
# scheduler start
scheduler = BackgroundScheduler(daemon=True, timezone="utc")
scheduler.start()

while True:
    print(getPowerSupplyOutputLine(serial_info))
    if(q.GetRunManagerStatusFromCom == u"Acquiring"):
        # Acquisition Start from here
        print("Acquisition Starts ...")
        # retrieve raw filename
        raw_filename = q.GetMangledRawFilename
        # set log filename
        gradient_log_filename = raw_filename[:-4] + ".csv"
        # retrieve voltage scheme
        volt_gradient_filename = raw_filename.split('.')[0].split('_')[-1] + ".csv"
        #volt_gradient_filename = "volt001.csv"
        ### probably need to put non voltage gradient exception handling here
        # read voltage scheme into table
        volt_gradient_table = readGradientTable(volt_gradient_filename)
        print(volt_gradient_table)
        # schedule the gradient
        scheduleStepVoltage(serial_info, scheduler, volt_gradient_table, gradient_log_filename)
        # print schedule
        scheduler.print_jobs()

        # Wait until the acquisition is done
        while(q.GetRunManagerStatusFromCom == u"Acquiring"):
            sleep(5)
            print("Acquisition ...")
        sleep(1)
        # ensure power set to zero & set polarity to normal
        setPowerSupplyVoltage(serial_info, voltage = 0)
        setPowerSupplyCurrent(serial_info, current = 0)
        setPowerSupplyPolarity(serial_info, polarity = 1)
        print("Acquisition Ends ...")
        sleep(1)
        
