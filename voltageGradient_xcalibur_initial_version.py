import win32com.client as win32
import serial
import datetime
from datetime import timedelta
from time import sleep
import pandas as pd
import csv
import os
from apscheduler.schedulers.background import BackgroundScheduler

def getPowerSupplyOutputLine(ser):
    # add time mark in front
    return('\t'.join([str(datetime.datetime.now()), ser.readline().decode('ascii')]))

def setPowerSupplyVoltage(ser, voltage):
    ser.write(("V="+str(voltage)).encode('ascii'))
    #return(voltage)
    
def setPowerSupplyCurrent(ser, current):
    ser.write(("I="+str(current)).encode('ascii'))
    #return(current)
    
def setPowerSupplyPolarity(ser, polarity):
    ser.write(("P="+str(polarity)).encode('ascii'))
    
def scheduleStepVoltage(ser, scheduler, volt_gradient_table):
    # get the start time of acquisition (RT=0)
    now = datetime.datetime.utcnow()

    # schedule voltage, currrent, and polarity based on the table
    for index, row in volt_gradient_table.iterrows():
        #print(row["time(min)"], row["voltage(V)"], row["current(A)"])
        # set the RT point
        run_at = now + timedelta(minutes= float(row["time(min)"]))
        # set proper type for each value
        current = round(float(row["current(A)"]),2)
        voltage = round(float(row["voltage(V)"]),2)
        polarity = int(row["polarity"])
        # schedule the current, voltage, and polarity with RT point
        scheduler.add_job(setPowerSupplyCurrent, 'date', run_date = run_at, args = [ser, current])
        scheduler.add_job(setPowerSupplyVoltage, 'date', run_date = run_at, args = [ser, voltage])
        scheduler.add_job(setPowerSupplyPolarity, 'date', run_date = run_at, args = [ser, polarity])

def readGradientTable(volt_gradient_filename):
    # get the voltage scheme from VOLT csv file
    volt_gradient_table = pd.read_csv(volt_gradient_filename, skipinitialspace=True)
    volt_gradient_table = volt_gradient_table.apply(pd.to_numeric,downcast='float') # change all to float
    
    return(volt_gradient_table)

### INITIALIZATION
# communication with the power supply
ser = serial.Serial('/dev/ttyACM4', 9600)  ## serial.Serial('COM5', 9600)
# xcalibur connection
#q = win32.Dispatch("AcqServer.AcqInterfaceSupport1")
# scheduler start
scheduler = BackgroundScheduler(daemon=True, timezone="utc")
scheduler.start()

while True:
    if(q.GetRunManagerStatusFromCom == u"Acquiring"):
        # Acquisition Start from here

        # retrieve raw filename
        raw_filename = q.GetMangledRawFilename
        # set log filename
        gradient_log_filename = raw_filename[:-4] + ".csv"
        # retrieve voltage scheme
        volt_gradient_filename = raw_filename.split('.')[0].split('_')[-1] + ".csv"
        # read voltage scheme into table
        volt_gradient_table = readGradientTable(volt_gradient_filename)
        print(volt_gradient_table)
        # schedule the gradient
        scheduleStepVoltage(ser, scheduler, volt_gradient_table)
        # print schedule
        scheduler.print_jobs()

        # Wait until the acquisition is done
        while(q.GetRunManagerStatusFromCom == u"Acquiring"):
            # log here
            with open(gradient_log_filename,"a") as f:
                writer = csv.writer(f)
                writer.writerow([time.time(),getPowerSupplyOutputLine(ser)])
            sleep(1)
        # ensure power set to zero & set polarity to normal
        setPowerSupplyVoltage(ser, voltage = 0)
        setPowerSupplyCurrent(ser, current = 0)
        setPowerSupplyPolarity(ser, polarity = 1)
        sleep(1)
        
