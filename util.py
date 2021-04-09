import datetime
from datetime import timedelta
import pandas as pd

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
