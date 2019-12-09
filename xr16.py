#!/usr/bin/python

import socket, sys, threading 
from threading import Thread
import time
import os
from OSC import OSCClient, OSCMessage, decodeOSC
import RPi.GPIO as GPIO

def sendOSC(address, param = None):
    global device, txSocket
    msg = OSCMessage(address)
    if param != None:
        # when sending values, ignore ACK response
        last_cmd_addr = address
        if isinstance(param, list):
            msg.extend(param)
        else:
            msg.append(param)
    else:
        # sending parameter request, don't ignore response
        last_cmd_addr = ''
    if (device and txSocket ):
        print 'sending: %s' % (msg)        
        txSocket.sendto(msg.getBinary(),(device,portNum))  
    
    
def getMsg():
    global txSocket
    responses = []
    try:
        #Attempt to receive up to 512 bytes of data
        #data,addr = rxSocket.recvfrom(512) 
        #Echo the data back to the sender
        #rxSocket.sendto(str(data).encode('utf-8'),addr)
        if (txSocket):
            data, addr = txSocket.recvfrom(1024)
            responses = decodeOSC(data)
    except socket.timeout:
            print "response timeout"
            return
    except socket.error:
            #reaching here means no data are received
            return
    #for response in responses:
        #print(str(response)+"/")
    return responses
                   
def find_mixer():
    global messageLed
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, True)
    client.settimeout(3)
    client.sendto("/xinfo\0\0", ("<broadcast>", portNum))
    print ("searching console...")
    try:
        response = decodeOSC(client.recv(512))
    except socket.timeout:
        print "No server found"
        messageLed=1
        return False
    client.close()

    if response[0] != '/xinfo':
        print "Unknown response"
        return False
    else:
        print "Found " + response[4] + " with firmware " + response[5] + " on IP " + response[2]
        messageLed=0
        return response[2]

def restartMonitor():
    global device
    global txSocket
    global delaytime
    global messageLed
    device = find_mixer()       
    if (device):
        print("findMixer() return:"+str(device))
        messageLed=0
        if (txSocket<>None):
            txSocket.close()
        txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        txSocket.setblocking(0)           
        #should only register to usefull info such CH MIX ON
        #print ("send /xremote to" + str(device))
        #txSocket.sendto("/xremote".encode('utf-8'),device)
        sendOSC("/xremote")
        #sendOSC("/formatsubscribe","/rtn/*/mix/on",1)
        getConsoleState()
    threading.Timer(9, restartMonitor).start()
    
def getConsoleState():
    print ("getting console state...")
    getChannelState()
    #get level of the volumecontrol channel
    for i in range(len(veryLongPushMsgString)):
        sendOSC(veryLongPushMsgString[i])
        time.sleep(0.1)       
    #default value tap delay            
    sendOSC(tapCommand)

def asyncRxThread(portNum):
    global exit
    global txSocket
    global tapCommand
    
    print ("Thread watching data on UDP is running")
    
    while not exit:
            msg=getMsg()
            if (msg):
                #print("RX: adress("+str(msg[0])+") value("+str(msg[2])+")")
                print("RX: "+str(msg))                  
                for i in range(len(channelsOn)):
                    if (msg[0]==shortLongPushMsgString[i]):
                        channelsOn[i]=bool(msg[2])
                        GPIO.output(toggleLed[i], not channelsOn[i]) 
                                
                for i in range(len(veryLongPushMsgString)):
                    if (msg[0]==veryLongPushMsgString[i]):
                        channelLevel[i]=msg[2]
                                        
                if (msg[0]==tapCommand):
                    delaytime=(msg[2]*3000)
    print("asyncRxThread exit ")

def allLedsOff():
    for i in range(len(channelsOn)):
        GPIO.output(toggleLed[i],False)

def allLedsOn():
    for i in range(len(channelsOn)):
        GPIO.output(toggleLed[i],True)



def ledBarShow():
    global messageLedProgress
    allLedsOff()
    for i in range(0,messageLedProgress):
        GPIO.output(toggleLed[i],True)
        #print("PROGRESS GPIO.output("+str(toggleLed[i])+",True)")

 
def ledBarControl(param):
    global exit
    global prevMessageLed
    global messageLedProgress
    global messageLed

    while not exit:

        #we change the message, means we go back to normal mode by selecting 0
        if (messageLed==0 and prevMessageLed<>0):
            prevMessageLed = 0
            #restore the current led/channel from console read
            getChannelState()
        #message 1 is "please wait loading so we just shift leds ...
        if (messageLed==1):
            prevMessageLed = 1
            if (messageLedProgress>4):
                    messageLedProgress=0                    
            ledBarShow()
            messageLedProgress=messageLedProgress+1         
            time.sleep(0.1)
        #message 2 means we light all led until messageLedProgress
        if (messageLed==2):
            prevMessageLed=2
            ledBarShow()
                    
    print("ledBarControl exit ")
 
def constrain(val, min_val, max_val):

    if val < min_val: return min_val
    if val > max_val: return max_val
    return val

def getChannelState():
    for i in range(len(channelsOn)):
        sendOSC(shortLongPushMsgString[i])
        time.sleep(0.1)
        msg=getMsg()
        if (msg):
            print (msg)
            channelsOn[i]=bool(msg[2])             
            GPIO.output(toggleLed[i], not channelsOn[i]) 

def setupGPIO():
    # set up the GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for i in range(len(buttons)):
        # first led
        GPIO.setup(toggleLed[i], GPIO.OUT)
        GPIO.setup(toggleLed[i+2], GPIO.OUT)
        #then button
        GPIO.setup(buttons[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)        
    
    #button tap
    GPIO.setup(tapButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(tapLed, GPIO.OUT)    

  
def main(args):  
    global messageLed, messageLedProgress, exit
   
    #this is the time that the tap timer was initialised    
    startTime = long(0)  

    #This is part of the stopwatch that times the difference between the tempo button presses
    start = False

    #The next three variables are controlling the blink rate of the tempo LED
    ledState5 = False;      
    #will store last time LED was updated
    previousMillis = long(0)
    currentMillis = long(0)
    previous5 = True
    previousstate5 = True
    state5 = False
    rawtime = long(0)
    # the follow variable is a long because the time, measured in miliseconds,
    # will quickly become a bigger number than can be stored in an int.
    #interval at which to blink (milliseconds) that is restricted to 100ms-3000
    
    #// interval time unrestricted
    rawtime = long(0)
    #// the global debounce time in ms for the foot switches    
    debounce = long(200)  
    time5 = long(0)



    levelIncrement=0.004
    # set up some variable to read from the button
    input = False
    maintain=False
    previousInput = [True,True]
    push_time = [0,0]
    indexChannel=0
    
    setupGPIO()

    global exit
    global txSocket
    global delaytime           
     
    while not exit:
        try:
            if (device):
                #//Take the current time for the cycle.
                currentMillis = millis = int(round(time.time() * 1000))		
                
                for i in range(len(buttons)):
                    # read the button state to the variable input
                    input = GPIO.input(buttons[i])
                    # make sure the button state isn't what it used to be
                    if ((not previousInput[i]) and input):
                        if (push_time[i]<15):
                            if (push_time[i]<6):
                                print("short push")
                                indexChannel=0+(i*2)                      
                            if (push_time[i]>6) and (push_time[i]<20):
                                print("long push("+str(push_time[i])+")")
                                indexChannel=1+(i*2)
                            channelsOn[indexChannel] = not channelsOn[indexChannel]
                            #GPIO.output(toggleLed[indexChannel], not channelsOn[indexChannel]) 
                            #print("GPIO.output("+str(toggleLed[indexChannel])+", not "+str(channelsOn[indexChannel])+")")
                            #convert bollean to int
                            sendOSC(shortLongPushMsgString[indexChannel],int(channelsOn[indexChannel] == True)) 
                        else:
                            print("long maintain("+str(push_time[i])+") on button:"+str(i))
                            input = previousInput
                        #print("indexChannel="+str(indexChannel)+" ch1: "+str(channelsOn[i])+" ch2: "+str(channelsOn[i+1]))                
                        print "i:"+str(i)+" indexChannel:"+str(indexChannel)
                        push_time[i] = 0 
                        levelIncrement = 0.004
                        messageLed=0
                        
                    # copy the button state to the previousInput variable
                    previousInput[i] = input
                    if (not input):
                        push_time[i] = push_time[i] +1 
                        if (push_time[i]>15):
                            #here goes the code to put volume up and down
                            messageLed=2
                            messageLedProgress=int(round(channelLevel[0]*4))
                            #print("messageLed="+str(messageLed)+" progress:"+str(messageLedProgress))
                           
                            if (i==0):
                                #print("volume down("+str(push_time[i])+")")
                                if (channelLevel[0]>levelIncrement):
                                    sendOSC(veryLongPushMsgString[0],channelLevel[0]-levelIncrement)                           
                                    levelIncrement=levelIncrement+0.0005
                                    
                            else:
                                #print("volume up("+str(push_time[i])+")")
                                if (channelLevel[0]<1-levelIncrement):
                                    sendOSC(veryLongPushMsgString[0],channelLevel[0]+levelIncrement) 
                                    levelIncrement=levelIncrement+0.0005
                            if (previousInput[0]==False and previousInput[1]==False):
                                exit = True
                                print("Quit")
                                                             
                # wait to "debounce" the input
                time.sleep(0.05)                      
                #Attempt to receive the echo from the server
                
                reading5 = GPIO.input(tapButton)
                if (reading5 == False and previous5 == True and ((currentMillis - time5) > debounce)):
                    if (state5):
                        state5 = False
                    else:
                        state5 = True
                    time5 = currentMillis
                    print (str(time5))
                          
                
                ########################### tap routine ###############################
                
                #Finaly we evaluate whether switch five has changed state from the previous cycle and create a timer function to register the difference between presses
                rawtime =   currentMillis - startTime              
                #store elapsed time
                if (rawtime > 3000):
                #Last tap more than two seconds ago
                  start = False
                  #// stop time calculation
                 

                if (state5 != previousstate5):
                #// check for a high to low transition
                        
                    if (start):					
                    #// if true then found a new button press while clock is not running - calculate the time
                    
                        delaytime = constrain (rawtime, 50, 3000)

                        #// then we take our variables for the time values and construct them into a HEX message to send to the unit. 
                        
                        #//...then all we need to do is write it to the MIDI out port....
                        #Serial.write(BehringerHeader,sizeof(BehringerHeader))
                        #print ("delay time:"+str(thousand)+"'"+str(thousand)+"'"+str(huns)+"'"+str(tens)+"'"+str(units))
                        #print("delay:"+str(delaytime))
                        tap=float((delaytime/1000.0)%60)
                        print ("tap:"+str(tap)+"delay: "+str(delaytime))
                        sendOSC(tapCommand,int(delaytime))
                    
                    startTime = currentMillis                                  
                    #// store the start time
                    start = True										
                    #// activate time calculation for next cycle
                

                #// This next section tries to eliminate false triggering and long delay times by restarting the timing clock for the tempo switch, if the gap is greater than 3000ms.
                if (currentMillis - startTime > 3000):
                    start = False
                
                #// This next section blinks LED 5 at the tempo rate
                if(currentMillis - previousMillis > (delaytime / 2)):
                
                    #// if the LED is off turn it on and vice-versa:
                    if (ledState5):
                      ledState5 = False
                    else:
                      ledState5 = True
                    #// set the LED with the ledState of the variable:
                    #digitalWrite(outPin5, ledState5);
                    GPIO.output(tapLed,ledState5) 
                    previousMillis = currentMillis
                    #save the last time you blinked the LED
                       
                        
                previous5 = reading5
                previousstate5 = state5
                ##########################################################
            #else:
                #print ("no connection...")            
        except KeyboardInterrupt:
            exit = True
            print ("Received Ctrl+C... initiating exit")
            break
    
    udpRxThreadHandle.join()
    ledBarThreadHandle.join()
    allLedsOff()
    return

channelsOn = [True,True,True,True]
#LED GPIO
toggleLed=[22,23,24,25]
# declare the GPIO pins we're using
buttons = [18,27]
tapButton = 17
tapLed = 4
portNum = 10024
#shortLongPushMsgString = ["/ch/01/mix/on","/ch/02/mix/on"]
shortLongPushMsgString = ["/rtn/1/mix/on","/rtn/2/mix/on","/rtn/3/mix/on","/rtn/4/mix/on"]
veryLongPushMsgString = ["/ch/01/mix/02/level"]
#veryLongPushMsgString = ["/bus/1/mix/01/level",""]
tapCommand="/fx/1/par/01"
channelLevel= [0.0,0.0]
delaytime = long(1000)    
exit = False
prevMessageLed=1
messageLed=1
messageLedProgress=0
device = None
txSocket = None
setupGPIO()

print ("Xair footswitch v0.25")
print ("Press Ctrl+Z to exit")
print ("")
    


ledBarThreadHandle = Thread(target=ledBarControl,args=("0",))    
ledBarThreadHandle.start()

udpRxThreadHandle = Thread(target=asyncRxThread,args=(portNum,))    
udpRxThreadHandle.start()

restartMonitor()

if __name__=="__main__":
    main(sys.argv[1:0])
print("exit")
os._exit(0)

