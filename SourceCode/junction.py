import os
import sys
import optparse
import subprocess
import random

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import socket
import datetime
# the port used for communicating with your sumo instance
PORT = 8873
flag=0
tlsid="0"
N=0;
troad=""
queue=[]
def search(item):
	for element in queue:
		if(element==item):
			return 1
	return 0
			
def lastElement():
	return queue[0]
def insert(item):
	global queue
	global N
	queue.append(item)
	N=N+1

def delete():
	global queue
	global N
	item=queue[0]
	queue.remove(item)
	N=N-1

def switchTraffic(tlsid):
	ti=str(datetime.datetime.now())
	print ti
	global flag
	global troad
	t=int(ti[17]+ti[18])
	print "inside switch traffic"
	print t
	traci.trafficlights.setPhaseDuration(tlsid,0.005)
	if(t>0 and t<=18):
		traci.trafficlights.setPhase(tlsid,1)
		print "traffic switched to phase 1"
	elif(t>18 and t<=20):
		traci.trafficlights.setPhase(tlsid,2)
		print "traffic switched to phase 2"
	elif(t>20 and t<=28):
		traci.trafficlights.setPhase(tlsid,4)
		print "traffic switched to phase 4"
	elif(t>28 and t<=30):
		traci.trafficlights.setPhase(tlsid,5)
		print "traffic switched to phase 5"
	elif(t>30 and t<=48):
		traci.trafficlights.setPhase(tlsid,7)
		print "traffic switched to phase 7"
	elif(t>48 and t<=50):
		traci.trafficlights.setPhase(tlsid,8)
		print "traffic switched to phase 8"
	elif(t>50 and t<=58):
		traci.trafficlights.setPhase(tlsid,10)
		print "traffic switched to phase 10"
	else:
		traci.trafficlights.setPhase(tlsid,11)
		print "traffic switched to phase 11"
	delete()
	flag=0
	troad=""
def normalTraffic(vid):
	global tlsid
	global flag
	global troad
	jpos=traci.junction.getPosition(tlsid)
	vpos=traci.vehicle.getPosition(vid)
	jx=abs(jpos[0]-500)
	jy=abs(jpos[1]-500)
	vx=abs(vpos[0]-500)
	vy=abs(vpos[1]-500)
	print vpos
	print jpos
	print "inside normal....."
	vroad=traci.vehicle.getRoadID(vid)
	if(vroad<>troad):
	#if(vpos[0]<500):
		switchTraffic(tlsid)
			

def changeTraffic(vid):
	laneIndx=traci.vehicle.getLaneIndex(vid)
        print vid
        currstate=""
	vlane=traci.vehicle.getLaneID(vid)
        print vlane
	alltid=traci.trafficlights.getIDList()
        print alltid
	global troad
	print " inside change traffic "
	for each in alltid:
		lanetid=traci.trafficlights.getControlledLanes(each)
                print lanetid
		if vlane in lanetid :
			tid=lanetid
			print tid
			state=""
			for item in lanetid:
				if(item==vlane):
					newstate="G"
					troad=traci.vehicle.getRoadID(vid)
				#print " required lane found "
				else:
					newstate="r"
				temp=state[:len(state)]+newstate
				state=temp
			if(state[0]=='G' or state[1]=='G' or state[2]=='G'or state[3]=='G' ):
				traci.trafficlights.setPhase(each,0)
				traci.trafficlights.setPhaseDuration(each,30)

			elif(state[4]=='G' or state[5]=='G' or state[6]=='G'):
				traci.trafficlights.setPhase(each,3)
				traci.trafficlights.setPhaseDuration(each,30)

			elif(state[7]=='G' or state[8]=='G' or state[9]=='G'  or state[10]=='G' ):
				traci.trafficlights.setPhase(each,6)
				traci.trafficlights.setPhaseDuration(each,30)
				
			elif(state[11]=='G' or state[12]=='G' or state[13]=='G'):
				traci.trafficlights.setPhase(each,9)
				traci.trafficlights.setPhaseDuration(each,30)
				
			
			global tlsid
			tlsid=each
			global flag
			flag=1
			print "flag changed to  ",flag
			
		
def run():
    """execute the TraCI control loop"""
    s = socket.socket()
    s.connect( ('127.0.0.1',55055) )
    so = socket.socket()
    so.connect( ('127.0.0.1',5555) )
    traci.init(PORT)
    traci.simulationStep()
    #list=traci.vehicle.getIDList()
    hi="hii"
    while  traci.simulation.getMinExpectedNumber() > 0:
		traci.simulationStep()
                list=traci.vehicle.getIDList()
                try:
                	if len(list)<1:
                		traci.simulationStep()
			else:
                                counter=0
				while(counter<len(list)):
				        pos = traci.vehicle.getPosition(list[counter])
					buff= traci.vehicle.getRoadID(list[counter])+"@"+str(pos[0])+"$"+str(pos[1])+"$"+list[counter]+"$"
					print buff
					s.send(buff)
                                        buff=""
					counter=counter+1
					print "Received ",s.recv(255)
                                        data=so.recv(255);
					global flag
					global N
					if(data<>hi):
						j=0;
   						vid=""
						while(data[j]<>'$'):
							vid=vid+data[j]
							j=j+1;
						print " ambulance arrived ",vid
						status=search(vid)
						if(status==0):
							insert(vid)
					if(flag==0 and N>0):
						vd=lastElement()
						changeTraffic(vd)
					
					print flag,"flag value"
					if(flag==1):
						vd=lastElement()
						normalTraffic(vd)
		except:
			s.send("stop")
			traci.close()
			s.close()
				
  		sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "jn.sumocfg","--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
