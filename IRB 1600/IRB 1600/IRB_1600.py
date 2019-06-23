
import vrep
import vrepConst
import time
import socket
import sys
import struct

joint=[]
pos=[]
V=[]
force=[]

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
while clientID<=-1:
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

    if clientID!=-1:  
        print ('Successful !!!')
    
    else:
        print ('connection not successful')
        #sys.exit('could not connect')

print ('connected to remote api server')

for i in  range(0,6):
    joint.append([])
    pos.append([])
    v.append([])
    force.append([])
    res,joint[i]=vrep.simxgetobjecthandle(clientid,'r'+str(i+1),vrep.simx_opmode_blocking)
    res,pos[i]=vrep.simxgetjointposition(clientid,joint[i],vrep.simx_opmode_streaming)
    res,force[i]=vrep.simxgetjointforce(clientid,joint[i],vrep.simx_opmode_streaming)

host = 'localhost'
port = 20000
port2=20001 #position
port3=20005 #force
port4=20002

addr = (host, port)
addr2 = (host, port2)
addr3=(host,port3)
addr4=(host,port4)



server = socket.socket(socket.af_inet, socket.sock_dgram)
server2=socket.socket(socket.af_inet, socket.sock_dgram)
server.bind(addr)
server2.bind(addr4)

while True:
    arrforce=struct.pack('dddddd',force[0],force[1],force[2],force[3],force[4],force[5])
    arrpos=struct.pack('dddddd',pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])


   
    server.sendto(arrpos,addr2)
    server2.sendto(arrforce,addr3)
    time.sleep(0.001)
    data,addr = server.recvfrom(64)  
    c=struct.unpack('dddddd',data)
    for i in range(0,6):        
        v[i]=float(c[i])
        vrep.simxsetjointtargetvelocity(clientid,joint[i],v[i],vrep.simx_opmode_oneshot)
        res,pos[i]=vrep.simxgetjointposition(clientid,joint[i],vrep.simx_opmode_buffer)
        res,force[i]=vrep.simxgetjointforce(clientid,joint[i],vrep.simx_opmode_buffer)

server.close()



