vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    while clientID <=-1
 
    disp('Failed connecting to remote API server');
    pause(1);
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    end
     disp('Connected to remote API server');
% vrep.simxSynchronous(clientID, true);
        objs=zeros(1,6);
  
        [res objs(1)]=vrep.simxGetObjectHandle(clientID,'R1',vrep.simx_opmode_oneshot_wait);
        [res objs(2)]=vrep.simxGetObjectHandle(clientID,'R2',vrep.simx_opmode_oneshot_wait);
        [res objs(3)]=vrep.simxGetObjectHandle(clientID,'R3',vrep.simx_opmode_oneshot_wait);
        [res objs(4)]=vrep.simxGetObjectHandle(clientID,'R4',vrep.simx_opmode_oneshot_wait);
        [res objs(5)]=vrep.simxGetObjectHandle(clientID,'R5',vrep.simx_opmode_oneshot_wait);
        [res objs(6)]=vrep.simxGetObjectHandle(clientID,'R6',vrep.simx_opmode_oneshot_wait);
        [res graph]=vrep.simxGetObjectHandle(clientID,'graph',vrep.simx_opmode_oneshot_wait);
        q=zeros(1,6);
%         q=IK(-0.5,0.5,1,pi/4,0,0);
        rer=zeros(1,6);
        num=zeros(1,6);
        pos=zeros(1,6);
        p=zeros(1,6);
        h=0.05;
        
        Kp=20;
        Ki=0.1;
        Kd=0.1;
   t=0;
   for i=1:6
   [res,pos(i)]=vrep.simxGetJointPosition(clientID,objs(i),vrep.simx_opmode_streaming);
   [res,R]=vrep.simxGetObjectOrientation(clientID,graph,-1,vrep.simx_opmode_streaming);
   end
   e=zeros(1,6);
   I=zeros(1,6);
   while t<=3600
        q=IK(0.2*cos(t)+0.2,0.2*sin(t)+0.8,0.4,0,0,0);
        el=e;
        e=q-pos;
        P=Kp*e;
        I=I+Ki*e;
        D=Kd*(e-el);
        u=P+I+D;
%         q=IK(0.8,0,1.2,0,0,0);      
          [res,R]=vrep.simxGetObjectOrientation(clientID,graph,-1,vrep.simx_opmode_buffer);
    for i=1:6

%         p(i)=vrep.simxSetJointPosition(clientID,objs(i),q(i),vrep.simx_opmode_oneshot);
         p(i)=vrep.simxSetJointPosition(clientID,objs(i),q(i),vrep.simx_opmode_oneshot);
        [res,pos(i)]=vrep.simxGetJointPosition(clientID,objs(i),vrep.simx_opmode_buffer);
        
    end
   disp(R);
   pause(h);
   t=t+h;
  
        end
   
   