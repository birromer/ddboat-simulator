-- u1 : moteur avant gauche Join_Left_Front frontMotorLeft
-- u2 : moteur avant droit Join_Right_Front frontMotorRight
-- u3 : moteur arriere gauche Join_Left_Back backMotorLeft
-- u4 : moteur arriere droit Join_Right_Back backMotorRight

-- function subscriber_cmd_u1_callback(msg)
--     spdMotor = msg["data"]
--     sim.setJointTargetVelocity(frontMotorLeft, spdMotor)
--   sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
-- end

-- function subscriber_cmd_u2_callback(msg)
--   spdMotor = msg["data"]
--   sim.setJointTargetVelocity(frontMotorRight, spdMotor)
--   sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
-- end

-- function subscriber_cmd_u3_callback(msg)
--   spdMotor = msg["data"]
--   sim.setJointTargetVelocity(backMotorLeft, spdMotor)
--   sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
-- end

-- function subscriber_cmd_u4_callback(msg)
--   spdMotor = msg["data"]
--   sim.setJointTargetVelocity(backMotorRight, spdMotor)
--   sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
-- end

require ("math")
socket=require("socket")


-- Use sleeping function of socket library
function sleep(sec)
    socket.select(nil,nil,sec)
end 


-- Get cuurent time (in sec) 
function gettime()
   return socket.gettime()
end


function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end


function getWheelAngularPosition (handle)
    angles = sim.getObjectOrientation(handle, sim_handle_parent)
    angPos = angles[3]*180.0/math.pi
    angPos = angPos + 180.0
    return angPos
end 


-- Compute increment of odemeter (
function deltaWheelAngularPosition(curPos,lastPos,wSpeed)
  -- this function Compute increment of odemeter
    if wSpeed == 0.0 then
        deltaPos = 0.0
    else
        deltaPos = curPos - lastPos
    end
    if deltaPos < 0.0 then
        if wSpeed > 0.0 then
            deltaPos = deltaPos + 360.0
        end
    else
        if wSpeed < 0.0 then
            deltaPos = deltaPos - 360.0
        end
    end
    local nTicks=300.0
    deltaPos = deltaPos*nTicks/360.0
    --print ("delta pos ",curPos,lastPos,wSpeed,deltaPos)
    return deltaPos
end


function getCompass(objectName,statemotor)
  -- This function get the value of the compass 
  objectHandle = sim.getObjectHandle(objectName)
  relTo = -1
  o = sim.getObjectOrientation(objectHandle, relTo)
  if statemotor then -- if motor is ON 
    heading = -o[3] + gaussian(0,1)*math.pi  -- we add a gaussian noise
  else
    heading = -o[3]   -- north along X > 0
  end
  return heading*180/math.pi -- in degre
end


function getGPS(objectName)
  -- This function get the value of the position of the boat
  -- and estimate the longitude and the lattitude of the boat
  -- the reference is the "ponton of Guerledan"
  longRef = -3.01473333*math.pi/180  -- To complete
  latRef = 48.19906500*math.pi/180  -- To complete
  rho =  6370000 -- To complete
  accurateGPS= 0.00001 -- To complete in degre
  relTo = -1
  objectHandle = sim.getObjectHandle(objectName)
  p = sim.getObjectPosition(objectHandle,relTo)
  ly = (p[2]/rho) + latRef
  lx = (p[1]/(rho*math.cos(ly))) + longRef
  return {longitude = lx*180/math.pi + accurateGPS * gaussian(0,1), lattitude = ly*180/math.pi+ accurateGPS * gaussian(0,1)}
end


function getAcceleration()
  -- This function get the value of acceleration
  accurateAccelerometer = 0.1 --To complette
  tmpData = sim.tubeRead(accelCommunicationTube)
    if (tmpData) then
    accel=sim.unpackFloats(tmpData)
    end
    return {accel_x = accel[1] + accurateAccelerometer * gaussian(0,1), accel_y = accel[2] + accurateAccelerometer * gaussian(0,1), accel_z = accel[3] + accurateAccelerometer * gaussian(0,1)}
end


function getGyrometer()
  -- This function get the value of acceleration
  accurateGyrometer = 0.01 --To complette
    tmpData = sim.tubeRead(gyroCommunicationTube)
    if (tmpData) then
    gyro = sim.unpackFloats(tmpData)
    end
    return {gyro_x = gyro[1] + accurateGyrometer * gaussian(0,1), gyro_y = gyro[2] + accurateGyrometer * gaussian(0,1), , gyro_z = gyro[3] + accurateGyrometer * gaussian(0,1)}
end


function getEncoder()
  -- This function update encoders with relative orientation of the wheel (wrt joint axis)
    for ip=1,#propellers do
      handle=sim.getObjectHandle(objectName)

      relativeOrientation = getWheelAngularPosition(handle)
      currentOrientationTime = sim.getSimulationTime()
      --print (ip,relativeOrientation,lastRelativeOrientation[ip])
      --print (ip,propellersCnt[ip])
      if ip == 1 then  -- left propeller (count ++)
        propellerSpeed = leftpropellerSpeed
        dCnt = deltaWheelAngularPosition(relativeOrientation,lastRelativeOrientation[ip],propellerSpeed)
        propellersCnt[ip] = propellersCnt[ip] + dCnt
      end
      if ip == 2 then  -- right propeller (count --)
        propellerSpeed = rightWheelSpeed
        dCnt = deltaWheelAngularPosition(relativeOrientation,lastRelativeOrientation[ip],propellerSpeed)
        propellersCnt[ip] = propellersCnt[ip] - dCnt
      end
      --print (ip,propellersCnt[ip])
      lastRelativeOrientation[ip] = relativeOrientation
      lastOrientationTime[ip] = currentOrientationTime
    end
      return {leftEncoder = propellersCnt[1], rightEncoder = propellersCnt[2]}
end


function subscriber_cmd_ul_callback(msg)
  spdMotor = msg["data"]
  sim.setJointTargetVelocity(backMotorLeft, spdMotor)
  sim.setJointTargetVelocity(frontMotorLeft, spdMotor)

  sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
end

function subscriber_cmd_ur_callback(msg)
  spdMotor = msg["data"]
  sim.setJointTargetVelocity(backMotorRight, spdMotor)
  sim.setJointTargetVelocity(frontMotorRight, spdMotor)

  sim.addStatusbarMessage('cmd_u1 subscriber receiver : wheels speed ='..spdMotor)
end

function getPose(objectName)
  -- This function get the object pose at ROS format geometry_msgs/Pose
  objectHandle=sim.getObjectHandle(objectName)
  relTo = -1
  p=sim.getObjectPosition(objectHandle,relTo)
  o=sim.getObjectQuaternion(objectHandle,relTo)

  return {
    position={x=p[1],y=p[2],z=p[3]},
    orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
  }
end

function getTransformStamped(objHandle,name,relTo,relToName)
  -- This function retrieves the stamped transform for a specific object
  t=sim.getSystemTime()
  p=sim.getObjectPosition(objHandle,relTo)
  o=sim.getObjectQuaternion(objHandle,relTo)

  return {
    header={
      stamp=t,
      frame_id=relToName
    },
    child_frame_id=name,
    transform={
      translation={x=p[1],y=p[2],z=p[3]},
      rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
    }
  }
end

function sysCall_init()
  -- The child script initialization
  objectName = "Chassis"
  objectHandle = sim.getObjectHandle(objectName)

  -- get left and right motors handles
  backMotorLeft = sim.getObjectHandle("Join_Left_Back")
  backMotorRight = sim.getObjectHandle("Join_Right_Back")
  frontMotorLeft = sim.getObjectHandle("Join_Left_Front")
  frontMotorRight = sim.getObjectHandle("Join_Right_Front")

  rosInterfacePresent = simROS

  -- Prepare the publishers and subscribers :
  if rosInterfacePresent then
    publisher1 = simROS.advertise('/simulationTime','std_msgs/Float32')
    publisher2 = simROS.advertise('/pose','geometry_msgs/Pose')

    --subscriber1=simROS.subscribe('/cmd_u1','std_msgs/Float32','subscriber_cmd_u1_callback')
    --subscriber2=simROS.subscribe('/cmd_u2','std_msgs/Float32','subscriber_cmd_u2_callback')
    --subscriber3=simROS.subscribe('/cmd_u3','std_msgs/Float32','subscriber_cmd_u3_callback')
    --subscriber4=simROS.subscribe('/cmd_u4','std_msgs/Float32','subscriber_cmd_u4_callback')

    subscriber3 = simROS.subscribe('/cmd_ul','std_msgs/Float32','subscriber_cmd_ul_callback')
    subscriber4 = simROS.subscribe('/cmd_ur','std_msgs/Float32','subscriber_cmd_ur_callback')
  end

  -- Get some handles (as usual !):
  onboard_camera = sim.getObjectHandle('OnBoardCamera')

  -- Enable an image publisher and subscriber:
  pub = simROS.advertise('/image', 'sensor_msgs/Image')
  simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

  -- initialization for sensor processing
  -- initialize communication tube with accelerometers and gyroscopes
  accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.GetNameSuffix(nil),1) 
  accel={0.0, 0.0, 0.0}
  gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.GetNameSuffix(nil),1) -- put this in the initialization phase
  gyro={0.0, 0.0, 0.0}
  lastRelativeOrientation={}
  lastOrientationTime={}
  propellersCnt={}
  propellers = {"LeftPropellers", "RightPropellers"} -- To complete
end

function sysCall_sensing()
    -- Publish the image of the vision sensor:
    local data, w, h = sim.getVisionSensorCharImage(onboard_camera)
    d = {}
    d['header'] = {seq=0,stamp=simROS.getTime(), frame_id="a"}
    d['height'] = h
    d['width'] = w
    d['encoding'] = 'rgb8'
    d['is_bigendian'] = 1
    d['step'] = w*3
    d['data'] = data
    simROS.publish(pub,d)
end
 

function sysCall_actuation()
   -- Send an updated simulation time message, and send the transform of the object attached to this script:
   if rosInterfacePresent then
      -- publish time and pose topics
      simROS.publish(publisher1, {data=sim.getSimulationTime()})
      simROS.publish(publisher2, getPose("Chassis"))

      -- send a TF
      -- To send several transforms at once, use simROS.sendTransforms instead
   end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
  if rosInterfacePresent then
    simROS.shutdownPublisher(publisher1)
    simROS.shutdownPublisher(publisher2)

    --simROS.shutdownSubscriber(subscriber1)
    --simROS.shutdownSubscriber(subscriber2)
    simROS.shutdownSubscriber(subscriber3)
    simROS.shutdownSubscriber(subscriber4)

    simROS.shutdownPublisher(pub)
  end
end
