--z_u1 et z_u2 are the topics to control for motors left/right

require ("math")

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


function getCompass(objectName, statemotor)
  -- This function get the value of the compass 
  objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
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
  longRef = -3.01473333*math.pi/180  
  latRef = 48.19906500*math.pi/180 
  rho =  6370000 
  accurateGPS= 0.00001 
  relTo = -1
  objectHandle = sim.getObjectHandle(objectName)
  p = sim.getObjectPosition(objectHandle,relTo)
  ly = (p[2]/rho) + latRef
  lx = (p[1]/(rho*math.cos(ly))) + longRef
  return {
    lx*180/math.pi + accurateGPS * gaussian(0,1),
    ly*180/math.pi+ accurateGPS * gaussian(0,1)
  }
end


function getAcceleration()
  -- This function get the value of acceleration
  accurateAccelerometer = 0.1
  tmpData = sim.tubeRead(accelCommunicationTube)
    if (tmpData) then
    accel = sim.unpackFloats(tmpData)
    end
    return {
      accel[1] + accurateAccelerometer * gaussian(0,1),
      accel[2] + accurateAccelerometer * gaussian(0,1),
      accel[3] + accurateAccelerometer * gaussian(0,1)
    }
end


function getGyrometer()
  -- This function get the value of acceleration
  accurateGyrometer = 0.01

  tmpData = sim.tubeRead(gyroCommunicationTube)

  if (tmpData) then
    gyro = sim.unpackFloats(tmpData)
  end

  return {
    gyro[1] + accurateGyrometer * gaussian(0,1),
    gyro[2] + accurateGyrometer * gaussian(0,1),
    gyro[3] + accurateGyrometer * gaussian(0,1)
  }
end


function getEncoder()
  -- This function update encoders with relative orientation of the wheel (wrt joint axis)
    for ip=1,#propellers do
      handle = sim.getObjectHandle(objectName)

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


function subscriber_z_u1_callback(msg)
  -- gets the values for motor left
  spdMotor1 = msg["data"]
  sim.addForceAndTorque(MotorLeft,{0,spdMotor1,0})

  sim.addStatusbarMessage('z_u1 subscriber receiver : wheels speed ='..spdMotor1)
end

function subscriber_z_u2_callback(msg)
  -- gets the values for motor right
  spdMotor1 = msg["data"]
  sim.addForceAndTorque(MotorRight,{0,spdMotor1,0})

  sim.addStatusbarMessage('z_u2 subscriber receiver : wheels speed ='..spdMotor1)
end

function subscriber_buoyancy_callback(msg) --does'nt work
  objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
  forcex = msg["force"]
  torque2 = msg["torque"]

end

function getPose(objectName)
  -- This function get the object pose at ROS format geometry_msgs/Pose
  objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
  --objectHandle = sim.getObjectHandle(sim.handle_self)
  relTo = -1
  p = sim.getObjectPosition(objectHandle,relTo)
  o = sim.getObjectQuaternion(objectHandle,relTo)

  return {
    position={x=p[1],y=p[2],z=p[3]},
    orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
  }
end

function getSpeed(objectName)
  -- This function get the object pose at ROS format geometry_msgs/Pose
  objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
  relTo = -1
  p, o =sim.getObjectVelocity(objectHandle)

  return {
    position={x=p[1],y=p[2],z=p[3]},
    orientation={x=o[1],y=o[2],z=o[3],w=0}
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
 -- objectName = "Corps_boat"
  objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)

  -- get left and right motors handles
  MotorLeft = sim.getObjectHandle("motor_left")
  MotorRight = sim.getObjectHandle("motor_right")
 

  rosInterfacePresent = simROS

  -- Prepare the publishers and subscribers :
  if rosInterfacePresent then
    publisher1 = simROS.advertise('/simulationTime','std_msgs/Float32')
    publisher2 = simROS.advertise('/pose','geometry_msgs/Pose')
    publisher3 = simROS.advertise('/speed','geometry_msgs/Pose')

    publisher_gyro = simROS.advertise('/gyro','std_msgs/Float32MultiArray')
    publisher_acc  = simROS.advertise('/acc','std_msgs/Float32MultiArray')
    publisher_enc  = simROS.advertise('/encoder','std_msgs/Float32MultiArray')
    publisher_gps  = simROS.advertise('/gps','std_msgs/Float32MultiArray')
    publisher_comp = simROS.advertise('/compass','std_msgs/Float32')

    subscriber3 = simROS.subscribe('/main/z_u1','std_msgs/Float32','subscriber_z_u1_callback')
    subscriber4 = simROS.subscribe('/main/z_u2','std_msgs/Float32','subscriber_z_u2_callback')
    subscriber5 = simROS.subscribe('/Torseur','geometry_msgs/Wrench','subscriber_buoyancy_callback')
  end

  -- Get some handles (as usual !):
 -- onboard_camera = sim.getObjectHandle('OnBoardCamera')


  -- Enable an image publisher and subscriber:
--  pub = simROS.advertise('/image', 'sensor_msgs/Image')
--  simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

  -- initialization for sensor processing
  -- initialize communication tube with accelerometers and gyroscopes
  accelCommunicationTube = sim.tubeOpen(0, 'accelerometerData#', 1)

  accel={0.0, 0.0, 0.0}

  gyroCommunicationTube = sim.tubeOpen(0, 'gyroData#', 1) -- put this in the initialization phase
  gyro={0.0, 0.0, 0.0}

  h=simGetObjectAssociatedWithScript(sim_handle_self)

  lastRelativeOrientation={}
  lastOrientationTime={}
  propellersCnt={}
  propellers = {"LeftPropellers", "RightPropellers"} -- To complete
end

function sysCall_sensing()
    -- Publish the image of the vision sensor:
--    local data, w, h = sim.getVisionSensorCharImage(onboard_camera)
--    d = {}
--    d['header'] = {seq=0,stamp=simROS.getTime(), frame_id="a"}
--    d['height'] = h
--    d['width'] = w
--    d['encoding'] = 'rgb8'
--    d['is_bigendian'] = 1
--    d['step'] = w*3
--    d['data'] = data
--    simROS.publish(pub,d)

    gyro_data = getGyrometer()
    local gyro_msg = {}
    gyro_msg.data = gyro_data
    simROS.publish(publisher_gyro, gyro_msg)

    acc = getAcceleration()
    local acc_msg = {}
    acc_msg.data = acc
    simROS.publish(publisher_acc, acc_msg)

--    encoder = getEncoder()
--    encoder_msg = {}
--    encoder_msg.data = encoder
--    simROS.publish(publisher_enc, encoder_msg)

    gps_data = getGPS("GPS")
    gps_msg = {}
    gps_msg.data = gps_data
    simROS.publish(publisher_gps, gps_msg)

    compass_data = getCompass(sim.handle_self, statemotor)
    compass_msg = {}
    compass_msg.data = compass_data
    simROS.publish(publisher_comp, compass_msg)
end
 

function sysCall_actuation()
   -- Send an updated simulation time message, and send the transform of the object attached to this script:
   if rosInterfacePresent then
      -- publish time and pose topics
      simROS.publish(publisher1, {data=sim.getSimulationTime()})

      objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)

      simROS.publish(publisher2, getPose(sim.handle_self))
      simROS.publish(publisher3, getSpeed(sim.handle_self))

      -- send a TF
      simROS.sendTransform(getTransformStamped(objectHandle, objectName, -1, 'odom'))
      -- To send several transforms at once, use simROS.sendTransforms instead
   end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
  if rosInterfacePresent then
    --simROS.shutdownSubscriber(subscriber1)
    --simROS.shutdownSubscriber(subscriber2)
    simROS.shutdownSubscriber(subscriber3)
    simROS.shutdownSubscriber(subscriber4)
    simROS.shutdownSubscriber(subscriber5)

    simROS.shutdownPublisher(publisher1)
    simROS.shutdownPublisher(publisher2)
    simROS.shutdownPublisher(publisher3)
    simROS.shutdownPublisher(publisher_acc)
    simROS.shutdownPublisher(publisher_enc)
    simROS.shutdownPublisher(publisher_gps)
    simROS.shutdownPublisher(publisher_comp)
  end
end
