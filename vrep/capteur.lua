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



threadFunction=function()
    while sim.getSimulationState()~=sim_simulation_advancing_abouttostop do
        -- This is executed at each simulation step
      --local t0 = gettime()
      local simTime = sim.getSimulationTime()

      -- data sent 
      -- simulationTime
      -- gps {longitude, lattitude)
      -- compassValue {heading in degre}
      -- accelValuue {accel_x,accel_y, accel_z}
      -- gyroValue {gyro_x, gyro_y, gyro_z}
      -- encoderValue {leftEncoder, rightEncoder}

      -- get gps data
      local gps = getGPS(boat) -- -1 , location in world coordinates -- to complete boat
      -- get compass data
      local compassValue = getCompass(boat, false) --- to complete false or true
      -- get accelerometer data
      local accelValue = getAcceleration()
      -- get gyroscope data
      local  gyroValue = getGyrometer()
      --local stOut = "Dart Accel x="..accel[1]..", y="..accel[2]..", z="..accel[3]
      --stOut = stOut.." , Gyro x="..gyro[1]..", y="..gyro[2]..", z="..gyro[3]
      --print (stOut)

      --get encoder
      local encoderValue = getEncoder()

      -- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
      sim.switchThread() -- This thread will resume just before the main script is called again

    end
end

-- initialize communication tube with accelerometers and gyroscopes
accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.GetNameSuffix(nil),1) 
accel={0.0, 0.0, 0.0}
gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.GetNameSuffix(nil),1) -- put this in the initialization phase
gyro={0.0, 0.0, 0.0}
lastRelativeOrientation={}
lastOrientationTime={}
propellersCnt={}
propellers = {"LeftPropellers", "RightPropellers"} -- To complete

-- Execute the thread function:
res=false
err="not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   sim.addStatusbarMessage('Lua runtime error: '..err)
end
