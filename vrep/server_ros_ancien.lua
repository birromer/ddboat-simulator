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

function subscriber_cmd_ul_callback(msg)
  spdMotor1 = msg["data"]
  sim.addForceAndTorque(MotorLeft,{0,spdMotor1,0})

  sim.addStatusbarMessage('cmd_ul subscriber receiver : wheels speed ='..spdMotor1)
end

function subscriber_cmd_ur_callback(msg)
  spdMotor1 = msg["data"]
  sim.addForceAndTorque(MotorRight,{0,spdMotor1,0})

  sim.addStatusbarMessage('cmd_ur subscriber receiver : wheels speed ='..spdMotor1)
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
 -- objectName = "Corps_boat"
  objectHandle =sim.getObjectAssociatedWithScript(sim.handle_self)

  -- get left and right motors handles
  MotorLeft = sim.getObjectHandle("motor_left")
  MotorRight = sim.getObjectHandle("motor_right")
 

  rosInterfacePresent = simROS

  -- Prepare the publishers and subscribers :
  if rosInterfacePresent then
    publisher1 = simROS.advertise('/simulationTime','std_msgs/Float32')
    publisher2 = simROS.advertise('/pose','geometry_msgs/Pose')

    --subscriber1=simROS.subscribe('/cmd_u1','std_msgs/Float32','subscriber_cmd_u1_callback')
    --subscriber2=simROS.subscribe('/cmd_u2','std_msgs/Float32','subscriber_cmd_u2_callback')
    --subscriber3=simROS.subscribe('/cmd_u3','std_msgs/Float32','subscriber_cmd_u3_callback')
    --subscriber4=simROS.subscribe('/cmd_u4','std_msgs/Float32','subscriber_cmd_u4_callback')

    subscriber3 = simROS.subscribe('/main/z_u1','std_msgs/Float32','subscriber_cmd_ul_callback')
    subscriber4 = simROS.subscribe('/main/z_u2','std_msgs/Float32','subscriber_cmd_ur_callback')
  end

  -- Get some handles (as usual !):
 -- onboard_camera = sim.getObjectHandle('OnBoardCamera')

  -- Enable an image publisher and subscriber:
  pub = simROS.advertise('/image', 'sensor_msgs/Image')
  simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
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
      objectHandle =sim.getObjectAssociatedWithScript(sim.handle_self)
      simROS.publish(publisher2, getPose(objectHandle))

      -- send a TF
      simROS.sendTransform(getTransformStamped(objectHandle,objectName,referenceHandle,referenceName))
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