require ("math")

function subscriber_cmd_vel_callback(msg)
   -- This is the subscriber callback function when receiving /cmd_vel  topic
   -- The msg is a Lua table defining linear and angular velocities
   --   linear velocity along x = msg["linear"]["x"]
   --   linear velocity along y = msg["linear"]["y"]
   --   linear velocity along z = msg["linear"]["z"]
   --   angular velocity along x = msg["angular"]["x"]
   --   angular velocity along y = msg["angular"]["y"]
   --   angular velocity along z = msg["angular"]["z"]
   spdLin = msg["linear"]["x"]*10.0
   spdAng = msg["angular"]["z"]*10.0
   kLin = -0.5
   kAng = -0.2
   spdLeft = kLin*spdLin+kAng*spdAng
   spdRight = kLin*spdLin-kAng*spdAng
   sim.setJointTargetVelocity(leftFrontMotor,spdLeft)
   sim.setJointTargetVelocity(rightFrontMotor,spdRight)
   sim.setJointTargetVelocity(leftRearMotor,spdLeft)
   sim.setJointTargetVelocity(rightRearMotor,spdRight)
    sim.addStatusbarMessage('cmd_vel subscriber receiver : spdLin ='..spdLin..',spdAng='..spdAng.." command : spdLeft="..spdLeft..",act="..spdRight)
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
   print ("init")
   objectName="Water"
   WaterObj=sim.getObjectHandle(objectName)
   objectName2="Sphere"
   SphereObj=sim.getObjectHandle(objectName2)
   referenceName="ResizableFloor_5_25"
   referenceHandle=sim.getObjectHandle(referenceName)
  
   rosInterfacePresent=simROS
   -- Prepare the publishers and subscribers :
   if rosInterfacePresent then
      publisher1=simROS.advertise('/simulationTime','std_msgs/Float32')
      publisher2=simROS.advertise('/pose','geometry_msgs/Pose')
      publisher3=simROS.advertise('/centralAxisAngle','std_msgs/Float32')
      --subscriber1=simROS.subscribe('/cmd_vel','geometry_msgs/Twist','subscriber_cmd_vel_callback')
   end
end

function sysCall_actuation()
   -- Send an updated simulation time message, send the transform of the central axis
   -- and send the angle of the central axis
   if rosInterfacePresent then
      -- publish time, pose and angle topics
      simROS.publish(publisher1,{data=sim.getSimulationTime()})
      simROS.publish(publisher2,getPose("Prismatic_joint"))
      centralAxisAngle = sim.getJointPosition(centralAxis)*180.0/math.pi
      simROS.publish(publisher3,{data=centralAxisAngle})
      -- send a TF  :  robot w.r.t. floor
      simROS.sendTransform(getTransformStamped(objectHandle,objectName,referenceHandle,referenceName))
      -- To send several transforms at once, use simROS.sendTransforms instead
   end
end

function sysCall_cleanup()
   -- Following not really needed in a simulation script (i.e. automatically shut down
   -- at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher1)
        simROS.shutdownPublisher(publisher2)
        simROS.shutdownPublisher(publisher3)
        simROS.shutdownSubscriber(subscriber1)
    end
end
