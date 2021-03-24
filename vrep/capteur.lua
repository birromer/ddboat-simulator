
function getCompass(objectName)
  -- This function get the value of the compass 
  objectHandle=sim.getObjectHandle(objectName)
  relTo = -1
  o=sim.getObjectOrientation(objectHandle,relTo)
  heading = -0[3]   -- north along X > 0

  return heading
end
