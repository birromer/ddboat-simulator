require ("math")

function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end

function getCompass(objectName,motorON)
  -- This function get the value of the compass 
  objectHandle=sim.getObjectHandle(objectName)
  relTo = -1
  o=sim.getObjectOrientation(objectHandle,relTo)
  if statemotor then -- if motor is ON 
  	heading = -o[3] + gaussian(0,1)*math.pi  -- we add a gaussian noise
  else
  	heading = -o[3]   -- north along X > 0
  end
  return heading
end
