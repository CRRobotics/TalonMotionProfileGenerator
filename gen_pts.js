function angle_diff(a, b)
{
	return ((a - b + 3.141592653589) % (3.141592653589*2)) - 3.141592653589;
}

function getUpperAndLowerPathPts(xOrig, yOrig, dAngle, spacing)
{
	var pts = [];
	/*if (Math.abs(m) < 0.0001)
	{
		pts.push([xOrig, yOrig + spacing]);
		pts.push([xOrig, yOrig - spacing]);
	}
	else
	{
		var perpendicular = -1 / m;
		var dx = Math.sqrt((spacing * spacing) / (1 + perpendicular * perpendicular));
		var dy = dx * perpendicular;
		if (m > 0)
		{
			pts.push([xOrig - dx, yOrig - dy]);
			pts.push([xOrig + dx, yOrig + dy]);
		}
		else
		{
			pts.push([xOrig + dx, yOrig + dy]);
			pts.push([xOrig - dx, yOrig - dy]);
		}
	}
	return pts;*/
    pts.push([xOrig + spacing * Math.cos(dAngle + 1.57079632679), yOrig + spacing * Math.sin(dAngle + 1.57079632679)]);
    pts.push([xOrig + spacing * Math.cos(dAngle - 1.57079632679), yOrig + spacing * Math.sin(dAngle - 1.57079632679)]);
    return pts;
}

var POLY_DEG = 5;
//Gets the y-value of a 5th degree polynomial defined by coefficients at x
function f(x, coefficients)
{
	var sum = 0;
	for (i = POLY_DEG; i >= 0; i--)
	{
		sum += Math.pow(x, i) * coefficients[POLY_DEG - i];
	}
	return sum;
}

//Gets dy/dx of a 5th degree polynomial defined by coefficients at x
function derivative(x, coefficients)
{
	var sum = 0;
	for (i = POLY_DEG - 1; i >= 0; i--)
	{
		sum += (i + 1) * Math.pow(x, i) * coefficients[POLY_DEG - 1 - i];
	}
	return sum;
}

//Gets d2y/dx2 of a 5th degree polynomial defined by coefficients at x
function acceleration(x, coefficients)
{
	var sum = 0;
	for (i = POLY_DEG - 2; i >= 0; i--)
	{
		sum += (i + 2) * (i + 1) * Math.pow(x, i) * coefficients[POLY_DEG - 2 - i];
	}
	return sum;
}


onmessage = function(msg) {
	var d = msg.data;
	postMessage({id: d.id, data: genPts(d.data)})
	close();
}

function genPts(constants)
{
	var speedPts = []

	act_robot_speed = 100;//Max speed for 1 side
	act_robot_acc = 180;//Max acc for 1 side
	currentVelU = 0;
    currentVelL = 0;
	var TIME_STEP = 0.005;
	var LEN_STEP = 0.000000000001;
	var curveLimit = 0;
    maxVelUpper = Math.min(act_robot_speed, currentVelU + act_robot_acc * TIME_STEP);
    maxDistUpper = (maxVelUpper) * TIME_STEP;
    maxVelLower = Math.min(act_robot_speed, currentVelL + act_robot_acc * TIME_STEP);
    maxDistLower = (maxVelLower) * TIME_STEP;
    curDistUpper = 0;
    curDistLower = 0;
		
	var totalError = 0;
    for (ii = 0; ii < constants.length; ii++)//accelerating
    {
        curveLimit = 0;
		var derivAngle = Math.atan2(derivative(curveLimit, constants[ii][1]), derivative(curveLimit, constants[ii][0]));
		var initialPts = getUpperAndLowerPathPts(f(curveLimit, constants[ii][0]), f(curveLimit, constants[ii][1]), derivAngle, 16)
        var xPosUpper2 = initialPts[0][0];
        var yPosUpper2 = initialPts[0][1];
        var xPosLower2 = initialPts[1][0];
        var yPosLower2 = initialPts[1][1];
		curveLimit = 0.00000001;
		while (curveLimit < 10/* && curveLimit > -10*/)
        {
            var derivAngle = Math.atan2(derivative(curveLimit, constants[ii][1]), derivative(curveLimit, constants[ii][0]));
            var currentPts = getUpperAndLowerPathPts(f(curveLimit, constants[ii][0]), f(curveLimit, constants[ii][1]), derivAngle, 16);
            var xPosUpper1 = currentPts[0][0];
            var yPosUpper1 = currentPts[0][1];
            var xPosLower1 = currentPts[1][0];
            var yPosLower1 = currentPts[1][1];
            var upperDistTraveled = Math.sqrt((xPosUpper1 - xPosUpper2) * (xPosUpper1 - xPosUpper2) + (yPosUpper1 - yPosUpper2) * (yPosUpper1 - yPosUpper2));
            var lowerDistTraveled = Math.sqrt((xPosLower1 - xPosLower2) * (xPosLower1 - xPosLower2) + (yPosLower1 - yPosLower2) * (yPosLower1 - yPosLower2));
            if (Math.abs(angle_diff(Math.atan2((yPosUpper1 - yPosUpper2), (xPosUpper1 - xPosUpper2)), derivAngle)) >= 3.141592 * 2 / 3)
			{
				//console.log("Upper Negative Triggered");
				upperDistTraveled *= -1;
			}
			if (Math.abs(angle_diff(Math.atan2((yPosLower1 - yPosLower2), (xPosLower1 - xPosLower2)), derivAngle)) >= 3.141592 * 2 / 3)
			{
				//console.log("Lower Negative Triggered");
				lowerDistTraveled *= -1;
			}
			
			curDistUpper += upperDistTraveled;
            curDistLower += lowerDistTraveled;
			xPosUpper2 = xPosUpper1;
            yPosUpper2 = yPosUpper1;
            xPosLower2 = xPosLower1;
            yPosLower2 = yPosLower1;
			if (curDistUpper > maxDistUpper || curDistLower > maxDistLower)
            {
                var upperDist = Math.max(Math.min(curDistUpper, maxDistUpper), 0);
                var lowerDist = Math.max(Math.min(curDistLower, maxDistLower), 0);
                var dir = 1;
                if (constants[ii][2])
                    dir = -1;
                var upperSpeedPt = upperDist / TIME_STEP;
                var lowerSpeedPt = lowerDist / TIME_STEP;
                currentVelU = upperSpeedPt;
                currentVelL = lowerSpeedPt;
                upperSpeedPt = upperSpeedPt * dir;
                lowerSpeedPt = lowerSpeedPt * dir;
                
				postMessage({"drawBlip": [xPosLower2, yPosLower2, "#00ff00"]})
				postMessage({"drawBlip": [xPosUpper2, yPosUpper2, "#00ff00"]})
				//drawBlip(xPosLower2, yPosLower2, "#00ff00", ctx2);
				//drawBlip(xPosUpper2, yPosUpper2, "#00ff00", ctx2);
				if (curDistUpper < 0)
					totalError += Math.abs(curDistUpper);
				if (curDistLower < 0)
					totalError += Math.abs(curDistLower);
				
				//This will introduce some minor error, but it should be negligible
                curDistUpper = Math.max(curDistUpper - maxDistUpper, 0);
                curDistLower = Math.max(curDistLower - maxDistLower, 0);
				totalError += Math.max(curDistUpper, curDistLower);
				
                if (dir == -1)
					speedPts.push([lowerSpeedPt, upperSpeedPt, derivAngle, lowerDist * -1, upperDist * -1, TIME_STEP]);
				else
					speedPts.push([upperSpeedPt, lowerSpeedPt, derivAngle, upperDist, lowerDist, TIME_STEP]);
                maxVelUpper = Math.min(act_robot_speed, currentVelU + act_robot_acc * TIME_STEP);
                maxDistUpper = (maxVelUpper) * TIME_STEP;
                maxVelLower = Math.min(act_robot_speed, currentVelL + act_robot_acc * TIME_STEP);
                maxDistLower = (maxVelLower) * TIME_STEP;
            }
            var increment = Math.abs(LEN_STEP / Math.max(upperDistTraveled, lowerDistTraveled));
			curveLimit += increment;
			upperDistTraveled = 0;
			lowerDistTraveled = 0;
        }
		if (ii == constants.length - 1 || constants[ii][2] != constants[ii + 1][2]){
			var slowDownTime = Math.floor(act_robot_speed * 0.5 / act_robot_acc / TIME_STEP);
			for (p = 0; p < slowDownTime; p++){
				speedPts[speedPts.length - 1 - p][0] = speedPts[speedPts.length - 1 - p][0] * (p * 0.7 + slowDownTime * 0.3) / slowDownTime;
				speedPts[speedPts.length - 1 - p][1] = speedPts[speedPts.length - 1 - p][1] * (p * 0.7 + slowDownTime * 0.3) / slowDownTime;
				speedPts[speedPts.length - 1 - p][5] = speedPts[speedPts.length - 1 - p][5] * slowDownTime / (p * 0.7 + slowDownTime * 0.3);
			}
		}
    }
	var IN_TO_R = 1 / (3.94 * 3.14159265359)
	var TO_RPM = 60 * IN_TO_R
	 
	var tot_time = 0;
	for (p = 0; p < speedPts.length; p++){
		speedPts[p].push(tot_time);
		tot_time += speedPts[p][5];
		speedPts[p][0] = speedPts[p][0] * TO_RPM;
		speedPts[p][1] = speedPts[p][1] * TO_RPM;
		speedPts[p][3] = speedPts[p][0] * IN_TO_R;
		speedPts[p][4] = speedPts[p][1] * IN_TO_R;
	}
	console.log("TOTAL_ERROR " + totalError);
	console.log("TIME " + tot_time);
	return speedPts;
}
