function angle_diff(a, b)
{
	return ((a - b + 3.141592653589) % (3.141592653589*2)) - 3.141592653589;
}

function getUpperAndLowerPathPts(xOrig, yOrig, dAngle, spacing)
{
	var pts = [];
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
	console.log("REACHED_1")
	var speedPts = []

	act_robot_speed = 30;//150;//100;//Max speed for 1 side
	act_robot_acc = 60;//90;//Max acc for 1 side
	currentVelU = 0;
    currentVelL = 0;
	var TIME_STEP = 0.005;
	var LEN_STEP = 0.000000000001;
	var curveLimit = 0;
	
	    //First generate our slow down endpts
	var act_robot_decc = act_robot_decc * 1.5;
    var DATA_COUNT_NEEDED = Math.ceil(act_robot_speed / act_robot_decc / TIME_STEP)
    var BACK_LEN_STEP = 0.00001;
    var intgrLimits = [];
    for (kk = 0; kk < constants.length; kk++)
    {
        if (kk == constants.length - 1 || constants[kk][2] != constants[kk + 1][2])
        {
            var bCurveLimit = 10;
            var bSpeedPts = [];
            var derivAngle = Math.atan2(derivative(bCurveLimit, constants[kk][1]), derivative(bCurveLimit, constants[kk][0]));
            var currentPts = getUpperAndLowerPathPts(f(bCurveLimit, constants[kk][0]), f(bCurveLimit, constants[kk][1]), derivAngle, 16);
            var bSpeedU = 0;
            var bSpeedL = 0;
            var bDistU = (bSpeedU + act_robot_decc * TIME_STEP) * TIME_STEP;
            var bDistL = (bSpeedL + act_robot_decc * TIME_STEP) * TIME_STEP;
            var bDistTravU = 0;
            var bDistTravL = 0;

            var xU2 = currentPts[0][0];
            var yU2 = currentPts[0][1];
            var xL2 = currentPts[1][0];
            var yL2 = currentPts[1][1];
            bCurveLimit -= BACK_LEN_STEP
            var tUDist = 0;
            var tLDist = 0;
            while (bSpeedPts.length < DATA_COUNT_NEEDED)
            {
                derivAngle = Math.atan2(derivative(bCurveLimit, constants[kk][1]), derivative(bCurveLimit, constants[kk][0]));
                currentPts = getUpperAndLowerPathPts(f(bCurveLimit, constants[kk][0]), f(bCurveLimit, constants[kk][1]), derivAngle, 16);
                var xU1 = currentPts[0][0];
                var yU1 = currentPts[0][1];
                var xL1 = currentPts[1][0];
                var yL1 = currentPts[1][1];
                //if (Math.abs(angle_diff(Math.atan2((yL2 - yL1), (xL2 - xL1)), derivAngle + 3.141592)) >= 3.141592 * 2 / 3)
                //    bDistTravU -= Math.sqrt((xU1 - xU2) * (xU1 - xU2) + (yU1 - yU2) * (yU1 - yU2))
                //else
                    bDistTravU += Math.sqrt((xU1 - xU2) * (xU1 - xU2) + (yU1 - yU2) * (yU1 - yU2))
				//console.log(bDistTravU);
                //if (Math.abs(angle_diff(Math.atan2((yU2 - yU1), (xU2 - xU1)), derivAngle + 3.141592)) >= 3.141592 * 2 / 3)
                //    bDistTravL -= Math.sqrt((xL1 - xL2) * (xL1 - xL2) + (yL1 - yL2) * (yL1 - yL2))
                //else
                    bDistTravL += Math.sqrt((xL1 - xL2) * (xL1 - xL2) + (yL1 - yL2) * (yL1 - yL2))
                if (bDistTravU > bDistU || bDistTravL > bDistL)
                {
                    let bUSpeed = bDistTravU / TIME_STEP;
                    let bLSpeed = bDistTravL / TIME_STEP;
                    if (constants[kk][2])
                    {
                        bSpeedPts.push([-1 * bUSpeed, bLSpeed, derivAngle * 180 / 3.1415926535 - 180, 0, 0, TIME_STEP]);
                    }
                    else
                    {
                        bSpeedPts.push([bLSpeed, -1 * bUSpeed, derivAngle * 180 / 3.1415926535, 0, 0, TIME_STEP]);
                    }
                    bSpeedU = bUSpeed;
                    bSpeedL = bLSpeed;
                    bDistTravU = 0;
                    bDistTravL = 0;
                    bDistU = Math.min((bSpeedU + act_robot_decc * TIME_STEP), act_robot_speed) * TIME_STEP;
                    bDistL = Math.min((bSpeedL + act_robot_decc * TIME_STEP), act_robot_speed) * TIME_STEP;
                }
                xU2 = xU1;
                yU2 = yU1;
                xL2 = xL1;
                yL2 = yL1;
                bCurveLimit -= BACK_LEN_STEP;
            }
            bSpeedPts.reverse();
            intgrLimits.push([bCurveLimit, bSpeedPts]);
        }
        else
            intgrLimits.push([10]);
    }
	console.log("REACHED_0");
	
    curDistUpper = 0;
    curDistLower = 0;
	
	var totalError = 0;
    for (ii = 0; ii < constants.length; ii++)//accelerating
    {
		maxVelUpper = Math.min(act_robot_speed, currentVelU + act_robot_acc * TIME_STEP);
		maxDistUpper = (maxVelUpper) * TIME_STEP;
		maxVelLower = Math.min(act_robot_speed, currentVelL + act_robot_acc * TIME_STEP);
		maxDistLower = (maxVelLower) * TIME_STEP;
    
        curveLimit = 0;
		var derivAngle = Math.atan2(derivative(curveLimit, constants[ii][1]), derivative(curveLimit, constants[ii][0]));
		var initialPts = getUpperAndLowerPathPts(f(curveLimit, constants[ii][0]), f(curveLimit, constants[ii][1]), derivAngle, 16)
        var xPosUpper2 = initialPts[0][0];
        var yPosUpper2 = initialPts[0][1];
        var xPosLower2 = initialPts[1][0];
        var yPosLower2 = initialPts[1][1];
		curveLimit = 0.00000001;
		while(curveLimit < intgrLimits[ii][0])
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
				
                if (dir != -1)
					speedPts.push([lowerSpeedPt, -1 * upperSpeedPt, derivAngle * 180 / 3.1415926535, lowerDist * -1, upperDist * -1, TIME_STEP]);
				else
					speedPts.push([upperSpeedPt, -1 * lowerSpeedPt, derivAngle * 180 / 3.1415926535 - 180, upperDist, lowerDist, TIME_STEP]);
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
		if (intgrLimits[ii][0] != 10)
		{
			currentVelU = 0;
			currentVelL = 0;
			curDistUpper = 0;
			curDistLower = 0;
			speedPts = speedPts.concat(intgrLimits[ii][1]);
		}
		
			
	}
	var IN_TO_R = 1 / (6.00 * 3.14159265359)
	var TO_RPM = 60 * IN_TO_R

	var tot_time = 0;
	for (p = 0; p < speedPts.length; p++)
	{
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
