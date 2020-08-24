from numbers import Number
from math import sqrt


class Ship:
    def __init__(self, position, speed, heading):

        if isinstance(position, tuple) == False:
            raise NameError('Your ship instance has not a valid tuple position')
        if isinstance(position[0], Number) == False:
            raise NameError('Your ship instance has not a valid tuple position')
        if isinstance(position[1], Number) == False:
            raise NameError('Your ship instance has not a valid tuple position')
        if isinstance(speed, Number) == False:
            raise NameError('Your ship instance speed is not a float number')
        if isinstance(heading, Number) == False:
            raise NameError('Your ship instance speed is not a float number')

        self.position = position
        self.speed = speed
        self.heading = heading


def ARPA_calculations(objectA, objectB):
    if isinstance(objectA, Ship) == False or isinstance(objectB, Ship) == False:
        raise NameError('This function is only usable with Ship instances')

    pointA = objectA.position
    objectA_speed = objectA.speed
    vectorA_angle = objectA.heading
    pointB = objectB.position
    objectB_speed = objectB.speed
    vectorB_angle = objectB.heading

    # Calculation of relative object datas from object B to object A
    (vectorB_angle_relativ, objectB_speed_relative) = calculate_relative_vector(pointA, objectA_speed, vectorA_angle,
                                                                                pointB, objectB_speed, vectorB_angle)

    # if two objects have same speed or they are static, then CPA = their current distance
    if ((objectA_speed == objectB_speed) and (vectorA_angle == vectorB_angle)) or (
            objectA_speed <= 0.001 and objectB_speed <= 0.001):

        print("Already at her minimum CPA")
        cpa = round(calculate_distance(pointA, pointB), 2)

        return {'cpa': cpa, 'tcpa': 0}

    elif pointA == pointB:
        print("Objects in the same position")
        return {'cpa': 0, 'tcpa': 0}

    else:

        # Check if the object is already at his minimum CPA
        if check_ship_going_away(pointA, vectorA_angle, pointB, vectorB_angle_relativ, objectB_speed_relative) == True:

            print("Ship going away, already at her minimum CPA")
            cpa = round(calculate_distance(pointA, pointB), 3)

            return {'cpa': cpa, 'tcpa': 0}

        else:

            # Calculation of the crossing lines position
            cp_position = calculate_cp_position(pointA, objectA_speed, vectorA_angle, pointB, objectB_speed,
                                                vectorB_angle, vectorB_angle_relativ, objectB_speed_relative)

            # Is the CPA position ahead or astern the ship's beam ?
            signe = calculate_CPA_sign(vectorA_angle, pointA, cp_position)

            cpa = round(calculate_distance(pointA, cp_position) * signe, 3)
            # [s]
            tcpa = (calculate_distance(pointB, cp_position) / objectB_speed_relative) * 60.0

            latAcpa, lonAcpa = calculate_future_position(pointA, objectA_speed * tcpa / 60, vectorA_angle)
            latBcpa, lonBcpa = calculate_future_position(pointB, objectB_speed * tcpa / 60, vectorB_angle)

            return {'cpa': cpa, 'tcpa': tcpa}


def calculate_distance(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    return sqrt((pointA[0]-pointB[0])**2 + (pointA[1]-pointB[1])**2)


def calculate_future_position(pointA, object_speed, vector_angle):

    object_speed = object_speed*1.852

    lat1 = radians(pointA[0])
    lon1 = radians(pointA[1])
    vector_angle = radians(vector_angle)

    lat2 = asin(sin(lat1)*cos(object_speed/6378.137)+cos(lat1)*sin(object_speed/6378.137)*cos(vector_angle))
    lon2 = lon1+ atan2(sin(vector_angle)*sin(object_speed/6378.137)*cos(lat1), cos(object_speed/6378.137)-sin(lat1)*sin(lat2))

    return( round(degrees(lat2),7),round(degrees(lon2),7))


def calculate_bearing(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = radians(pointA[0])
    lat2 = radians(pointB[0])

    diffLong = radians(pointB[1] - pointA[1])

    x = sin(diffLong) * cos(lat2)
    y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(diffLong))

    initial_bearing = atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180deg to + 180deg which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below

    initial_bearing = degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing


def calculate_relative_vector(pointA, objectA_speed, vectorA_angle, pointB, objectB_speed, vectorB_angle):
    # Calculation of objects' positions one hour later
    pointB2 = calculate_future_position(pointB, objectB_speed, vectorB_angle)
    pointA2 = calculate_future_position(pointA, objectA_speed, vectorA_angle)
    vectorA_angle_opp = calculate_bearing(pointA2, pointA)
    pointA3 = calculate_future_position(pointB2, objectA_speed, vectorA_angle_opp)

    vectorB_angle_relativ = calculate_bearing(pointB, pointA3)
    objectB_speed_relative = calculate_distance(pointB, pointA3)

    vectorB_angle_relative = 
    return vectorB_angle_relativ, objectB_speed_relative

