

def isAngleClose(angle1, angle2, checkAngle=1.5):
    return abs(getDeltaAngle(angle1, angle2)) < checkAngle


def angleToCoords(angle):
    if angle < 0:
        angle = 360 + angle
    elif angle >= 360:
        angle = angle % 360
    return angle


def getDeltaAngle(absolute, other):
    da = 0.
    if absolute < 180:
        if absolute <= other <= absolute + 180:
            da = other - absolute
        else:
            if other < absolute:
                da = absolute - other
            else:
                da = absolute + (360 - other)
            da *= -1
    else:  # > 180
        if absolute - 180 <= other <= absolute:
            da = other - absolute
        else:
            if other > absolute:
                da = other - absolute
            else:
                da = other + (360 - absolute)

    return da


if __name__ == '__main__':
    print(getDeltaAngle(225, 13))
    print(getDeltaAngle(13, 225))

    print(getDeltaAngle(13, 156))
    print(getDeltaAngle(156, 13))


