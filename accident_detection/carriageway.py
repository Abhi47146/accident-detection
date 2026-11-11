import math
def get_carriageway(center,line_config):

    x, y = center

    x1, y1 = (line_config[0],line_config[1])
    x2, y2 = (line_config[2],line_config[3])
    x1=int(x1)
    x2=int(x2)
    y1=int(y1)
    y2 = int(y2)

    value = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

    if value > 0:
        return "LEFT"

    elif value < 0:
        return "RIGHT"

    else:
        return "DIVIDER"