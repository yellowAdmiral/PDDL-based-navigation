def get_goals_in_range(x, y):
    """
    The function is expected to be given the `x` and `y` coordinates 
    of the robot. It returns a list with the name of the goals that
    are considered "close" i.e., under 0.8m.
    
    To use this function you will need to install the module 'shapely'.
    """ 

    import shapely

    red = shapely.Polygon([
      (3.18, 2.12),
      (4.19, 2.12),
      (4.19, 3.90),
      (3.18, 3.90)])

    green = shapely.Polygon([
      (3.18, -4.86),
      (4.19, -4.86),
      (4.19, -3.02),
      (3.18, -3.02)])

    ducks = shapely.Polygon([
      (-2.59, 3.90),
      (-3.07, 3.08),
      (-3.55, 3.37),
      (-3.09, 4.19)])

    balls = shapely.Polygon([
      (-2.55, -3.67),
      (-2.25, -4.20),
      (-3.09, -4.67),
      (-3.34, -4.15)])


    goals = {'red': red, 'green': green, 'ducks': ducks, 'balls': balls }

    ret = []
    for goal in goals.keys():
        distance = goals[goal].distance(shapely.Point(x, y))
        if distance < 0.8:
            ret.append(goal)

    return ret

