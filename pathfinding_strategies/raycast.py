def _raycast(x1, y1, x2, y2):
    #supercover line algorithm
    #adapted from http://eugen.dedu.free.fr/projects/bresenham/
    x_list = []
    y_list = []
    i =0           # loop counter
    ystep=0
    xstep=0    # the step on y and x axis
    error=0           # the error accumulated during the increment
    errorprev=0       # *vision the previous value of the error variable
    y = y1 
    x = x1  # the line points
    ddy=0 
    ddx=0        # compulsory variables: the double values of dy and dx
    dx = x2 - x1
    dy = y2 - y1
    #POINT (y1, x1);  # first point
    x_list.append(x1)
    y_list.append(y1)
    # NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0):
        ystep = -1
        dy = -dy
    else:
        ystep = 1
    if (dx < 0):
        xstep = -1
        dx = -dx
    else:
        xstep = 1
    ddy = 2 * dy
    ddx = 2 * dx
    if (ddx >= ddy):  # first octant (0 <= slope <= 1)
        # compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx  # start in the middle of the square
        for i in range(dx):  # do not use the first point (already done)
            x += xstep
            error += ddy
            if (error > ddx):  # increment y if AFTER the middle ( > )
                y += ystep
                error -= ddx
                # three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx):  # bottom square also
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                elif (error + errorprev > ddx):  # left square also
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                else:  # corner: bottom and left squares also
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                    # #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                    pass
            
            #POINT (y, x)
            x_list.append(x)
            y_list.append(y)
            errorprev = error
        
    else:  # the same as above
        errorprev = error = dy
        for i in range(dy):
            y += ystep
            error += ddx
            if (error > ddy):
                x += xstep
                error -= ddy
                if (error + errorprev < ddy):
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                elif (error + errorprev > ddy):
                    #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                else:
                    #POINT (y, x-xstep)
                    x_list.append(x-xstep)
                    y_list.append(y)
                    # #POINT (y-ystep, x)
                    x_list.append(x)
                    y_list.append(y-ystep)
                    pass
            
            #POINT (y, x)
            x_list.append(x)
            y_list.append(y)
            errorprev = error
    # assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
    return x_list, y_list

def hasLOS(grid, a, b):
    """
    uses raycasting to calculate LOS between nodes on an OccupancyGrid
    :param grid The occupancy grid
    :param a [(x,y)] the first point
    :param b [(x, y)] the second point
    :return     [bool] has LOS
    """
    #+0.5 on each direction to account for center-cell offset
    # x, y = PathPlanner.dda(a, b)
    x, y = _raycast(a[0], a[1], b[0], b[1])
    for i in range(len(x)):
        if(grid[y[i]][x[i]]>0.5):
            print("conflict on "+str((x[i], y[i])))
            return (x[i], y[i])
    return None