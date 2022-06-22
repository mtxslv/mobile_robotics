from sympy import Polygon, Line, Point

def is_there_shared_point(s1,s2):
    if s1.p1 == s2.p1 or s1.p1 == s2.p2 or s1.p2 == s2.p1 or s1.p2 == s2.p2:
        return True
    else:
        return False
        
def find_shared_segment(polygon_1, polygon_2):
    shared_segment = None
    for side1 in polygon_1.sides:
        if shared_segment != None:
            break
        for side2 in polygon_2.sides:
            if is_there_shared_point(side1,side2): # this guarantee two segments share at least one extreme point
                if side2.direction.evalf()[0] == 0: # this guarantee the segment is vertical
                    shared_segment = side2
                    break
    if shared_segment != None:
        return shared_segment
    else:
        return -1