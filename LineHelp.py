import GraphDomain as gd

#the following 2 functions where taken from https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
#by user Grumdrig

def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)


# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):

    #
    # TODO add in colinear detection with slopes
    if(B.x==A.x or D.x==C.x):
        return None
    if(abs((B.y-A.y)/(B.x-A.x))==abs((D.y-C.y)/(D.x-C.x))):
        return None
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)