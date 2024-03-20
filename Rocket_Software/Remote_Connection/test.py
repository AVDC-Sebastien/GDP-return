import numpy as np

print(np.array(((0,0,[0,0,0,0,0,0])),dtype=[('TIME',float),('ID',int),('DATA',float,(6))]))
def save_Camera_TOP(Camera : list = [0,0,0,[0,0,0,0,0,0]]):
    '''
    [T, id, n, [0,0,0,0,0,0]]
    '''
    print(np.array(tuple(Camera),dtype=[('TIME',float),('ID',float),('N',float),('DATA',float,(6))]))
save_Camera_TOP([0,0,0,[0,0,0,0,0,0]])