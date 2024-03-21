import numpy as np

def save_Qualisys(Qualisys=[0,0,0], a = [0,0,0,0,0,0], time: float = 0):
    '''
    [data] et time
    '''
    data_array = np.array((time, Qualisys,a), dtype=[('TIME', float), ('DATA', float, (3,)), ('angle', float, (6,))])
    print(np.append(data_array,np.array((time, Qualisys,a), dtype=[('TIME', float), ('DATA', float, (3,)), ('angle', float, (6,))])))
    print(data_array)

save_Qualisys([0,0,0],[0,0,0,0,0,0],4)