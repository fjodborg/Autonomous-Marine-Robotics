"""
Grid


"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl
from scipy.ndimage import gaussian_filter


def sourceValues(x,y,X,Y,gridsize):
    indexx,indexy = 0, 0
    valx, valy = abs(x-X[0]), abs(y-Y[0])
    for n in range(0,len(X)):
        if abs(x-X[n]) < valx:
            indexx = indexx + 1
            valx = abs(x-X[n])

    for n in range(0,len(Y)):
        if abs(y-Y[n]) < valy:
            indexy = indexy + 1
            valy = abs(y-Y[n])

def findIndex(x,y,X,Y,gridsize):
    indexx,indexy = 0, 0
    valx, valy = abs(x-X[0]), abs(y-Y[0])
    for n in range(0,len(X)):
        if abs(x-X[n]) < valx:
            indexx = indexx + 1
            valx = abs(x-X[n])

    for n in range(0,len(Y)):
        if abs(y-Y[n]) < valy:
            indexy = indexy + 1
            valy = abs(y-Y[n])

    return indexx, indexy
def createSourceMap(x,y,gridsize,smooth,xp,yp):

    X = np.arange(min(x),max(x),gridsize)
    Y = np.arange(min(y),max(y),gridsize)

    data = np.zeros((len(Y), len(X)))


def grid(x, y, z,x2,y2,z2, gridsize,smooth):

    X = np.arange(min(x),max(x),gridsize)
    Y = np.arange(min(y),max(y),gridsize)
    X2 = np.arange(min(x2),max(x2),gridsize)
    Y2 = np.arange(min(y2),max(y2),gridsize)
    
    
    clmap = "jet"
    data = np.zeros((len(Y), len(X)))

    for n in range(len(x)):
        ix, iy = findIndex(x[n],y[n],X,Y,gridsize)
        if ((z[n]) > data[iy][ix]):
            data[iy][ix] = np.sqrt(z[n])

    data2 = np.zeros((len(Y), len(X)))

    for n in range(len(x2)):
        ix, iy = findIndex(x2[n],y2[n],X2,Y2,gridsize)
        if ((z2[n]) > data2[iy][ix]):
            data2[iy][ix] = np.sqrt(z2[n])
    combined_data = np.array([data,data2])

    if (smooth):
        data = gaussian_filter(data, sigma=0.5)
        data2 = gaussian_filter(data2, sigma=0.5)
   
    #Get the min and max of all your data
    _min, _max = np.min(combined_data), np.max(combined_data)
    _max = 2

    print(_min, _max)
    fig = plt.figure()
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Palatino"],
    })
    ax = fig.add_subplot(1, 2, 1)
    #Add the vmin and vmax arguments to set the color scale
    ax.imshow(data,cmap = clmap, extent=[min(x),max(x),min(y),max(y)],
                   origin="lower", interpolation='bilinear', vmin = _min, vmax = _max)
    plt.scatter(x,y,s = 1, color = 'r')
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    ax.autoscale(False)
    ax2 = fig.add_subplot(1, 2, 2)

    #Add the vmin and vmax arguments to set the color scale
    ax2.imshow(data2,cmap = clmap, extent=[min(x),max(x),min(y),max(y)],
                   origin="lower", interpolation='bilinear', vmin = _min, vmax = _max)
    plt.scatter(x2,y2,s = 1, color = 'r')
    ax2.autoscale(False)
    norm = mpl.colors.Normalize(vmin=_min, vmax=_max)
    sm = plt.cm.ScalarMappable(norm=norm, cmap = clmap)

    cbar = plt.colorbar(sm).ax.set_ylabel(r"Intensity \bigg[$\displaystyle\frac{1}{m^2}$\bigg]", rotation=270, labelpad=15)
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    fig.savefig('sim_map.eps',format="eps")
    plt.show()
    """
    print("Data")
    print("np.amax(data = ", np.amax(data))
    data /=np.max(z)
    print("np.amax(data = ", np.amax(data))
    #print("max=", np.max(z))
    #print(data)
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)

    clmap = "gist_earth"

    if (smooth):
        data = gaussian_filter(data, sigma=1)
        ax.imshow(data,cmap = clmap, extent=[min(x),max(x),min(y),max(y)],
               origin="lower", interpolation='none',vmin=0.1, vmax = 4)
    else:
        ax.imshow(data, cmap = clmap, extent=[min(x),max(x),min(y),max(y)],
                   origin="lower", interpolation='none',vmin=0.1, vmax = 4)

    norm = mpl.colors.Normalize(vmin=0, vmax=thres)
    sm = plt.cm.ScalarMappable(norm=norm, cmap = clmap)
    ax.autoscale(False)
    plt.scatter(x,y,s = 1, color = 'r')
    plt.colorbar(sm)
    fig.savefig('hest')
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.show()
"""

def is_float(element):
    try:
        float(element)
        return True
    except ValueError:
        return False

if __name__ == "__main__":


    f = open('poses-simulation02.txt',"r")
    #f = open('poses.txt',"r")
    data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])

    data2 = np.array(data[1:-1])
    
    meas = data2[:,1]
    x = data2[:,2]
    y = data2[:,3]


    f = open('poses-simulation04.txt',"r")
    #f = open('poses.txt',"r")
    data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])

    data2 = np.array(data[1:-1])
    
    meas2 = data2[:,1]
    x2 = data2[:,2]
    y2 = data2[:,3]

    #x = [1,2,3,4,5,0.1,0.2,0.3,0.4,0.5]
    #y = [1,2,3,4,5,0.1,0.2,0.3,0.4,0.5]
    #meas = [1,1,1,1,1,1,1,1,1,2]
    grid(x,y,meas,x2,y2,meas2, 0.98765,True)
