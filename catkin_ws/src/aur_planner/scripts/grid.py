"""
Continuous-map for Bluerov2 simulations




"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl
from scipy.ndimage import gaussian_filter
import math


def findIndex(x,y,X,Y,gridsize):
    indexx,indexy = 0, 0
    valx, valy = abs(x-X[0]-gridsize/2), abs(y-Y[0]-gridsize/2)
    for n in range(0,len(X)):
        if abs(x-X[n]-gridsize/2) < valx:
            indexx = indexx + 1
            valx = abs(x-X[n]-gridsize/2)

    for n in range(0,len(Y)):
        if abs(y-Y[n]-gridsize/2) < valy:
            indexy = indexy + 1
            valy = abs(y-Y[n]-gridsize/2)

    return indexx, indexy

def contMap(x, y, z,x2,y2,z2, gridsize,smooth, thres, sat_limit, sources):
    print("min(min(x),min(x2)) = ",min(min(x),min(x2)))
    X = np.arange(min(min(x),min(x2)),max(max(x),max(x2)),gridsize)
    Y = np.arange(min(min(y),min(y2)),max(max(y),max(y2)),gridsize)
    X2 = X
    Y2 = Y
    
    
    clmap = "jet"
    pad = 1
    padding_mode = "linear_ramp"
    data = np.zeros((len(Y), len(X)))

    sum1, sum2 = 0.0, 0.0

    # Find number of samples above threshold
    for index in z:
        if index >= thres:
            sum1 = sum1+1
    for index in z2:
        if index >= thres:
            sum2 = sum2+1

    # Calculate total distance traveled by bluerov
    dist1, dist2 = 0.0, 0.0
    for i in range(1,len(x)):
        dist1 = dist1 + math.sqrt((x[i]-x[i-1])**2+(y[i]-y[i-1])**2)
    for i in range(1,len(x2)):
        dist2 = dist2 + math.sqrt((x2[i]-x2[i-1])**2+(y2[i]-y2[i-1])**2)        


    for n in range(len(x)):
        ix, iy = findIndex(x[n],y[n],X,Y,gridsize)
        if ((z[n]) > data[iy][ix]):
            data[iy][ix] = min(z[n],sat_limit)
        
    data2 = np.zeros((len(Y2), len(X2)))
    
    for n in range(len(x2)):
        ix, iy = findIndex(x2[n],y2[n],X2,Y2,gridsize)
        if ((z2[n]) > data2[iy][ix]):
            data2[iy][ix] = min(z2[n], sat_limit)

    # Padding
    
    data= np.pad(data, pad, mode = padding_mode)
    data2 = np.pad(data2, pad, mode = padding_mode)
    
    if (smooth):
        data = gaussian_filter(data, sigma=1)
        data2 = gaussian_filter(data2, sigma=1)

    data = data[pad:-pad][pad:-pad]
    data2 = data2[pad:-pad][pad:-pad]

    combined_data = np.array([data, data2])
    #Get the min and max of all your data
    print(np.shape(data), np.shape(data))


    _min, _max = np.min(combined_data), np.max(combined_data)
    #_min, _max = min(min(X),min(X2)), max(max(X),max(X2))

    print("Number of samples above threshold: sum1 = {}, sum2 = {}".format(sum1,sum2))
    print("Total distance traveled: dist1 = {}, dist2 = {}".format(dist1,dist2))
    print("samples/path: adaptive = {}, standard = {}".format(sum1/dist1,sum2/dist2))



    fig = plt.figure()
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Palatino"],
    })
    ax = fig.add_subplot(1, 2, 1)
    #Add the vmin and vmax arguments to set the color scale
    ax.imshow(data,cmap = clmap, extent=[min(X),max(X),min(Y),max(Y)],
                   origin="lower", interpolation='bilinear', vmin = _min, vmax = _max)
    plt.tight_layout()
    plt.scatter(x,y,s = 1, color = 'r')
    plt.scatter(sources[0],sources[1],s = 200, color = 'k', marker = "+")
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    ax.autoscale(False)
    ax.set_xlim([min(X), max(X)])
    ax.set_ylim([min(Y), max(Y)])
    ax2 = fig.add_subplot(1, 2, 2)

    #Add the vmin and vmax arguments to set the color scale
    ax2.imshow(data2,cmap = clmap, extent=[min(X),max(X),min(Y),max(Y)],
                   origin="lower", interpolation='bilinear', vmin = _min, vmax = _max)
    plt.tight_layout()
    plt.scatter(x2,y2,s = 1, color = 'r')
    plt.scatter(sources[0],sources[1],s = 200, color = 'k', marker = "+")
    ax2.autoscale(False)
    norm = mpl.colors.Normalize(vmin=_min, vmax=_max)
    sm = plt.cm.ScalarMappable(norm=norm, cmap = clmap)
    cbar = plt.colorbar(sm).ax.set_ylabel(r"Intensity \bigg[$\displaystyle\frac{W}{m^2}$\bigg]", rotation=270, labelpad=30)
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    ax2.set_xlim([min(X), max(X)])
    ax2.set_ylim([min(Y), max(Y)])
    plt.tight_layout()
    fig.tight_layout()
    fig.savefig('sim_map.png',format="png", bbox_inches='tight')
    # plt.show()
    

def is_float(element):
    try:
        float(element)
        return True
    except ValueError:
        return False

def main():
    
    f = open('../../../../EVERYTHING/scripts/poses-simulation05.txt',"r")
 
    data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])

    data2 = np.array(data[1:-1])
    
    meas = data2[:,1]
    x = data2[:,2]
    y = data2[:,3]
    f = open('../../../../EVERYTHING/scripts/poses-simulation07.txt',"r")
    data = ([[float(elem) for elem in row.split(",") if is_float(elem)] for row in f.read().split("\n") ])
    data2 = np.array(data[1:-1])
    
    
    meas2 = data2[:,1]
    x2 = data2[:,2]
    y2 = data2[:,3]

    sources = np.array([[10, 10,10, 12.5, 12.5, 2], [15, 20, 22,20, 25,40]])
    contMap(x,y,meas,x2,y2,meas2, 0.98765,True, 0.01, 2, sources)



if __name__ == "__main__":
    main()


