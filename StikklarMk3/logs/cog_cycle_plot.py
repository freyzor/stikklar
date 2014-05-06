import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import math
from matplotlib import ticker
from collections import namedtuple

d = "#cog,20114,0.00,-1.03,-32.19,2.29,-2.26,175.00,170.00,160,138.00,-170.00,130,-121.00,-170.00,160,-205.00,170.00,160"

Data = namedtuple("Data", [
    "time", "t",
    "xcx","xcy",
    "cogx","cogy",
    "alrx", "alry",
    "afbx", "afby",
    "rfx","rfy","rfz",
    "lfx","lfy","lfz",
    "lrx","lry","lrz",
    "rrx","rry","rrz",
    ])

def get_data_from_line(line):
    parts = line.split(",")
    args = [float(v) for v in parts[1:]]
    return Data(*args)

#data = get_data_from_line(d)

zcolors = [
    (0.0, 0.0, 1.0),
    (0.0, 0.5, 0.75),
    (0.0, 1.0, 0.5),
]

def get_z_marker(z):
    if z > 159:
        return "v"
    else:
        return "^"

def get_z_color(z):
    if z > 159:
        return 'b'
    else:
        return 'r'

def add_support_ploygon(ax, d):
    supportVerts = []
    outlineVerts = []
    if d.rfz > 159:
        supportVerts.append((d.rfx, d.rfy))
    outlineVerts.append((d.rfx, d.rfy))
    if d.lfz > 159:
        supportVerts.append((d.lfx, d.lfy))
    outlineVerts.append((d.lfx, d.lfy))
    if d.lrz > 159:
        supportVerts.append((d.lrx, d.lry))
    outlineVerts.append((d.lrx, d.lry))
    if d.rrz > 159:
        supportVerts.append((d.rrx, d.rry))
    outlineVerts.append((d.rrx, d.rry))
    if len(supportVerts) < 3:
        return
    # to close the poly
    supportVerts.append(supportVerts[0])
    supportCodes = [Path.MOVETO, Path.LINETO, Path.LINETO]
    
    if len(supportVerts) == 4:
        supportCodes.append(Path.CLOSEPOLY)
    else:
        supportCodes.append(Path.LINETO)
        supportCodes.append(Path.CLOSEPOLY)

    path = Path(supportVerts, supportCodes)
    color = 'blue' if len(supportVerts) == 5 else 'blue'
    patch = patches.PathPatch(path, facecolor=color, lw=2, zorder=2, alpha=0.1)
    ax.add_patch(patch)

    outlineVerts.append(outlineVerts[0])

    x, y = zip(*outlineVerts)
    ax.plot(x, y, "", c='b', zorder=3)

def draw_data(data, i):
    fig, ax = plt.subplots()

    # draw polygons
    add_support_ploygon(ax, data)

    # add COG and diagonal center
    ax.scatter([data.cogx], [data.cogy], c='g', s=50, marker='8', label="cog", zorder=20)
    ax.scatter([data.xcx], [data.xcy], c='r', s=50, marker='o', label="Xc", zorder=19)

    # foot placements
    ax.scatter([data.rfx], [data.rfy], c=get_z_color(data.rfz), s=75, marker=get_z_marker(data.rfz), label="rf", zorder=10)
    ax.scatter([data.lfx], [data.lfy], c=get_z_color(data.lfz), s=75, marker=get_z_marker(data.lfz), label="lf", zorder=10)
    ax.scatter([data.lrx], [data.lry], c=get_z_color(data.lrz), s=75, marker=get_z_marker(data.lrz), label="lr", zorder=10)
    ax.scatter([data.rrx], [data.rry], c=get_z_color(data.rrz), s=75, marker=get_z_marker(data.rrz), label="rr", zorder=10)
    
    # draw the diagonal lines between legs
    ax.plot([data.rfx, data.lrx], [data.rfy, data.lry], c=(0.4, 0.6, 0.9), zorder=5)
    ax.plot([data.lfx, data.rrx], [data.lfy, data.rry], c=(0.4, 0.6, 0.9), zorder=5)

    # angle bisector vectors
    ax.arrow(data.xcx, data.xcy, data.afbx, data.afby, head_width=10, head_length=20, fc='k', ec='k', zorder=10, alpha=0.5)
    ax.arrow(data.xcx, data.xcy, data.alrx, data.alry, head_width=10, head_length=20, fc='k', ec='k', zorder=10, alpha=0.5)

    draw_body(ax, data.cogx, data.cogy)

    ax.grid(True)
    ax.set_title("Walk state @ t=%s" % data.t)
    plt.axis((-300,300,-300,300))
    plt.xlabel("x-axis - back/front")
    plt.ylabel("y-label - left/right")
    #plt.show()
    plt.savefig("../plots/ripple_geo/RIPPLE_GEO_walk_cycle_4_%03d.png" % i)


def draw_body(ax, cogx, cogy):
    verts = [
        (100, 100),
        (100, -100),
        (-100, -100),
        (-100, 100),
        (100, 100),
    ]
    verts = [(x+cogx, y+cogy) for (x, y) in verts]
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='blue', lw=2, zorder=1, alpha=0.1)
    ax.add_patch(patch)

with open("cog_cycle4.log", "r") as f:
    i = 0
    for line in f.readlines():
        data = get_data_from_line(line)
        draw_data(data, i)
        i += 1

#draw_data(data)
