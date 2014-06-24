# -*- coding: utf-8 -*-
import csv
from pathlib import Path

import pylab

workdir = Path('output')

def draw(time, datas, label_t, label_y, title, legend=[], axis=[], filename=''):
    pylab.clf()
    for data in datas:
        pylab.plot(time, data)
    pylab.xlabel(label_t)
    pylab.ylabel(label_y)
    pylab.title(title)
    pylab.grid()
    if legend: pylab.legend(legend, loc=0, frameon=False)
    if axis: pylab.axis(axis)
    filename = (workdir / 'img' / (filename or title)).as_posix() + '.png'
    pylab.savefig(filename)

def load_csv(filepath):
    reader = csv.reader(filepath.open())
    ts, xs, ys, zs = [], [], [], []
    for i, row in enumerate(reader):
        if i==0:
            continue
        ts.append(row[0])
        xs.append(row[1])
        ys.append(row[2])
        zs.append(row[3])
    return ts, xs, ys, zs

def main():
    paths = [x for x in workdir.iterdir() if x.suffix == '.csv']
    #for path in paths:
    #    ts, xs, ys, zs = load_csv(path)
    #    title = path.name
    #    filename = path.name.replace('.csv','')
    #    label_t = 'time (frame)'
    #    label_y = 'coordinate'
    #    legend = ['x', 'y', 'z']
    #    draw(ts, (xs, ys, zs), label_t, label_y, title, legend, filename=filename)
    n = 1
    i = 0
    j = n
    
    while(j <= len(paths)):
        ps = paths[i:j]
        i += n; j+= n
        envelopes = []
        legends = []
        for p in ps:
            ts, xs, ys, zs = load_csv(p)
            envelopes += [xs, ys, zs]
            legends += ['x','y','z']
        title = ps[0].name
        filename = ps[0].name.replace('.csv','')
        label_t = 'time (sec)'
        label_y = 'coordinate (mm)'
        draw(ts, envelopes, label_t, label_y, title, legends, filename=filename) 

if __name__ == '__main__':
    main()

