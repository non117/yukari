# -*- coding: utf-8 -*-
import csv

def diff(l1, l2):
    l = []
    for i in range(1,len(l1)):
        l.append( (float(l2[i])-float(l1[i]), i) )
    return l


def main():
    filename = 'all_similarity.csv'
    reader = csv.reader(open(filename))
    l = []
    for line in reader:
        l.append(line)
    header = l[0]
    for j in range(1,4):
        print(l[j][0], l[j+1][0])
        d1 = sorted(diff(l[j],l[j+1]))
        for i in range(10):
            print("{0:6.3f}".format(d1[i][0]),',', header[d1[i][1]])
        print()


if __name__ == '__main__':
    main()

