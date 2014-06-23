#!/bin/sh
make clean
make
./yukari data/kato3.csv data/yoshino1.csv data/yoshino6.csv
python2.7 graph.py

