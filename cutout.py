# -*- coding: utf-8 -*-
import csv
import sys

'''
YANO SYSTEMの骨格データを指定された秒数の範囲に切り取るスクリプト。
Python 3.xの環境があれば動きます。
コマンドライン引数として、変換したいcsvファイルを取ります。
数値以外を入力した時の動作は未定義です。
'''

def main():
    print('begin second')
    a = float(input())
    print('end second')
    b = float(input())
    filename = sys.argv[1]
    data = csv.reader(open(filename))
    corrected_filename = filename.split(',')[0] + '_cut.csv'
    writer = csv.writer(open(corrected_filename, 'w'))
    for i, row in enumerate(data):
        if(i==0):
            writer.writerow(row[:46])
            continue
        sec = row[0]
        if(a <= float(sec) <= b):
            writer.writerow(row[:46])

if __name__ == '__main__':
    main()

