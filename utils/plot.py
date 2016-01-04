import matplotlib.pyplot as plt
import numpy as np

import matplotlib.cm as cm

import csv
import sys

class loadTopicData(object):

    def loadCsvFile(self,filename):
        print("read file ",filename)
        data_iter = csv.reader(open(filename), delimiter=",")
        data = [row for row in data_iter]
        return data

    def loadFiles(self,files):
        dataset = []
        for name in files:
            dataset.append(self.loadCsvFile(name))

        return dataset

    def parseRow(self,row):
        points = []
        #offset, how many columns to ignore
        offset = 3
        for i in range(1,24,2):
            #print("row is",row)
            x = float(row[offset+i])
            y = float(row[offset + i + 1])
            #print(x,y)
            points.append((x,y))

        return points 

def initFig():
    #clear figure
    plt.clf()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis((0,128,0,128))

def plotPattern(data,index):
    loader = loadTopicData()
    points = loader.parseRow(data[index])
    colors = cm.rainbow(np.linspace(0, 1, 20))
    for p in points:
        plt.scatter(p[0],128.0-p[1],color=colors[index % 20], alpha=0.5)


def main():
    loader = loadTopicData()

    files = sys.argv[1:]
    dataset = loader.loadFiles(files)

    plt.figure('test')


    #ignore header row
    first = True 
    line = 1
    for row in dataset[0]:
        if first is True:
            first = False
            continue 

        initFig()
        for i in range(0,len(dataset)):
            print("dataset is",i)
            plotPattern(dataset[i],line)


        #plot file2

        line += 1
        plt.ion()
        plt.show()
        raw_input("Press Enter to continue...")



    plt.show()

if __name__ == '__main__':
    main()
