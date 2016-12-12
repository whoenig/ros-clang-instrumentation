#!/usr/bin/env python

import argparse
import numpy as np
import yaml
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="configuration file (yaml)")
    parser.add_argument("name", help="project (folder) name")
    args = parser.parse_args()

    # read file
    with open(args.config) as configFile:
        data = yaml.safe_load(configFile)

    topicIdx = 0
    for topic in data["topics"]:
        print("Topic: " + topic["name"])
        numNodes = len(topic["nodes"])
        allNodeData = []
        for nodeIdx in range(0, numNodes):
            fileName = os.path.expanduser("~/.ros/{}_{}.txt".format(topicIdx, nodeIdx))
            nodeData = np.loadtxt(fileName, dtype=np.uint64, delimiter=",", ndmin=2)
            allNodeData.append(nodeData)
            # print(nodeData)
        for i in range(1, numNodes):
            numElements = min(allNodeData[i-1].shape[0], allNodeData[i].shape[0])
        dts = np.empty((numElements, numNodes - 1))
        for i in range(1, numNodes):
            dts[:,i-1] = allNodeData[i][0:numElements,0] - allNodeData[i-1][0:numElements,0]
        print("  Latencies [us]:")
        print("    " + str(dts).replace('\n','\n    '))
        for iteration in range(0, numElements):
            print("  Iteration {}:".format(iteration))
            for i in range(0, numNodes):
                fileName = os.path.join(os.path.abspath("."), "build/" + args.name + "/paths_{}_{}.txt".format(topicIdx, i))
                # fileName = "/home/whoenig/projects/phd/cs610/project/milestone2/ros_ws/build/test3/paths_{}_{}.txt".format(topicIdx, i)
                with open(fileName) as f:
                    lines = f.readlines()
                if allNodeData[i].shape[1] > 1:
                    entry = lines[allNodeData[i][iteration,1]]
                    entries = entry.split(",")[1:]
                    lastEntry = None
                    for e in entries:
                        if e != lastEntry:
                            print("    " + os.path.basename(e))
                        lastEntry = e
                else:
                    print("    " + lines[0])
                if i < numNodes - 1:
                    print("    Latency: {} us".format(dts[iteration, i]))

        topicIdx += 1

