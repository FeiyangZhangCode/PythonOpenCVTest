import numpy as np

testlist = np.zeros((1, 3), dtype=float)

testlist[0][0] = 0.1
testlist[0][1] = 0.2
testlist[0][2] = 0.3
# print(len(testlist))
testlist = np.append(testlist, [[1.1, 1.2, 1.3]], axis=0)
testlist = np.append(testlist, [[2.1, 2.2, 2.3]], axis=0)



for datalist in testlist:
    datalist[2] = 3.0
    # print(datalist)
    # print(datalist[0], datalist[1], datalist[2])
    # for d1, d2, d3 in datalist:
    #     print(d1, d2, d3)
print(testlist)