import json
Keys = ['FileName', 'PodLocation', 'PodState', 'PodID']

def LoadJson(InFile):
    File = open(InFile,'r')
    PodsDict = json.loads(File.read())
    # with open('PodLoc.json','r') as File:
    #     for line in File:
    #         line = json.loads(line.strip())
    #         print(line)
    return PodsDict

def GetPodLocAndWaypointsFileName(InFile, InPodID):
    PodsDict = LoadJson(InFile)
    PodInfo = PodsDict[InPodID]
    File = PodInfo[Keys[0]][:]
    return PodInfo[Keys[1]], File

if __name__ == "__main__":
    # PodDict = {'PodID': 1, 'PodLocation': [2.0, 3.0, 40], 'PodState': 'ForPickup'}
    # PodDict2 = {'PodID': 2, 'PodLocation': [2.0, 3.0, 40], 'PodState': 'Unavailable'}
    # with open('PodLoc.json','a') as sample:
    #     sample.write('{}\n'.format(json.dumps(PodDict)))
    #     sample.write('{}\n'.format(json.dumps(PodDict2)))
    a, b = GetPodLocAndWaypointsFileName('PickupPodLoc.json', '12')
    print(b)
    



    

