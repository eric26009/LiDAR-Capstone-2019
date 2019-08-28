class Point:
    """
    point class for Probability map. tallies number of times hit
    and the probability that the point is valid
    """
    def __init__(self):
        self.hits = 0
        self.probability = 0
    
    def addProbability(self, prob):
        # cap at 1
        summedProb = self.probability + prob
        self.probability = summedProb if summedProb < 1 else 1 


class ProbabilityMap:
    """
    creates a point cloud based on probabilities. gives a more accurate representation of a room.

    first add point hits, then call create map.
    """
    def __init__(self,initialResidualProbability, residualDissapationRate, hitProbability, threshold, collapseSize):
        # params
        self.initialResidualProbability = initialResidualProbability
        self.residualDissapationRate = residualDissapationRate
        self.hitProbability = hitProbability
        self.threshold = threshold
        self.collapseSize = collapseSize
        # instance vars
        self.points = {}
        self.visited = set()
        self.residualCutoff = .1
    
    def addPointHit(self,x,y,z):
        p = (int(x/self.collapseSize),int(y/self.collapseSize),int(z/self.collapseSize))
        # add to dict if not already
        if(p not in self.points):
            self.points[p] = Point()
        # update number of hits
        self.points[p].hits += 1
    
    def createMap(self, progressCallback = None):
        """
        processes map and returns an array of [x,y,z]
        """
        print("starting the stitching...")
        # process probabilies
        keys = self.points.keys()[:]
        count = 0
        for key in keys:
            if(progressCallback != None and count % 1000 == 0 ):
                    progressCallback(float(count) /len(keys))
            for i in range(0,self.points[key].hits):
                self.points[key].addProbability(self.hitProbability)
                self.calculateResidualProbabilities(key)
            count += 1
                
        
        # return valid points
        trimmedPointCloud = []
        for (key,value) in self.points.iteritems():
            if(value.probability >= self.threshold):
                trimmedPointCloud.append([key[0]*self.collapseSize,key[1]*self.collapseSize,key[2]*self.collapseSize])
        return trimmedPointCloud
    
    def calculateResidualProbabilities(self, initialPointKey):
        self.visited.clear()
        self.calculateResidualProbabilitiesHelper(initialPointKey,self.initialResidualProbability)
        return 

    def calculateResidualProbabilitiesHelper(self, pointKey, prob):
        # check if already visited
        if(pointKey in self.visited):
            return 

        # check if finished
        if(prob < self.residualCutoff):
            return        
        
        # update probability
        if(pointKey not in self.points):
            self.points[pointKey] = Point()
        self.points[pointKey].addProbability(prob)
        
        # mark visited
        self.visited.add(pointKey)

        # recursively call on adjacent neighbors 
        iterateBy = 1
        x = pointKey[0]
        y = pointKey[1]
        z = pointKey[2]
        right =  (x+iterateBy, y  ,z  )
        left =   (x-iterateBy, y  ,z  )
        above =  (x  , y+iterateBy,z  )
        below =  (x  , y-iterateBy,z  )
        forward =(x  , y  ,z+iterateBy)
        behind = (x  , y  ,z-iterateBy)
        self.calculateResidualProbabilitiesHelper(above, prob / self.residualDissapationRate)
        self.calculateResidualProbabilitiesHelper(below, prob / self.residualDissapationRate)
        self.calculateResidualProbabilitiesHelper(right, prob / self.residualDissapationRate)
        self.calculateResidualProbabilitiesHelper(left, prob / self.residualDissapationRate)
        self.calculateResidualProbabilitiesHelper(forward, prob / self.residualDissapationRate)
        self.calculateResidualProbabilitiesHelper(behind, prob / self.residualDissapationRate)