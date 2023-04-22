using System;
using System.Collections.Generic;
using System.Linq;

public class Area
{
    private int label;
    private int[] topLeftPoint;
    private int[] bottomRightPoint;
    private List<Vertex> pickUpEntrances = new List<Vertex>();
    private List<Vertex> pickUpExits = new List<Vertex>();
    private List<Vertex> dropOffEntrances = new List<Vertex>();
    private List<Vertex> dropOffExits = new List<Vertex>();
    public Area(int label)
    {
        this.label = label;
    }
    public Area(int label, int[] topLeftPoint, int[] bottomRightPoint, Server server)
    {
        this.label = label;
        this.topLeftPoint = topLeftPoint;
        this.bottomRightPoint = bottomRightPoint;
    }
    public int getLabel()
    {
        return label;
    }
    public int[] getTopLeftPoint()
    {
        return topLeftPoint;
    }
    public int[] getBottomRightPoint()
    {
        return bottomRightPoint;
    }
    public void addPickUpEntrances(Vertex pickUpEntrance)
    {
        pickUpEntrances.Add(pickUpEntrance);
    }
    public List<Vertex> getPickUpEntrances()
    {
        return pickUpEntrances;
    }
    public void addPickUpExits(Vertex pickUpExit)
    {
        pickUpExits.Add(pickUpExit);
    }
    public List<Vertex> getPickUpExits()
    {
        return pickUpExits;
    }
    public void addDropOffEntrances(Vertex dropOffEntrance)
    {
        dropOffEntrances.Add(dropOffEntrance);
    }
    public List<Vertex> getDropOffEntrances()
    {
        return dropOffEntrances;
    }
    public void addDropOffExits(Vertex dropOffExit)
    {
        dropOffExits.Add(dropOffExit);
    }
    public List<Vertex> getDropOffExits()
    {
        return dropOffExits;
    }
}

public class Vertex
{
    private int[] label;
    private int type;
    private Area area = new Area(0);
    private int dynamicCost = 0;
    private bool isRingRoad = false;
    private bool isOuterRingRoad = false;
    private bool isCurveStart = false;
    private bool isCurveCenter = false;
    private bool isCurveEnd = false;
    private bool isCurveGrid = false;
    private bool isEntrance = false;
    private bool isExit = false;
    private bool isOuterRingRoadCorner = false;
    private Curve curve;
    private int curveGridLabel = 1000;
    private bool isCorner = false;
    private Curve cornerCurve;
    private double[] curveGridPosition;
    private double curveAngle;
    private Vertex extraGridOccupied;

    public Vertex(int[] label, int type)
    {
        this.label = label;
        this.type = type;
    }
    public int[] getLabel()
    {
        return label;
    }
    public int getType()
    {
        return type;
    }
    public void setArea(Area area)
    {
        this.area = area;
    }
    public Area getArea()
    {
        return area;
    }
    public void setIsRingRoad(bool isRingRoad)
    {
        this.isRingRoad = isRingRoad;
    }
    public bool getIsRingRoad()
    {
        return isRingRoad;
    }
    public void setIsEntrance(bool isEntrance)
    {
        this.isEntrance = isEntrance;
    }
    public bool getIsEntrance()
    {
        return isEntrance;
    }
    public void setIsExit(bool isExit)
    {
        this.isExit = isExit;
    }
    public bool getIsExit()
    {
        return isExit;
    }
    public void addDynamicCost(int cost)
    {
        dynamicCost += cost;
    }
    public int getDynamicCost()
    {
        return dynamicCost;
    }
    public void setIsCurveStart(bool isCurveStart)
    {
        this.isCurveStart = isCurveStart;
    }
    public bool getIsCurveStart()
    {
        return isCurveStart;
    }
    public void setIsCurveEnd(bool isCurveEnd)
    {
        this.isCurveEnd = isCurveEnd;
    }
    public bool getIsCurveEnd()
    {
        return isCurveEnd;
    }
    public void setIsCurveGrid(bool isCurveGrid)
    {
        this.isCurveGrid = isCurveGrid;
    }
    public bool getIsCurveGrid()
    {
        return isCurveGrid;
    }
    public void setCurve(Curve curve)
    {
        this.curve = curve;
    }
    public Curve getCurve()
    {
        return curve;
    }
    public void setCurveGridLabel(int curveGridLabel)
    {
        this.curveGridLabel = curveGridLabel;
    }
    public int getCurveGridLabel()
    {
        return curveGridLabel;
    }
    public void setCurveGridPosition(double[] curveGridPosition)
    {
        this.curveGridPosition = curveGridPosition;
    }
    public double[] getCurveGridPosition()
    {
        return curveGridPosition;
    }
    public void setCurveAngle(double curveAngle)
    {
        this.curveAngle = curveAngle;
    }
    public double getCurveAngle()
    {
        return curveAngle;
    }
    public void setIsCorner(bool isCorner)
    {
        this.isCorner = isCorner;
    }
    public bool getIsCorner()
    {
        return isCorner;
    }
    public void setIsCurveCenter(bool isCurveCenter)
    {
        this.isCurveCenter = isCurveCenter;
    }
    public bool getIsCurveCenter()
    {
        return isCurveCenter;
    }
    public void setIsOuterRingRoad(bool isOuterRingRoad)
    {
        this.isOuterRingRoad = isOuterRingRoad;
    }
    public bool getIsOuterRingRoad()
    {
        return isOuterRingRoad;
    }
    public void setIsOuterRingRoadCorner(bool isOuterRingRoadCorner)
    {
        this.isOuterRingRoadCorner = isOuterRingRoadCorner;
    }
    public bool getIsOuterRingRoadCorner()
    {
        return isOuterRingRoadCorner;
    }
    public void setCornerCurve(Curve cornerCurve)
    {
        this.cornerCurve = cornerCurve;
    }
    public Curve getCornerCurve()
    {
        return cornerCurve;
    }
    public void setExtraGridOccupied(Vertex extraGridOccupied)
    {
        this.extraGridOccupied = extraGridOccupied;
    }
    public Vertex getExtraGridOccupied()
    {
        return extraGridOccupied;
    }
}

public class Edge
{
    private Vertex nextVertex;
    private int cost;
    private int direction;
    public Edge(Vertex nextVertex, int cost, int direction)
    {
        this.nextVertex = nextVertex;
        this.cost = cost;
        this.direction = direction;
    }
    public Vertex getNextVertex()
    {
        return nextVertex;
    }
    public int getCost()
    {
        return cost + nextVertex.getDynamicCost();
    }
    public int getDirection()
    {
        return direction;
    }
}

public class Curve
{
    private int label;
    private Vertex vertexStart;
    private Vertex vertexEnd;
    private double[] center;
    private double startAngle;
    private double endAngle;
    private double radius;
    private List<Vertex> grids = new List<Vertex> { };

    public Curve(int label, Vertex vertexStart, Vertex vertexEnd, double[] center, double startAngle, double endAngle, double radius)
    {
        this.label = label;
        this.vertexStart = vertexStart;
        this.vertexEnd = vertexEnd;
        this.center = center;
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.radius = radius;
    }
    public void setLabel(int label)
    {
        this.label = label;
    }
    public int getLabel()
    {
        return label;
    }
    public Vertex getVertexStart()
    {
        return vertexStart;
    }
    public Vertex getVertexEnd()
    {
        return vertexEnd;
    }
    public double[] getCenter()
    {
        return center;
    }
    public double getStartAngle()
    {
        return startAngle;
    }
    public double getEndAngle()
    {
        return endAngle;
    }
    public double getRadius()
    {
        return radius;
    }
    public void addCurveGrids(Vertex curveGrid)
    {
        grids.Add(curveGrid);
    }
    public List<Vertex> getCurveGrids()
    {
        return grids;
    }
}

public class MoveGrid : Vertex
{
    private List<Edge> edges = new List<Edge>();
    public MoveGrid(int[] label, int type) : base(label, type)
    {
    }
    public void addEdge(Vertex nextVertex, int cost, int direction)
    {
        Edge edge = new Edge(nextVertex, cost, direction);
        edges.Add(edge);
    }
    public List<Edge> getEdge()
    {
        List<Edge> newEdge = new List<Edge>();
        for (int i = 0; i < edges.Count; i++)
        {
            newEdge.Add(edges[i]);
        }
        return newEdge;
    }
    public void clearEdge()
    {
        edges = new List<Edge>();
    }
}

public class PickUpPoint : Vertex
{
    private List<Vertex> pickUpGrids = new List<Vertex>();
    public PickUpPoint(int[] label, int type) : base(label, type)
    {
    }
    public void addPickUpGrids(Vertex vertex)
    {
        this.pickUpGrids.Add(vertex);
    }
    public List<Vertex> getPickUpGrids()
    {
        return pickUpGrids;
    }
}

public class DropOffPoint : Vertex
{
    private List<Vertex> dropOffPoint = new List<Vertex>();
    public DropOffPoint(int[] label, int type) : base(label, type)
    {
    }
    public void addDropOffGrids(Vertex vertex)
    {
        this.dropOffPoint.Add(vertex);
    }
    public List<Vertex> getDropOffGrids()
    {
        return dropOffPoint;
    }
}

public class ChargingPoint : Vertex
{
    private List<Edge> edges = new List<Edge>();
    public ChargingPoint(int[] label, int type) : base(label, type)
    {
    }
    public void addEdge(Vertex nextVertex, int cost, int direction)
    {
        Edge edge = new Edge(nextVertex, cost, direction);
        edges.Add(edge);
    }
    public List<Edge> getEdge()
    {
        List<Edge> newEdge = new List<Edge>();
        for (int i = 0; i < edges.Count; i++)
        {
            newEdge.Add(edges[i]);
        }
        return newEdge;
    }
}

public class Graph
{
    private int[,] graph;
    private int[,] ringRoad;
    private int[] shape;
    private List<PickUpPoint> pickUpPoints = new List<PickUpPoint>();
    private List<DropOffPoint> dropOffPoints = new List<DropOffPoint>();
    private List<ChargingPoint> chargingPoints = new List<ChargingPoint>();
    private List<List<Vertex>> vertices = new List<List<Vertex>>();
    private List<Area> areas = new List<Area>();
    private List<Curve> curveRoads = new List<Curve>();

    public Graph(int[,] graph, int[,] ringRoad)
    {
        this.graph = graph;
        this.ringRoad = ringRoad;
        shape = new int[] { graph.GetUpperBound(0), graph.GetUpperBound(1) };

        // vertices labels
        // 0 = intersection
        // 1 = left
        // 2 = right
        // 3 = up
        // 4 = down
        // 6 = pickup points
        // 7 = dropoff points
        // 8 = charging points
        // 9 = not used

        // ring road labels  
        // 0 = not ring road
        // 1 = ring road
        // 2 = ring road entrance
        // 3 = ring road exit

        // Create Vertices
        for (int i = 0; i <= shape[0]; i++)
        {
            List<Vertex> tempVertices = new List<Vertex>();
            for (int j = 0; j <= shape[1]; j++)
            {
                int currentType = graph[i, j];
                int ringRoadType = ringRoad[i, j];
                Vertex v;

                // create vertices
                if (Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == currentType))
                {
                    v = new MoveGrid(new int[] { i, j }, graph[i, j]);
                }
                else if (currentType == 6)
                {
                    v = new PickUpPoint(new int[] { i, j }, graph[i, j]);
                    pickUpPoints.Add(v as PickUpPoint);
                }
                else if (currentType == 7)
                {
                    v = new DropOffPoint(new int[] { i, j }, graph[i, j]);
                    dropOffPoints.Add(v as DropOffPoint);
                }
                else if (currentType == 8)
                {
                    v = new ChargingPoint(new int[] { i, j }, graph[i, j]);
                    chargingPoints.Add(v as ChargingPoint);
                }
                else
                {
                    v = new Vertex(new int[] { i, j }, graph[i, j]);
                }

                // initialize ring road
                if (ringRoadType == 1 || ringRoadType == 2 || ringRoadType == 3)
                {
                    v.setIsRingRoad(true);
                }
                else if (ringRoadType == 4)
                {
                    v.setIsOuterRingRoad(true);
                }
                tempVertices.Add(v);
            }
            vertices.Add(tempVertices);
        }

        // Add Edges
        for (int i = 0; i <= shape[0]; i++)
        {
            for (int j = 0; j <= shape[1]; j++)
            {
                Vertex currentVertex = vertices[i][j];
                int currentType = currentVertex.getType();

                // Intersection
                if (currentType == 0)
                {
                    if (j != 0 &&
                    (Array.Exists(new int[] { 0, 1, 8 }, element => element == graph[i, j - 1]) ||
                    ringRoad[i, j - 1] != 0) &&
                    (j == shape[1] || graph[i, j + 1] != 2))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i][j - 1], 3, 1);
                    }

                    if (j != shape[1] &&
                    (Array.Exists(new int[] { 0, 2, 8 }, element => element == graph[i, j + 1]) ||
                    ringRoad[i, j + 1] != 0) &&
                    (j == 0 || graph[i, j - 1] != 1))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i][j + 1], 3, 2);
                    }

                    if (i != 0 &&
                    (Array.Exists(new int[] { 0, 3, 8 }, element => element == graph[i - 1, j]) ||
                    ringRoad[i - 1, j] != 0) &&
                    (i == shape[0] || graph[i + 1, j] != 4))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i - 1][j], 3, 3);
                    }

                    if (i != shape[0] &&
                    (Array.Exists(new int[] { 0, 4, 8 }, element => element == graph[i + 1, j]) ||
                    ringRoad[i + 1, j] != 0) &&
                    (i == 0 || graph[i - 1, j] != 3))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i + 1][j], 3, 4);
                    }

                }

                // Other grids to move
                else if (Array.Exists(new int[] { 1, 2, 3, 4 }, element => element == currentType))
                {
                    if (j != 0 && currentType != 2 &&
                    Array.Exists(new int[] { 0, 8, 1 }, element => element == graph[i, j - 1]))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i][j - 1], 3, 1);
                    }

                    if (j != shape[1] && currentType != 1 &&
                    Array.Exists(new int[] { 0, 8, 2 }, element => element == graph[i, j + 1]))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i][j + 1], 3, 2);
                    }

                    if (i != 0 && currentType != 4 &&
                    Array.Exists(new int[] { 0, 8, 3 }, element => element == graph[i - 1, j]))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i - 1][j], 3, 3);
                    }

                    if (i != shape[0] && currentType != 3 &&
                    Array.Exists(new int[] { 0, 8, 4 }, element => element == graph[i + 1, j]))
                    {
                        (currentVertex as MoveGrid).addEdge(vertices[i + 1][j], 3, 4);
                    }

                }

                // Pick up points
                else if (currentType == 6)
                {
                    if (i != 0 &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i - 1, j]))
                    {
                        (currentVertex as PickUpPoint).addPickUpGrids(vertices[i - 1][j]);
                    }
                    if (i != shape[0] &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i + 1, j]))
                    {
                        (currentVertex as PickUpPoint).addPickUpGrids(vertices[i + 1][j]);
                    }
                    if (j != 0 &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i, j - 1]))
                    {
                        (currentVertex as PickUpPoint).addPickUpGrids(vertices[i][j - 1]);
                    }
                    if (j != shape[1] &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i, j + 1]))
                    {
                        (currentVertex as PickUpPoint).addPickUpGrids(vertices[i][j + 1]);
                    }
                }

                // Drop off points
                else if (currentType == 7)
                {
                    if (i != 0 &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i - 1, j]))
                    {
                        (currentVertex as DropOffPoint).addDropOffGrids(vertices[i - 1][j]);
                    }
                    if (i != shape[0] &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i + 1, j]))
                    {
                        (currentVertex as DropOffPoint).addDropOffGrids(vertices[i + 1][j]);
                    }
                    if (j != 0 &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i, j - 1]))
                    {
                        (currentVertex as DropOffPoint).addDropOffGrids(vertices[i][j - 1]);
                    }
                    if (j != shape[1] &&
                    Array.Exists(new int[] { 0, 1, 2, 3, 4 }, element => element == graph[i, j + 1]))
                    {
                        (currentVertex as DropOffPoint).addDropOffGrids(vertices[i][j + 1]);
                    }
                }

                // Charging points
                else if (currentType == 8)
                {
                    if (i != 0 &&
                    Array.Exists(new int[] { 0, 1, 2, 3 }, element => element == graph[i - 1, j]) &&
                    (i == shape[0] || graph[i + 1, j] != 4))
                    {
                        (currentVertex as ChargingPoint).addEdge(vertices[i - 1][j], 3, 3);
                    }

                    if (i != shape[0] &&
                    Array.Exists(new int[] { 0, 1, 2, 4 }, element => element == graph[i + 1, j]) &&
                    (i == 0 || graph[i - 1, j] != 3))
                    {
                        (currentVertex as ChargingPoint).addEdge(vertices[i + 1][j], 3, 4);
                    }

                    if (j != 0 &&
                    Array.Exists(new int[] { 0, 1, 3, 4 }, element => element == graph[i, j - 1]) &&
                    (j == shape[1] || graph[i, j + 1] != 2))
                    {
                        (currentVertex as ChargingPoint).addEdge(vertices[i][j - 1], 3, 1);
                    }

                    if (j != shape[1] &&
                    Array.Exists(new int[] { 0, 2, 3, 4 }, element => element == graph[i, j + 1]) &&
                    (j == 0 || graph[i, j - 1] != 1))
                    {
                        (currentVertex as ChargingPoint).addEdge(vertices[i][j + 1], 3, 2);
                    }
                }
            }
        }
    }

    public void addRingRoadExitEdges(Vertex currentVertex, Vertex nextVertex, int direction, int edgeType)
    {
        // edge type:
        // 1: exit
        // 2: entrance
        if (edgeType == 1)
        {
            if (!(nextVertex is MoveGrid) || nextVertex.getIsEntrance())
            {
                return;
            }
            List<Edge> existingEdges = (currentVertex as MoveGrid).getEdge();
            for (int i = 0; i < existingEdges.Count; i++)
            {
                if (existingEdges[i].getNextVertex() == nextVertex)
                {
                    return;
                }
            }
            (currentVertex as MoveGrid).addEdge(nextVertex, 3, direction);
        }
        else
        {
            if (!(currentVertex is MoveGrid) || currentVertex.getIsExit())
            {
                return;
            }
            List<Edge> existingEdges = (currentVertex as MoveGrid).getEdge();
            for (int i = 0; i < existingEdges.Count; i++)
            {
                if (existingEdges[i].getNextVertex() == nextVertex)
                {
                    return;
                }
            }
            (currentVertex as MoveGrid).addEdge(nextVertex, 3, direction);
        }
    }

    public int[,] getGraph()
    {
        return graph;
    }

    public int[] getShape()
    {
        return shape;
    }

    public List<List<Vertex>> getVertices()
    {
        return vertices;
    }

    public List<Curve> getCurveRoads()
    {
        return curveRoads;
    }

    public List<PickUpPoint> getPickUpPoints()
    {
        return pickUpPoints;
    }

    public List<DropOffPoint> getDropOffPoints()
    {
        return dropOffPoints;
    }

    public List<ChargingPoint> getChargingPoints()
    {
        return chargingPoints;
    }

    public void addCurveRoad(Curve curve)
    {
        curveRoads.Add(curve);
    }

    public Curve getCurve(int label)
    {
        return curveRoads[label];
    }

    public int[,] getRingRoad()
    {
        return ringRoad;
    }

    // myList.Capacity = 1000;
    public List<Vertex> findRouteFromStartToEntrance(Vertex vertexStart, Vertex destination, int changeDirectionPenalize = 10)
    {
        HashSet<Vertex> visited = new HashSet<Vertex>(1000);
        List<Edge> toVisit = new List<Edge>(1000);
        List<int> visitCosts = new List<int>(1000);
        List<Tuple<List<Vertex>, int>> visitPaths = new List<Tuple<List<Vertex>, int>>(1000);

        List<Edge> firstToVisit;
        if (vertexStart is MoveGrid)
        {
            firstToVisit = (vertexStart as MoveGrid).getEdge();
        }
        else
        {
            firstToVisit = (vertexStart as ChargingPoint).getEdge();
        }
        toVisit = firstToVisit;

        for (int i = 0; i < toVisit.Count; i++)
        {
            Edge currentToVisit = toVisit[i];
            visitCosts.Add(currentToVisit.getCost());
            List<Vertex> newPath = new List<Vertex>(1000);
            newPath.Add(currentToVisit.getNextVertex());
            visitPaths.Add(Tuple.Create(newPath, currentToVisit.getDirection()));
        }

        while (toVisit.Count != 0)
        {
            int shortest = visitCosts.IndexOf(visitCosts.Min());
            Edge nextToVisit = toVisit[shortest];
            List<Vertex> curRoute = visitPaths[shortest].Item1;
            int lastDirection = visitPaths[shortest].Item2;
            int curCost = visitCosts[shortest];

            toVisit.RemoveAt(shortest);
            visitCosts.RemoveAt(shortest);
            visitPaths.RemoveAt(shortest);

            Vertex nextVertexToVisit = nextToVisit.getNextVertex();
            if (visited.Contains(nextVertexToVisit))
            {
                continue;
            }
            visited.Add(nextVertexToVisit);

            if (nextVertexToVisit == destination)
            {
                curRoute.AddRange(nextVertexToVisit.getCurve().getCurveGrids());
                return curRoute;
            }

            // else if(curRoute.Count > 2 && curRoute[curRoute.Count - 2].getIsOuterRingRoad()){
            //     continue;
            // }

            List<Edge> newEdges;
            if (nextVertexToVisit is MoveGrid)
            {
                newEdges = (nextVertexToVisit as MoveGrid).getEdge();
            }
            else
            {
                continue;
            }

            for (int j = 0; j < newEdges.Count; j++)
            {
                Edge currentEdge = newEdges[j];
                if (!visited.Contains(currentEdge.getNextVertex()))
                {
                    toVisit.Add(currentEdge);
                    int newCost = curCost + currentEdge.getCost();
                    if (lastDirection != currentEdge.getDirection() || currentEdge.getNextVertex().getIsOuterRingRoad())
                    {
                        newCost += changeDirectionPenalize;
                    }
                    visitCosts.Add(newCost);

                    List<Vertex> newRoute = new List<Vertex>(1000);
                    for (int k = 0; k < curRoute.Count; k++)
                    {
                        newRoute.Add(curRoute[k]);
                    }
                    newRoute.Add(currentEdge.getNextVertex());
                    visitPaths.Add(Tuple.Create(newRoute, currentEdge.getDirection()));
                }
            }
        }
        return new List<Vertex>();
    }

    public List<Vertex> findRouteInRingRoad(Vertex vertexStart, Vertex destination)
    {
        List<Edge> toVisit = new List<Edge>(1000);
        List<int> visitCosts = new List<int>(1000);
        List<Tuple<List<Vertex>, int>> visitPaths = new List<Tuple<List<Vertex>, int>>(1000);
        Vertex exitVertex = destination.getCurve().getVertexStart();

        List<Vertex> tempStartRoute = new List<Vertex>(1000);
        if (vertexStart.getIsCorner() && vertexStart.getIsCurveStart())
        {
            tempStartRoute = vertexStart.getCornerCurve().getCurveGrids();
            vertexStart = vertexStart.getCornerCurve().getVertexEnd();
        }

        List<Edge> firstToVisit;
        firstToVisit = (vertexStart as MoveGrid).getEdge();
        toVisit = firstToVisit;

        for (int i = 0; i < toVisit.Count; i++)
        {
            Edge currentToVisit = toVisit[i];
            visitCosts.Add(currentToVisit.getCost());
            List<Vertex> newPath = new List<Vertex>(1000);
            newPath.AddRange(tempStartRoute);
            newPath.Add(currentToVisit.getNextVertex());
            visitPaths.Add(Tuple.Create(newPath, currentToVisit.getDirection()));
        }

        while (toVisit.Count != 0)
        {
            int shortest = visitCosts.IndexOf(visitCosts.Min());
            Edge nextToVisit = toVisit[shortest];
            List<Vertex> curRoute = visitPaths[shortest].Item1;
            int curCost = visitCosts[shortest];

            toVisit.RemoveAt(shortest);
            visitCosts.RemoveAt(shortest);
            visitPaths.RemoveAt(shortest);

            Vertex nextVertexToVisit = nextToVisit.getNextVertex();

            if (nextVertexToVisit == exitVertex)
            {
                curRoute.AddRange(nextVertexToVisit.getCurve().getCurveGrids());
                return curRoute;
            }

            else if (nextVertexToVisit.getIsCurveStart() && nextVertexToVisit.getIsCorner())
            {
                curRoute.AddRange(nextVertexToVisit.getCornerCurve().getCurveGrids());
                nextVertexToVisit = nextVertexToVisit.getCornerCurve().getVertexEnd();
                if (nextVertexToVisit == exitVertex)
                {
                    curRoute.AddRange(nextVertexToVisit.getCurve().getCurveGrids());
                    return curRoute;
                }
            }

            List<Edge> newEdges;
            newEdges = (nextVertexToVisit as MoveGrid).getEdge();

            for (int j = 0; j < newEdges.Count; j++)
            {
                Edge currentEdge = newEdges[j];
                if (currentEdge.getNextVertex().getIsRingRoad())
                {
                    toVisit.Add(currentEdge);
                    int newCost = curCost + currentEdge.getCost();
                    visitCosts.Add(newCost);

                    List<Vertex> newRoute = new List<Vertex>(1000);
                    for (int k = 0; k < curRoute.Count; k++)
                    {
                        newRoute.Add(curRoute[k]);
                    }
                    newRoute.Add(currentEdge.getNextVertex());
                    visitPaths.Add(Tuple.Create(newRoute, currentEdge.getDirection()));
                }
            }
        }
        return new List<Vertex>();
    }

    public List<Vertex> findRouteWithOneDestination(Vertex vertexStart, List<Vertex> destination, int changeDirectionPenalize = 10)
    {
        HashSet<Vertex> visited = new HashSet<Vertex>(1000);
        List<Edge> toVisit = new List<Edge>(1000);
        List<int> visitCosts = new List<int>(1000);
        List<Tuple<List<Vertex>, int>> visitPaths = new List<Tuple<List<Vertex>, int>>(1000);

        List<Edge> firstToVisit;
        if (vertexStart is MoveGrid)
        {
            firstToVisit = (vertexStart as MoveGrid).getEdge();
        }
        else
        {
            firstToVisit = (vertexStart as ChargingPoint).getEdge();
        }
        toVisit = firstToVisit;

        for (int i = 0; i < toVisit.Count; i++)
        {
            Edge currentToVisit = toVisit[i];
            visitCosts.Add(currentToVisit.getCost());
            List<Vertex> newPath = new List<Vertex>(1000);
            newPath.Add(currentToVisit.getNextVertex());
            visitPaths.Add(Tuple.Create(newPath, currentToVisit.getDirection()));
        }

        while (toVisit.Count != 0)
        {
            int shortest = visitCosts.IndexOf(visitCosts.Min());
            Edge nextToVisit = toVisit[shortest];
            List<Vertex> curRoute = visitPaths[shortest].Item1;
            int lastDirection = visitPaths[shortest].Item2;
            int curCost = visitCosts[shortest];

            toVisit.RemoveAt(shortest);
            visitCosts.RemoveAt(shortest);
            visitPaths.RemoveAt(shortest);

            Vertex nextVertexToVisit = nextToVisit.getNextVertex();
            if (visited.Contains(nextVertexToVisit))
            {
                continue;
            }
            visited.Add(nextVertexToVisit);

            if (destination.Contains(nextVertexToVisit))
            {
                return curRoute;
            }
            // else if(curRoute.Count == 1){
            //     if(visited.Last().getIsOuterRingRoad()){
            //         List<Edge> edges = (curRoute[0] as MoveGrid).getEdge();
            //         for(int i = 0; i < edges.Count; i++){
            //             if(!edges[i].getNextVertex().getIsOuterRingRoad()){
            //                 continue;
            //             }
            //         }
            //     }
            // }
            // else if (visited.Last().getIsOuterRingRoad()){
            //     continue;
            // }

            List<Edge> newEdges;
            if (nextVertexToVisit is MoveGrid)
            {
                newEdges = (nextVertexToVisit as MoveGrid).getEdge();
            }
            else
            {
                continue;
            }

            for (int j = 0; j < newEdges.Count; j++)
            {
                Edge currentEdge = newEdges[j];
                if (!visited.Contains(currentEdge.getNextVertex()) && !currentEdge.getNextVertex().getIsRingRoad())
                {
                    toVisit.Add(currentEdge);
                    int newCost = curCost + currentEdge.getCost();
                    if (lastDirection != currentEdge.getDirection() || currentEdge.getNextVertex().getIsOuterRingRoad())
                    {
                        newCost += changeDirectionPenalize;
                    }
                    visitCosts.Add(newCost);

                    List<Vertex> newRoute = new List<Vertex>(1000);
                    for (int k = 0; k < curRoute.Count; k++)
                    {
                        newRoute.Add(curRoute[k]);
                    }
                    newRoute.Add(currentEdge.getNextVertex());
                    visitPaths.Add(Tuple.Create(newRoute, currentEdge.getDirection()));
                }
            }
        }
        return new List<Vertex>();
    }
}