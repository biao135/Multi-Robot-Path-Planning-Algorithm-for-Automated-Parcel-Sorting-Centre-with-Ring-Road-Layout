using System;
using System.Collections.Generic;
using System.Linq;

public class Server : Graph
{
    // for display information
    private int throughput;

    // map
    private List<List<Vertex>> vertices;
    private List<Area> areas = new List<Area>();
    private int[,] map;
    private Robot[,] mapRobots;
    private int[,] deadlockMap;
    // private List<Robot> robots;
    private List<Tuple<PickUpPoint, DropOffPoint>> tasks;

    // for charging
    List<ChargingPoint> chargingPoints;
    private Robot[] chargingPointsRobots;

    // for curve
    private List<Curve> curveRoads;
    private List<List<int>> curveGridsMap = new List<List<int>>();
    private List<List<int>> curveGridsDeadlockMap = new List<List<int>>();
    private List<List<Robot>> curveGridsMapRobots = new List<List<Robot>>();

    // for ring road
    private int maximumRobotInRingRoad = 0;
    private int robotInRingRoadCount = 0;
    private List<List<int>> ringRoadPickUpEntrances = new List<List<int>>();
    private List<List<int>> ringRoadPickUpExits = new List<List<int>>();
    private List<List<int>> ringRoadDropOffEntrances = new List<List<int>>();
    private List<List<int>> ringRoadDropOffExits = new List<List<int>>();
    private List<List<Vertex>> ringRoadPickUpEntrancesVertex = new List<List<Vertex>>();
    private List<List<Vertex>> ringRoadPickUpExitsVertex = new List<List<Vertex>>();
    private List<List<Vertex>> ringRoadDropOffEntrancesVertex = new List<List<Vertex>>();
    private List<List<Vertex>> ringRoadDropOffExitsVertex = new List<List<Vertex>>();

    public Server(int[,] graph, int[,] ringRoad) : base(graph, ringRoad)
    {
        // initialize map
        int[] mapShape = getShape();
        map = new int[mapShape[0], mapShape[1]];
        mapRobots = new Robot[mapShape[0], mapShape[1]];
        deadlockMap = new int[mapShape[0], mapShape[1]];
        // robots = new List<Robot>();
        chargingPoints = getChargingPoints();
        chargingPointsRobots = new Robot[getChargingPoints().Count];

        vertices = getVertices();

        // always have 5 tasks ready
        tasks = new List<Tuple<PickUpPoint, DropOffPoint>>();
        for (int i = 0; i < 5; i++)
        {
            generateTask();
        }

        // number of robots allowed in ring road
        int[] shape = getShape();
        for (int i = 0; i <= shape[0]; i++)
        {
            for (int j = 0; j <= shape[1]; j++)
            {
                if (ringRoad[i, j] == 1 || ringRoad[i, j] == 2 || ringRoad[i, j] == 3)
                {
                    maximumRobotInRingRoad++;
                }
            }
        }
        //maximumRobotInRingRoad = (int)(maximumRobotInRingRoad * 0.9);
        // maximumRobotInRingRoad = 1000;
    }

    public void addThroughput()
    {
        throughput += 1;
        //throughputCounter.text = throughput.ToString();
    }

    public int getThroughput()
    {
        return throughput;
    }

    public int getAvailableChargingPointCount()
    {
        int count = 0;
        for(int i = 0; i < chargingPointsRobots.Count(); i++)
        {
            if(chargingPointsRobots[i] == null)
            {
                count += 1;
            }
        }
        return count;
    }

    public void displayRobotCount(int robotCount)
    {
        //robotCounter.text = robotCount.ToString();
    }

    public void addArea(Area area)
    {
        int[] topLeftPoint = area.getTopLeftPoint();
        int[] bottomRightPoint = area.getBottomRightPoint();

        // detect which area that the vertices are located at
        for (int i = 0; i < vertices.Count; i++)
        {
            for (int j = 0; j < vertices[i].Count; j++)
            {
                int[] vertexPosition = vertices[i][j].getLabel();
                if (vertexPosition[0] >= bottomRightPoint[0] && vertexPosition[0] <= topLeftPoint[0] &&
                vertexPosition[1] >= topLeftPoint[1] && vertexPosition[1] <= bottomRightPoint[1])
                {
                    vertices[i][j].setArea(area);
                }
            }
        }

        ringRoadPickUpEntrances.Add(new List<int>());
        ringRoadPickUpExits.Add(new List<int>());
        ringRoadDropOffEntrances.Add(new List<int>());
        ringRoadDropOffExits.Add(new List<int>());
        ringRoadPickUpEntrancesVertex.Add(new List<Vertex>());
        ringRoadPickUpExitsVertex.Add(new List<Vertex>());
        ringRoadDropOffEntrancesVertex.Add(new List<Vertex>());
        ringRoadDropOffExitsVertex.Add(new List<Vertex>());

        areas.Add(area);
    }

    public List<Area> getAreas()
    {
        return areas;
    }

    private double[] calculatePositionInCircle(Curve curve, double angle)
    {
        angle = (double)(angle / 180 * Math.PI);
        double[] curveCenter = curve.getCenter();
        double curveRadius = curve.getRadius();
        double posX = curveCenter[0] + Math.Sin(angle) * curveRadius;
        double posY = curveCenter[1] + Math.Cos(angle) * curveRadius;
        return new double[] { posX, posY };
    }

    public void addCurve(int label, int[] startVertexPosition, int[] endVertexPosition, double[] center, double initialAngle, double endAngle, double radius, bool entrance)
    {
        Vertex startVertex = findVertex(new int[] { startVertexPosition[0], startVertexPosition[1] });
        Vertex endVertex = findVertex(new int[] { endVertexPosition[0], endVertexPosition[1] });
        Curve curve = new Curve(label, startVertex, endVertex, center, initialAngle, endAngle, radius);
        addCurveRoad(curve);
        curveGridsMap.Add(new List<int>());
        curveGridsDeadlockMap.Add(new List<int>());
        curveGridsMapRobots.Add(new List<Robot>());
        double incrementAngle = 18;

        // create grids for reservation
        startVertex.setIsCurveStart(true);
        startVertex.setCurve(curve);
        startVertex.setIsCurveGrid(true);
        startVertex.setCurveGridLabel(0);
        startVertex.setCurveGridPosition(calculatePositionInCircle(curve, initialAngle));
        startVertex.setCurveAngle(initialAngle);
        if (entrance)
        {
            startVertex.setIsEntrance(true);
        }
        else
        {
            startVertex.setIsCorner(true);
            startVertex.setCornerCurve(curve);
        }

        curveGridsMap[curveGridsMap.Count - 1].Add(0);
        curveGridsDeadlockMap[curveGridsMap.Count - 1].Add(0);
        curveGridsMapRobots[curveGridsMap.Count - 1].Add(null);

        int curveGridLabel = 0;
        double angle = initialAngle > endAngle ? initialAngle - incrementAngle : initialAngle + incrementAngle;
        while (true)
        {
            double[] gridPosition = calculatePositionInCircle(curve, angle);
            Vertex curveGrid = new Vertex(startVertexPosition, 5);
            curveGrid.setIsCurveCenter(true);
            curveGrid.setCurve(curve);
            curveGrid.setIsCurveGrid(true);
            curveGrid.setCurveGridLabel(curveGridLabel++);
            curveGrid.setCurveGridPosition(calculatePositionInCircle(curve, angle));
            if (!entrance)
            {
                curveGrid.setIsCorner(true);
            }
            curveGrid.setCurveAngle(angle);
            curve.addCurveGrids(curveGrid);
            curveGridsMap[curveGridsMap.Count - 1].Add(0);
            curveGridsDeadlockMap[curveGridsMap.Count - 1].Add(0);
            curveGridsMapRobots[curveGridsMap.Count - 1].Add(null);

            if (initialAngle > endAngle)
            {
                angle -= incrementAngle;
                if (angle <= endAngle)
                {
                    break;
                }
            }
            else
            {
                angle += incrementAngle;
                if (angle >= endAngle)
                {
                    break;
                }
            }
        }

        endVertex.setIsCurveEnd(true);
        endVertex.setCurve(curve);
        endVertex.setIsCurveGrid(true);
        endVertex.setCurveGridLabel(curveGridLabel);
        endVertex.setCurveGridPosition(calculatePositionInCircle(curve, endAngle));
        endVertex.setCurveAngle(endAngle);
        if (entrance)
        {
            endVertex.setIsExit(true);
        }
        else
        {
            endVertex.setIsCorner(true);
            endVertex.setCornerCurve(curve);
        }
        curve.addCurveGrids(endVertex);
        curveGridsMap[curveGridsMap.Count - 1].Add(0);
        curveGridsDeadlockMap[curveGridsMap.Count - 1].Add(0);
        curveGridsMapRobots[curveGridsMap.Count - 1].Add(null);

        // set extra grid to reserve if is curve center
        List<Vertex> curveGrids = curve.getCurveGrids();
        for (int i = 0; i < curveGrids.Count; i++)
        {
            if (curveGrids[i].getIsCurveCenter())
            {
                double[] currentPosition = curveGrids[i].getCurveGridPosition();
                if (curve.getStartAngle() == 90f)
                {
                    // up + right
                    Vertex extraVertexToReserve = vertices[(int)Math.Ceiling(currentPosition[0])][(int)Math.Ceiling(currentPosition[1])];
                    if (extraVertexToReserve is MoveGrid)
                    {
                        curveGrids[i].setExtraGridOccupied(extraVertexToReserve);
                    }
                }
                else if (curve.getStartAngle() == 180f)
                {
                    // up + left
                    Vertex extraVertexToReserve = vertices[(int)Math.Ceiling(currentPosition[0])][(int)Math.Floor(currentPosition[1])];
                    if (extraVertexToReserve is MoveGrid)
                    {
                        curveGrids[i].setExtraGridOccupied(extraVertexToReserve);
                    }
                }
                else if (curve.getStartAngle() == 270f)
                {
                    // down + left
                    Vertex extraVertexToReserve = vertices[(int)Math.Floor(currentPosition[0])][(int)Math.Floor(currentPosition[1])];
                    if (extraVertexToReserve is MoveGrid)
                    {
                        curveGrids[i].setExtraGridOccupied(extraVertexToReserve);
                    }
                }
                else
                {
                    // down + right
                    Vertex extraVertexToReserve = vertices[(int)Math.Floor(currentPosition[0])][(int)Math.Ceiling(currentPosition[1])];
                    if (extraVertexToReserve is MoveGrid)
                    {
                        curveGrids[i].setExtraGridOccupied(extraVertexToReserve);
                    }
                }
            }
        }
    }

    public void initializeOuterRingRoad()
    {
        // 1 = left
        // 2 = right
        // 3 = up
        // 4 = down
        int[] shape = getShape();
        int[,] ringRoad = getRingRoad();
        for (int i = 0; i <= shape[0]; i++)
        {
            for (int j = 0; j <= shape[1]; j++)
            {
                Vertex currentVertex = vertices[i][j];
                int currentType = currentVertex.getType();
                int ringRoadType = ringRoad[i, j];

                if (ringRoadType == 4 && currentVertex.getIsExit())
                {
                    if (currentType == 0)
                    {
                        addRingRoadExitEdges(currentVertex, vertices[i][j + 1], 2, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i][j - 1], 1, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i + 1][j], 4, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i - 1][j], 3, 1);
                    }
                    else if (currentType == 1)
                    {
                        (currentVertex as MoveGrid).clearEdge();
                        addRingRoadExitEdges(currentVertex, vertices[i][j - 1], 1, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i][j + 1], 2, 1);
                    }
                    else if (currentType == 2)
                    {
                        (currentVertex as MoveGrid).clearEdge();
                        addRingRoadExitEdges(currentVertex, vertices[i][j - 1], 1, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i][j + 1], 2, 1);
                    }
                    else if (currentType == 3)
                    {
                        (currentVertex as MoveGrid).clearEdge();
                        addRingRoadExitEdges(currentVertex, vertices[i - 1][j], 3, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i + 1][j], 4, 1);
                    }
                    else if (currentType == 4)
                    {
                        (currentVertex as MoveGrid).clearEdge();
                        addRingRoadExitEdges(currentVertex, vertices[i - 1][j], 3, 1);
                        addRingRoadExitEdges(currentVertex, vertices[i + 1][j], 4, 1);
                    }
                }
                if (ringRoadType == 4 && currentVertex.getIsEntrance())
                {
                    if (currentType == 0)
                    {
                        (currentVertex as MoveGrid).clearEdge();
                        addRingRoadExitEdges(vertices[i][j + 1], currentVertex, 1, 2);
                        addRingRoadExitEdges(vertices[i][j - 1], currentVertex, 2, 2);
                        addRingRoadExitEdges(vertices[i + 1][j], currentVertex, 3, 2);
                        addRingRoadExitEdges(vertices[i - 1][j], currentVertex, 4, 2);
                    }
                    else if (currentType == 1)
                    {
                        addRingRoadExitEdges(vertices[i][j - 1], currentVertex, 2, 2);
                    }
                    else if (currentType == 2)
                    {
                        addRingRoadExitEdges(vertices[i][j + 1], currentVertex, 1, 2);
                    }
                    else if (currentType == 3)
                    {
                        addRingRoadExitEdges(vertices[i - 1][j], currentVertex, 4, 2);
                    }
                    else if (currentType == 4)
                    {
                        addRingRoadExitEdges(vertices[i + 1][j], currentVertex, 3, 2);
                    }
                }
            }
        }

        curveRoads = getCurveRoads();
    }

    public void Register(int[] position, Robot robot)
    {
        map[position[0], position[1]] = 1;
        mapRobots[position[0], position[1]] = robot;
    }

    public Vertex findVertex(int[] position)
    {
        return vertices[position[0]][position[1]];
    }

    public void generateTask()
    {
        System.Random rdm = new System.Random();
        List<PickUpPoint> pickUpPoints = getPickUpPoints();
        PickUpPoint pickUpPoint = pickUpPoints[rdm.Next(pickUpPoints.Count)];
        List<DropOffPoint> dropOffPoints = getDropOffPoints();
        DropOffPoint dropOffPoint = dropOffPoints[rdm.Next(dropOffPoints.Count)];
        tasks.Add(Tuple.Create(pickUpPoint, dropOffPoint));
    }

    public Tuple<PickUpPoint, DropOffPoint> getTask(int[] robotPosition)
    {
        int distance = 0;
        int closestIndex = 0;

        // get the task with pickup point with closest distance with the robot
        for (int i = 0; i < tasks.Count; i++)
        {
            int[] pickUpPosition = tasks[i].Item1.getLabel();
            int tempDistance = Math.Abs(robotPosition[0] - pickUpPosition[0]) + Math.Abs(robotPosition[1] - pickUpPosition[1]);
            if (tempDistance < distance)
            {
                distance = tempDistance;
                closestIndex = i;
            }
        }

        Tuple<PickUpPoint, DropOffPoint> closestPickUp = tasks[closestIndex];
        tasks.RemoveAt(closestIndex);
        generateTask();

        return closestPickUp;
    }

    public void setExitEntrances(int areaNo, int type, int task, int curveStartNo, int curveEndNo)
    {
        // type:
        // 1: set exit
        // 2: set entrance

        // task:
        // 1: pick up or charging
        // 2: drop off

        Area area = getAreas()[areaNo - 1];
        for (int i = curveStartNo; i <= curveEndNo; i++)
        {
            Curve curve = curveRoads[i];
            Vertex currentVertex;
            if (curve.getVertexStart().getIsOuterRingRoad())
            {
                currentVertex = curve.getVertexStart();
            }
            else
            {
                currentVertex = curve.getVertexEnd();
            }

            if (type == 1 && task == 1)
            {
                area.addPickUpExits(currentVertex);
                ringRoadPickUpExits[area.getLabel() - 1].Add(0);
                ringRoadPickUpExitsVertex[area.getLabel() - 1].Add(currentVertex);
            }
            else if (type == 1 && task == 2)
            {
                area.addDropOffExits(currentVertex);
                ringRoadDropOffExits[area.getLabel() - 1].Add(0);
                ringRoadDropOffExitsVertex[area.getLabel() - 1].Add(currentVertex);
            }
            else if (type == 2 && task == 1)
            {
                area.addPickUpEntrances(currentVertex);
                ringRoadPickUpEntrances[area.getLabel() - 1].Add(0);
                ringRoadPickUpEntrancesVertex[area.getLabel() - 1].Add(currentVertex);
            }
            else
            {
                area.addDropOffEntrances(currentVertex);
                ringRoadDropOffEntrances[area.getLabel() - 1].Add(0);
                ringRoadDropOffEntrancesVertex[area.getLabel() - 1].Add(currentVertex);
            }
        }
    }

    public Vertex requestRingRoadExitEntrance(Vertex vertex, int type, int task)
    {
        // type:
        // 1: get exit
        // 2: get entrance

        // task:
        // 1: pick up or charging
        // 2: drop off

        Area area = vertex.getArea();

        if (type == 1 && task == 1)
        {
            int leastIndex = ringRoadPickUpExits[area.getLabel() - 1].IndexOf(ringRoadPickUpExits[area.getLabel() - 1].Min());
            ringRoadPickUpExits[area.getLabel() - 1][leastIndex] += 1;
            return ringRoadPickUpExitsVertex[area.getLabel() - 1][leastIndex];
        }
        else if (type == 1 && task == 2)
        {
            int leastIndex = ringRoadDropOffExits[area.getLabel() - 1].IndexOf(ringRoadDropOffExits[area.getLabel() - 1].Min());
            ringRoadDropOffExits[area.getLabel() - 1][leastIndex] += 1;
            return ringRoadDropOffExitsVertex[area.getLabel() - 1][leastIndex];
        }
        else if (type == 2 && task == 1)
        {
            int leastIndex = ringRoadPickUpEntrances[area.getLabel() - 1].IndexOf(ringRoadPickUpEntrances[area.getLabel() - 1].Min());
            ringRoadPickUpEntrances[area.getLabel() - 1][leastIndex] += 1;
            return ringRoadPickUpEntrancesVertex[area.getLabel() - 1][leastIndex];
        }
        else
        {
            int leastIndex = ringRoadDropOffEntrances[area.getLabel() - 1].IndexOf(ringRoadDropOffEntrances[area.getLabel() - 1].Min());
            ringRoadDropOffEntrances[area.getLabel() - 1][leastIndex] += 1;
            return ringRoadDropOffEntrancesVertex[area.getLabel() - 1][leastIndex];
        }
    }

    public void ReleaseGrid(Vertex vertex, Robot robot)
    {
        if (vertex.getIsCurveCenter())
        {
            int curveLabel = vertex.getCurve().getLabel();
            int curveGridLabel = vertex.getCurveGridLabel();
            if (curveGridsMapRobots[curveLabel][curveGridLabel] == robot)
            {
                curveGridsMap[curveLabel][curveGridLabel] = 0;
                curveGridsDeadlockMap[curveLabel][curveGridLabel] = 0;
                curveGridsMapRobots[curveLabel][curveGridLabel] = null;
            }
            if (vertex.getExtraGridOccupied() != null)
            {
                int[] position = vertex.getExtraGridOccupied().getLabel();
                if (mapRobots[position[0], position[1]] == robot)
                {
                    map[position[0], position[1]] = 0;
                    deadlockMap[position[0], position[1]] = 0;
                    mapRobots[position[0], position[1]] = null;
                }
            }
        }
        else
        {
            int[] position = vertex.getLabel();
            if (mapRobots[position[0], position[1]] == robot)
            {
                map[position[0], position[1]] = 0;
                deadlockMap[position[0], position[1]] = 0;
                mapRobots[position[0], position[1]] = null;
            }
        }
    }

    public void changeGridOwnership(Robot robot, Vertex vertex)
    {
        if (vertex.getIsCurveCenter())
        {
            int curveLabel = vertex.getCurve().getLabel();
            int curveGridLabel = vertex.getCurveGridLabel();
            curveGridsMap[curveLabel][curveGridLabel] = 1;
            curveGridsDeadlockMap[curveLabel][curveGridLabel] = 0;
            curveGridsMapRobots[curveLabel][curveGridLabel] = robot;
            if (vertex.getExtraGridOccupied() != null)
            {
                int[] position = vertex.getExtraGridOccupied().getLabel();
                map[position[0], position[1]] = 1;
                deadlockMap[position[0], position[1]] = 0;
                mapRobots[position[0], position[1]] = robot;
            }
        }
        else
        {
            int[] position = vertex.getLabel();
            map[position[0], position[1]] = 1;
            deadlockMap[position[0], position[1]] = 0;
            mapRobots[position[0], position[1]] = robot;
        }
    }

    public bool ReserveGrid(Vertex currentVertex, Vertex vertex, Robot robot, int direction, int speedLevel)
    {
        // request curve grid
        if (vertex.getIsCurveCenter())
        {
            int curveLabel = vertex.getCurve().getLabel();
            int curveGridLabel = vertex.getCurveGridLabel();
            Vertex extraGridOccupied = vertex.getExtraGridOccupied();

            // for curve grid reservation
            if (curveGridsMap[curveLabel][curveGridLabel] == 0 && (extraGridOccupied == null || map[extraGridOccupied.getLabel()[0], extraGridOccupied.getLabel()[1]] == 0))
            {
                curveGridsMap[curveLabel][curveGridLabel] = 1;
                curveGridsDeadlockMap[curveLabel][curveGridLabel] = 0;
                curveGridsMapRobots[curveLabel][curveGridLabel] = robot;
                if (extraGridOccupied != null)
                {
                    int[] position = extraGridOccupied.getLabel();
                    map[position[0], position[1]] = 1;
                    deadlockMap[position[0], position[1]] = 0;
                    mapRobots[position[0], position[1]] = robot;
                }
                return true;
            }

            // for mutual acceleration
            else if (curveGridsMapRobots[curveLabel][curveGridLabel] != null)
            {
                Robot followingRobot = curveGridsMapRobots[curveLabel][curveGridLabel];
                if (followingRobot.requestFollow(robot, direction))
                {
                    robot.setFollowingRobot(followingRobot);
                }
            }
        }

        // request normal grid
        else
        {
            int[] position = vertex.getLabel();
            if (mapRobots[position[0], position[1]] == robot)
            {
                return true;
            }

            // for grid reservation
            if (map[position[0], position[1]] == 0)
            {
                map[position[0], position[1]] = 1;
                deadlockMap[position[0], position[1]] = 0;
                mapRobots[position[0], position[1]] = robot;
                return true;
            }

            // for mutual acceleration
            else if (mapRobots[position[0], position[1]] != null)
            {
                Robot followingRobot = mapRobots[position[0], position[1]];
                if (followingRobot.requestFollow(robot, direction))
                {
                    robot.setFollowingRobot(followingRobot);
                }
            }
        }

        // detect circular wait deadlock if deadlock happens
        if (speedLevel == 0)
        {
            if (currentVertex.getIsCurveCenter())
            {
                curveGridsDeadlockMap[currentVertex.getCurve().getLabel()][currentVertex.getCurveGridLabel()] = 1;
            }
            else
            {
                int[] currentPosition = currentVertex.getLabel();
                deadlockMap[currentPosition[0], currentPosition[1]] = direction;
            }
            checkAndBreakCircularWait(currentVertex);
        }
        return false;
    }

    public void checkAndBreakCircularWait(Vertex startVertex)
    {
        int currentDirection;
        int deadlockStartVertexIndex;
        Vertex nextVertex = startVertex;
        List<Vertex> deadlockedVertices = new List<Vertex>(1000);

        while (true)
        {
            // to detect whether deadlock exists and where the circular wait deadlock starts and ends
            deadlockStartVertexIndex = deadlockedVertices.FindIndex(e => e == nextVertex);
            if (deadlockStartVertexIndex >= 0)
            {
                break;
            }

            // if next grid is curve center grid
            if (nextVertex.getIsCurveCenter())
            {
                Curve nextCurve = nextVertex.getCurve();
                currentDirection = curveGridsDeadlockMap[nextCurve.getLabel()][nextVertex.getCurveGridLabel()];
                if (currentDirection == 1)
                {
                    deadlockedVertices.Add(nextVertex);
                    nextVertex = nextCurve.getCurveGrids()[nextVertex.getCurveGridLabel() + 1];
                }
                else if (nextVertex.getExtraGridOccupied() != null)
                {
                    nextVertex = nextCurve.getCurveGrids()[nextVertex.getCurveGridLabel() + 1];
                }
                else
                {
                    return;
                }
            }

            // not curve center grids
            else
            {
                deadlockedVertices.Add(nextVertex);
                int[] nextPosition = nextVertex.getLabel();
                currentDirection = deadlockMap[nextPosition[0], nextPosition[1]];
                if (currentDirection == 1)
                {
                    nextVertex = vertices[nextPosition[0]][nextPosition[1] - 1];
                }
                else if (currentDirection == 2)
                {
                    nextVertex = vertices[nextPosition[0]][nextPosition[1] + 1];
                }
                else if (currentDirection == 3)
                {
                    nextVertex = vertices[nextPosition[0] - 1][nextPosition[1]];
                }
                else if (currentDirection == 4)
                {
                    nextVertex = vertices[nextPosition[0] + 1][nextPosition[1]];
                }
                else if (currentDirection == 5)
                {
                    Curve nextCurve = nextVertex.getCurve();
                    nextVertex = nextCurve.getCurveGrids()[1];
                }
                else
                {
                    return;
                }
            }
        }

        // Allow all robots in the circular wait deadlock to move
        for (int i = deadlockStartVertexIndex; i < deadlockedVertices.Count; i++)
        {
            if (deadlockedVertices[i].getIsCurveCenter())
            {
                int curveLabel = deadlockedVertices[i].getCurve().getLabel();
                int curveGridLabel = deadlockedVertices[i].getCurveGridLabel();
                curveGridsMapRobots[curveLabel][curveGridLabel].setApprove(true);
                curveGridsDeadlockMap[curveLabel][curveGridLabel] = 0;
            }
            else
            {
                int[] currentVertexLabel = deadlockedVertices[i].getLabel();
                mapRobots[currentVertexLabel[0], currentVertexLabel[1]].setApprove(true);
                deadlockMap[currentVertexLabel[0], currentVertexLabel[1]] = 0;
            }
        }
    }

    public ChargingPoint requestCharging(bool urgent, Robot robot)
    {
        if (chargingPointsRobots.Length == 0)
        {
            return null;
        }

        // get all available charging points
        List<int> availableIndexes = new List<int>(500);
        for (int i = 0; i < chargingPointsRobots.Length; i++)
        {
            if (chargingPointsRobots[i] == null)
            {
                availableIndexes.Add(i);
            }
        }

        // battery level below 10%
        if (urgent && availableIndexes.Count == 0)
        {
            // find the robot with the highest battery level
            Robot highestBatteryRobot;
            highestBatteryRobot = chargingPointsRobots[0];
            int highestIndex = 0;
            for (int i = 1; i < chargingPointsRobots.Length; i++)
            {
                if (chargingPointsRobots[i].getBattery() > highestBatteryRobot.getBattery())
                {
                    highestBatteryRobot = chargingPointsRobots[i];
                    highestIndex = i;
                }
            }

            // ask the robot with the highest battery level and > 30% battery level to sort parcels
            if (chargingPointsRobots[highestIndex].getBattery() > 8640)
            {
                chargingPointsRobots[highestIndex].setCharging(false);
                chargingPointsRobots[highestIndex] = robot;
                return getChargingPoints()[highestIndex];
            }
            return null;
        }

        //battery level > 10% and no charging points available
        else if (availableIndexes.Count == 0)
        {
            return null;
        }

        // battery level > 10% and charging points available assign robot to the closest charging point
        int distance = 10000;
        int closestIndex = 0;
        int[] robotPosition = robot.getPosition();
        for (int i = 0; i < availableIndexes.Count; i++)
        {
            int[] chargingPosition = chargingPoints[availableIndexes[i]].getLabel();
            int tempDistance = (int)Math.Abs(robotPosition[0] - chargingPosition[0]) + (int)Math.Abs(robotPosition[1] - chargingPosition[1]);

            if (tempDistance < distance)
            {
                distance = tempDistance;
                closestIndex = i;
            }
        }

        // assign the robot to the charging point
        chargingPointsRobots[availableIndexes[closestIndex]] = robot;
        return getChargingPoints()[availableIndexes[closestIndex]];
    }

    public void notifyFullCharge(int label)
    {
        for (int i = 0; i < chargingPointsRobots.Length; i++)
        {
            if (chargingPointsRobots[i] != null && chargingPointsRobots[i].getLabel() == label)
            {
                chargingPointsRobots[i] = null;
                return;
            }
        }
    }

    public bool requestEnterRingRoad()
    {
        if (robotInRingRoadCount < maximumRobotInRingRoad)
        {
            robotInRingRoadCount++;
            return true;
        }
        return false;
    }

    public void addRobotInRingRoadCount(int count)
    {
        robotInRingRoadCount += count;
    }
}