using System;
using System.Collections.Generic;
using System.Linq;

public class RobotMovement
{
    private Robot robot;

    private int label;
    private Server server;

    // 1 = charging, 2 = pickup, 3 = dropoff

    // For normal navigation
    private Vector2 realTimePosition;
    private double fixedDeltaTime = 0.02;
    private Vertex position;
    private Vertex gridToRelease;
    private Vertex dropOffVertex;
    private List<Vertex> pickUpPoint;
    private List<Vertex> dropOffPoint;
    private ChargingPoint chargingPoint;
    private bool nextDropOff = false;
    private List<Vertex> route = new List<Vertex>();
    private int atRoute = 0;
    private List<Vertex> reservedGrids = new List<Vertex>(5);
    private bool reservationStatus = false;
    private Vector2 endPosition;
    private bool finishedRoute = true;
    private bool moving = false;
    private bool halfGrid = false;
    private bool approveOneNode = false;

    // For acceleration and deceleration
    private List<bool> changeDirection = new List<bool>();
    private bool lastChangeDirection = false;
    private int speedLevel = 0;
    private double[] speed = new double[] { 0, 1.118, 2.699, 3.517, 4.172, 4.736, 5 };
    private int lastSpeedLevel = 0;
    private bool lastReservationStatus = false;

    // For ring road
    private int threshold = 0;
    private bool requestedRingRoad = false;

    // For curve
    private bool isCurve = false;
    private Curve curve;
    private double currentAngle = 0;
    private double stopAngle = 0;
    private double endAngle = 0;
    private double angularSpeed = 90;

    public void Initialize(int label, Server server, int[] position, double fixedDeltaTime, System.Random rdm)
    {
        this.server = server;
        this.position = gridToRelease = server.findVertex(position);
        this.fixedDeltaTime = fixedDeltaTime;
        endPosition = new Vector2(position[1], position[0]);
        realTimePosition = endPosition;
        robot = new Robot(label, position, server, rdm);
        server.Register(position, robot);
        threshold = (int)((server.getShape()[0] + server.getShape()[1]) * 0.2);
    }

    private int getDirection(Vertex vertex1, Vertex vertex2)
    {
        if (vertex1.getIsCurveCenter() || vertex2.getIsCurveCenter())
        {
            return 5;
        }
        int[] position1 = vertex1.getLabel();
        int[] position2 = vertex2.getLabel();
        if (position1[0] < position2[0])
        {
            return 4; //up
        }
        else if (position1[0] > position2[0])
        {
            return 3; //down
        }
        else if (position1[1] < position2[1])
        {
            return 2; //right
        }
        else if (position1[1] > position2[1])
        {
            return 1; //left
        }
        return 0;
    }

    private int getDirection2(Vertex vertex1, Vertex vertex2)
    {
        if (vertex1.getIsOuterRingRoad() && vertex2.getIsCurveCenter() || vertex1.getIsCurveCenter() && vertex2.getIsOuterRingRoad())
        {
            int[] position1 = vertex1.getLabel();
            int[] position2 = vertex2.getLabel();

            if (vertex1.getIsCurveCenter())
            {
                position1 = new int[] { (int)Math.Round(vertex1.getCurveGridPosition()[0]), (int)Math.Round(vertex1.getCurveGridPosition()[1]) };
            }
            else
            {
                position2 = new int[] { (int)Math.Round(vertex2.getCurveGridPosition()[0]), (int)Math.Round(vertex2.getCurveGridPosition()[1]) };
            }

            if (position1[0] < position2[0])
            {
                return 4; //up
            }
            else if (position1[0] > position2[0])
            {
                return 3; //down
            }
            else if (position1[1] < position2[1])
            {
                return 2; //right
            }
            else if (position1[1] > position2[1])
            {
                return 1; //left
            }
        }
        else if (vertex1.getIsCurveCenter() || vertex2.getIsCurveCenter())
        {
            return 5;
        }
        else
        {
            int[] position1 = vertex1.getLabel();
            int[] position2 = vertex2.getLabel();
            if (position1[0] < position2[0])
            {
                return 4; //up
            }
            else if (position1[0] > position2[0])
            {
                return 3; //down
            }
            else if (position1[1] < position2[1])
            {
                return 2; //right
            }
            else if (position1[1] > position2[1])
            {
                return 1; //left
            }
        }
        return 0;
    }

    private List<bool> detectChangeDirection(List<Vertex> fullRoute)
    {
        if (fullRoute.Count <= 1)
        {
            return new List<bool>() { true };
        }
        List<bool> changeDirectionDetected = new List<bool>(fullRoute.Count);
        int currentDirection = getDirection2(position, fullRoute[0]);
        for (int i = 0; i < fullRoute.Count - 1; i++)
        {
            int nextDirection = getDirection2(fullRoute[i], fullRoute[i + 1]);
            if (currentDirection != nextDirection && currentDirection != 5 && nextDirection != 5)
            {
                changeDirectionDetected.Add(true);
            }
            else
            {
                changeDirectionDetected.Add(false);
            }
            currentDirection = nextDirection;
        }

        // stop at last vertex
        changeDirectionDetected.Add(true);
        return changeDirectionDetected;
    }

    private void addCost(List<Vertex> fullRoute, int addValue)
    {
        for (int i = 0; i < fullRoute.Count; i++)
        {
            fullRoute[i].addDynamicCost(addValue);
        }
    }

    private List<Vertex> findRoute(Vertex vertexStart, Vertex destinationVertex, List<Vertex> destinations, int task)
    {
        // type:
        // 1: set exit
        // 2: set entrance

        // task:
        // 1: pick up or charging
        // 2: drop off

        int[] vertexStartPosition = vertexStart.getLabel();
        int[] destinationPosition = destinations[0].getLabel();
        addCost(route, -1);
        atRoute = 0;

        // use ring road to reach areas in diagonal only
        // no.of robots in ring road cannot exceed a threshold
        int currentAreaLabel = vertexStart.getArea().getLabel();
        int nextAreaLabel = destinations[0].getArea().getLabel();
        int serverAreaCount = server.getAreas().Count;
        if (Math.Abs(currentAreaLabel - nextAreaLabel) > 1 &&
            !(currentAreaLabel == 1 && nextAreaLabel == serverAreaCount) && !(currentAreaLabel == serverAreaCount && nextAreaLabel == 1) &&
            server.requestEnterRingRoad())
        {
            requestedRingRoad = true;

            Vertex requestedEntrance = server.requestRingRoadExitEntrance(vertexStart, 2, task);
            Vertex requestedExit = server.requestRingRoadExitEntrance(destinationVertex, 1, task);
            List<Vertex> route1 = server.findRouteFromStartToEntrance(vertexStart, requestedEntrance);
            List<Vertex> route2 = server.findRouteInRingRoad(route1.Last(), requestedExit);
            List<Vertex> route3 = server.findRouteWithOneDestination(route2.Last(), destinations);
            return route1.Concat(route2).Concat(route3).ToList();
        }
        return server.findRouteWithOneDestination(vertexStart, destinations, 3);
    }

    private double[] moveInCurve(Curve curve, double angle)
    {
        angle = (double)(angle / 180 * Math.PI);
        double[] curveCenter = curve.getCenter();
        double curveRadius = curve.getRadius();
        double posX = curveCenter[1] + Math.Cos(angle) * curveRadius;
        double posY = curveCenter[0] + Math.Sin(angle) * curveRadius;
        return new double[] { posX, posY };
    }

    public void Update()
    {
        // Debug.Log(fixedDeltaTime);
        // battery charges
        if (robot.getCharging())
        {
            robot.charge(fixedDeltaTime);
            return;
        }

        // battery drops
        robot.setBattery(robot.getBattery() - fixedDeltaTime);

        // finished the task, get new task and find the route
        if (finishedRoute)
        {
            if (requestedRingRoad)
            {
                requestedRingRoad = false;
                server.addRobotInRingRoadCount(-1);
            }
            finishedRoute = false;
            if (nextDropOff)
            {
                route = findRoute(position, dropOffVertex, dropOffPoint, 2);
                changeDirection = detectChangeDirection(route);
                nextDropOff = false;
                addCost(route, 2);
            }
            else
            {
                int task = robot.getDesiredTask();
                // charging
                if (task != 3)
                {
                    pickUpPoint = dropOffPoint = null;
                    //charge
                    if (task == 1)
                    {
                        chargingPoint = server.requestCharging(true, robot);
                    }
                    // charge immediately
                    else
                    {
                        chargingPoint = server.requestCharging(false, robot);
                    }
                    if (chargingPoint != null)
                    {
                        route = findRoute(position, chargingPoint, new List<Vertex>() { chargingPoint as Vertex }, 1);
                        changeDirection = detectChangeDirection(route);
                        addCost(route, 2);
                    }
                }

                //sorting or no available charging point
                if (task == 3 || task != 3 && chargingPoint == null)
                {
                    chargingPoint = null;
                    Tuple<PickUpPoint, DropOffPoint> points = server.getTask(robot.getPosition());
                    pickUpPoint = points.Item1.getPickUpGrids();
                    dropOffVertex = points.Item2;
                    dropOffPoint = points.Item2.getDropOffGrids();
                    route = findRoute(position, points.Item1, pickUpPoint, 1);
                    changeDirection = detectChangeDirection(route);
                    nextDropOff = true;
                    addCost(route, 2);
                }
            }
        }

        if (!moving)
        {
            reservationStatus = false;
            // to reserve grid
            if (route.Count > 0 && (speedLevel < 6 || halfGrid) && !lastChangeDirection && !approveOneNode)
            {
                Vertex curVertex = route[atRoute];
                int direction = getDirection(position, curVertex);

                // normal grid reservation
                if (robot.getApprove())
                {
                    approveOneNode = true;
                    server.changeGridOwnership(robot, curVertex);
                    reservationStatus = true;
                }

                else if (server.ReserveGrid(position, curVertex, robot, direction, speedLevel))
                {
                    reservationStatus = true;
                }

                // robot follows another robot at the front
                //if (robot.getFollowingRobot() != null)
                //{
                //    Robot followingRobot = robot.getFollowingRobot();

                //    // request grid from the following robot
                //    if (followingRobot.requestGridOwnership(robot, curVertex, direction))
                //    {
                //        reservationStatus = true;
                //        server.changeGridOwnership(robot, curVertex);
                //    }

                //    // stop following if the following robot does not have grid needed
                //    else
                //    {
                //        followingRobot.setFollowByRobot(null);
                //        followingRobot.removeReservedVertex(curVertex);
                //        robot.setFollowingRobot(null);
                //    }
                //}

                // if reservation is granted either from server or other robots
                if (reservationStatus)
                {
                    reservedGrids.Add(curVertex);
                    lastChangeDirection = changeDirection[0];

                    // add reserved vertex that can be requested by robots at the back
                    if (route.Count > atRoute + 1)
                    {
                        robot.addReservedVertex(curVertex, getDirection(curVertex, route[atRoute + 1]));
                    }
                    else
                    {
                        robot.addReservedVertex(curVertex, 0);
                    }

                    atRoute++;
                    changeDirection.RemoveAt(0);
                }
            }

            // stop following if want to change direction or stop
            else if (lastChangeDirection && robot.getFollowingRobot() != null)
            {
                robot.getFollowingRobot().setFollowByRobot(null);
                robot.setFollowingRobot(null);
            }

            // to move to the next position
            if (reservedGrids.Count != 0 || halfGrid)
            {
                moving = true;

                if (isCurve && currentAngle == stopAngle)
                {
                    isCurve = false;
                }

                else if (!isCurve && !halfGrid && reservedGrids[0].getIsCurveCenter())
                {
                    isCurve = true;
                    curve = reservedGrids[0].getCurve();
                    currentAngle = curve.getStartAngle();
                    stopAngle = curve.getEndAngle();
                }

                // normal navigation
                if (!isCurve)
                {
                    if (halfGrid)
                    {
                        // move another half grid
                        halfGrid = false;
                        endPosition = new Vector2(position.getLabel()[1], position.getLabel()[0]);
                        robot.setPosition(position.getLabel());
                    }

                    else
                    {
                        // move for half grid
                        halfGrid = true;
                        gridToRelease = position;
                        position = reservedGrids[0];
                        reservedGrids.RemoveAt(0);

                        // normal navigation
                        int[] robotPosition = gridToRelease.getLabel();
                        endPosition = new Vector2((double)((position.getLabel()[1] + robotPosition[1]) / 2.0), (double)((position.getLabel()[0] + robotPosition[0]) / 2.0));

                        // to display parcel image
                        if (dropOffPoint != null && dropOffPoint.Contains(position))
                        {
                            server.addThroughput();
                        }
                    }
                }

                // moving in curve
                else
                {
                    if (halfGrid)
                    {
                        halfGrid = false;
                        endAngle = position.getCurveAngle();
                    }

                    else
                    {
                        halfGrid = true;
                        gridToRelease = position;
                        position = reservedGrids[0];
                        reservedGrids.RemoveAt(0);
                        endAngle = (gridToRelease.getCurveAngle() + position.getCurveAngle()) / 2;
                    }
                }
            }
        }

        // reached position or reached angle
        else if (realTimePosition == endPosition || (isCurve && currentAngle == endAngle))
        {
            moving = false;
            if (halfGrid)
            {
                gridToRelease.addDynamicCost(-1);
                server.ReleaseGrid(gridToRelease, robot);
                robot.removeReservedVertex(gridToRelease);
                robot.setApprove(false);
            }

            else
            {
                approveOneNode = false;

                // to display parcel image

                //finished whole route
                if (route.Count == atRoute && reservedGrids.Count == 0)
                {
                    finishedRoute = true;
                }

                // reached charging point
                if (chargingPoint != null && chargingPoint.getLabel() == position.getLabel())
                {
                    robot.setCharging(true);
                }
            }
        }

        // use average speed instead of actual acceleration and deceleration
        int reservedGridsCount = reservedGrids.Count;

        if (!halfGrid)
        {
            if (reservedGridsCount == 0)
            {
                if (lastSpeedLevel == 1)
                { // no available grids
                    speedLevel = 0;
                    lastChangeDirection = false;
                }
                else
                {
                    speedLevel = 1; // if both acceleration and deceleration
                }
            }
            else if (reservedGridsCount == 1)
            {
                if (reservationStatus)
                {
                    speedLevel = 2; // if acceleration
                }
                else
                {
                    speedLevel = 3; // if deceleration
                }
            }
            else if (reservedGridsCount == 2)
            {
                if (reservationStatus)
                {
                    speedLevel = 4; // if acceleration
                }
                else
                {
                    speedLevel = 5; // if deceleration
                }
            }
            else
            {
                speedLevel = 6; // if acceleration
            }
        }
        else
        {
            if (reservedGridsCount == 0)
            {
                if (reservationStatus)
                {
                    speedLevel = 1; // if acceleration
                }
                else
                {
                    speedLevel = 2; // if deceleration
                }
            }
            else if (reservedGridsCount == 1)
            {
                if (reservationStatus)
                {
                    speedLevel = 3; // if acceleration
                }
                else
                {
                    speedLevel = 4; // if deceleration
                }
            }
            else if (reservedGridsCount == 2)
            {
                if (reservationStatus && lastSpeedLevel == 6)
                {
                    speedLevel = 6; // if acceleration continously
                }
                else if (reservationStatus)
                {
                    speedLevel = 5; // if acceleration
                }
                else
                {
                    speedLevel = 6; // if deceleration
                }
            }
            else
            {
                speedLevel = 6;
            }
        }
        lastReservationStatus = reservationStatus;
        lastSpeedLevel = speedLevel;

        // move animation 
        if (speedLevel > 0)
        {
            // for normal navigation
            if (!isCurve)
            {
                Vector2 newPosition = Vector2.MoveTowards(realTimePosition, endPosition, speed[speedLevel] * fixedDeltaTime);
                realTimePosition = newPosition;
            }

            // for moving in curve
            else
            {
                if (currentAngle < stopAngle)
                {
                    currentAngle = Math.Min(currentAngle + angularSpeed * fixedDeltaTime * (speedLevel + 0.5) / 6, endAngle);
                }
                else if (currentAngle > stopAngle)
                {
                    currentAngle = Math.Max(currentAngle - angularSpeed * fixedDeltaTime * (speedLevel + 0.5) / 6, endAngle);
                }
                double[] newPositionFloat = moveInCurve(curve, currentAngle);

                Vector2 newPosition = Vector2.MoveTowards(realTimePosition, new Vector2(newPositionFloat[0], newPositionFloat[1]), speed[speedLevel] * fixedDeltaTime);
                realTimePosition = newPosition;
            }
        }
    }
}