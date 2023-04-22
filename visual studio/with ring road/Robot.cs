using System;
using System.Collections;
using System.Collections.Generic;

public class Robot
{
    private Server server;
    private int label;
    private bool serverApproved;
    private int[] position;
    private double battery;
    private bool charging;
    private Robot following;
    private Robot followByRobot;
    private List<Tuple<Vertex, int>> reservedVertexes;
    private Robot giveWayRobot;

    public Robot(int label, int[] position, Server server, Random rdm)
    {
        this.label = label;
        this.position = position;
        this.server = server;
        serverApproved = false;
        battery = rdm.Next(5760, 28800);
        // battery = 2000f;
        // 100% = 28800f, 30% = 8640f, 10% = 2880f
        charging = false;
        reservedVertexes = new List<Tuple<Vertex, int>>(20) { Tuple.Create(server.findVertex(position), 0) };
    }
    public int getLabel()
    {
        return label;
    }
    public void setApprove(bool serverApproved)
    {
        this.serverApproved = serverApproved;
        if (serverApproved && followByRobot != null)
        {
            followByRobot.setFollowingRobot(null);
            followByRobot = null;
        }
    }
    public bool getApprove()
    {
        return serverApproved;
    }
    public void updatePosition(int[] position)
    {
        this.position = position;
    }
    public void setPosition(int[] position)
    {
        this.position = position;
    }
    public int[] getPosition()
    {
        return position;
    }
    public int getDesiredTask()
    {
        // sort parcels
        if (battery > 8640f)
        {
            return 3;
        }
        // charge
        else if (battery > 2880f)
        {
            return 2;
        }
        // charge immediately
        return 1;
    }
    public void setBattery(double battery)
    {
        this.battery = battery;
    }
    public double getBattery()
    {
        return battery;
    }
    public void setCharging(bool charging)
    {
        this.charging = charging;
    }
    public bool getCharging()
    {
        return charging;
    }
    public void charge(double amount)
    {
        battery += amount * 5.4f;
        if (battery >= 28800f)
        {
            battery = 28800f;
            charging = false;
            server.notifyFullCharge(label);
        }
    }
    public void setFollowingRobot(Robot robot)
    {
        following = robot;
    }
    public Robot getFollowingRobot()
    {
        return following;
    }
    public void setFollowByRobot(Robot robot)
    {
        followByRobot = robot;
    }
    public Robot getFollowByRobot()
    {
        return followByRobot;
    }
    public bool requestFollow(Robot robot, int direction)
    {
        if (!serverApproved && (reservedVertexes[0].Item2 == direction || reservedVertexes[0].Item2 == 5 || direction == 5))
        {
            if (followByRobot == null)
            {
                setFollowByRobot(robot);
                return true;
            }
        }
        return false;
    }
    public void addReservedVertex(Vertex v, int direction)
    {
        reservedVertexes.Add(Tuple.Create(v, direction));
    }
    public void removeReservedVertex(Vertex vertex)
    {
        int index = reservedVertexes.FindIndex(e => e.Item1 == vertex) + 1;
        for (int i = 0; i < index; i++)
        {
            reservedVertexes.RemoveAt(0);
        }
    }
    public bool requestGridOwnership(Robot robot, Vertex vertex, int direction)
    {
        if (reservedVertexes.Count > 1)
        {
            Vertex firstVertex = reservedVertexes[0].Item1;
            int firstVertexDirection = reservedVertexes[0].Item2;
            if ((firstVertex == vertex || vertex.getExtraGridOccupied() == firstVertex || firstVertex.getExtraGridOccupied() == vertex) &&
            (firstVertexDirection == direction || firstVertexDirection == 5 || direction == 5))
            {
                return true;
            }
        }
        return false;
    }
}