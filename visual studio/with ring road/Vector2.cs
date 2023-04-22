using System;

class Vector2
{
    public double x;
    // Y component of the vector.
    public double y;
    public Vector2(double x, double y) { this.x = x; this.y = y; }
    public static Vector2 MoveTowards(Vector2 current, Vector2 target, double maxDistanceDelta)
    {
        // avoid vector ops because current scripting backends are terrible at inlining
        double toVector_x = target.x - current.x;
        double toVector_y = target.y - current.y;

        double sqDist = toVector_x * toVector_x + toVector_y * toVector_y;

        if (sqDist == 0 || (maxDistanceDelta >= 0 && sqDist <= maxDistanceDelta * maxDistanceDelta))
            return target;

        double dist = (double)Math.Sqrt(sqDist);

        return new Vector2(current.x + toVector_x / dist * maxDistanceDelta,
            current.y + toVector_y / dist * maxDistanceDelta);
    }
    public static bool operator == (Vector2 lhs, Vector2 rhs)
    {
        // Returns false in the presence of NaN values.
        double diff_x = lhs.x - rhs.x;
        double diff_y = lhs.y - rhs.y;
        return (diff_x * diff_x + diff_y * diff_y) < kEpsilon * kEpsilon;
    }
    public static bool operator !=(Vector2 lhs, Vector2 rhs)
    {
        // Returns true in the presence of NaN values.
        return !(lhs == rhs);
    }

    // *Undocumented*
    public const double kEpsilon = 0.00001;
    // *Undocumented*
    public const double kEpsilonNormalSqrt = 1e-15;
}

