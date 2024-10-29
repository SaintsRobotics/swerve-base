package frc.robot.utils;

//TODO: add docs

public class FastPoly {
    public class Point {
        public final double x;
        public final double y;
        
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double cdsq(Point tar) {
            return Math.pow(tar.x - x, 2) + Math.pow(tar.y - y, 2);
        }
    }

    private enum Direction {
        DIRECTION_CW,
        DIRECTION_CCW
    }

    private final int sides;
    private final Point[] points;
    private final double[] ltlims;
    private final int[] ci = new int[2];

    
    public FastPoly(int sides, Point[] points, Point pos, double vx, double vy) { //NOTE: only convex polygons will work //TODO: enforce convex
        this.sides = sides;
        this.points = points;
        ltlims = new double[sides];
        
        
        Point ideal;
        boolean sk = true;

        updateLims(pos);
        for (int i = 0; i < sides-1; i++) {
            ci[0] = i;
            ci[1] = i+1;
            ideal = solveCI(pos, vx, vy);
            if (!(exceedLim(pos, ideal, ci[0], Direction.DIRECTION_CCW) || exceedLim(pos, ideal, ci[1], Direction.DIRECTION_CW))) {
                sk = false;
                break;
            }
        }
        if (sk) {
            ci[0]++;
            ci[1] = 0;
        }
    }

    private void updateLims(Point pos) {
        final Point rbpxo = new Point(pos.x + 1, pos.y);
        for (int i = 0; i < sides; i ++) {
            //NOTE: theta bearing ref is +x
            //NOTE: uses WPILib coordinate system
            
            //TODO: find a faster way to calculate angle limits
            final double bsq = pos.cdsq(points[i]);
            ltlims[i] = Math.acos((rbpxo.cdsq(points[i]) - bsq - 1) / (2 * Math.sqrt(bsq)));
        }
    }

    private Point solveCI(Point pos, double vx, double vy) {
        final double y1 = points[ci[0]].y;
        final double y2 = points[ci[1]].y;
        if (y1 == y2) { //m2 = infinity
            final double m1 = vx/vy;
            return new Point((y1 - pos.y + m1 * pos.x) / m1, y1);
        }

        final double x2 = points[ci[1]].x;
        final double m1 = vx/vy;
        final double m2 = (points[ci[0]].x - x2)/(y1 - y2);

        final double ypmm1xp = pos.y - m1 * pos.x;

        final double x = (m2 * x2 + ypmm1xp - y2) / (m2 - m1);
        return new Point(x, x * m1 + ypmm1xp);

        /*
         *  -m1 1   -m1x1+y1
         *  -m2 1   -m2x2+y2
         */
    }

    private boolean exceedLim(Point pos, Point sol, int i, Direction dir) {
        final Point rbpxo = new Point(pos.x + 1, pos.y);
        final double bsq = pos.cdsq(sol);
        final double lim = Math.acos((rbpxo.cdsq(sol) - bsq - 1) / (2 * Math.sqrt(bsq)));
        if (dir == Direction.DIRECTION_CW) {
            return lim >= ltlims[i];
        }
        return lim < ltlims[i];
    }

    // get required acceleration
    public double calc(Point pos, double vx, double vy) { //NOTE: velocities are field relative
        Point ideal = solveCI(pos, vx, vy);
        updateLims(pos);

        if (exceedLim(pos, ideal, ci[0], Direction.DIRECTION_CCW)) {
            do {
                ci[0]--;
                if (ci[0] == -1) ci[0] = sides-1;
                ideal = solveCI(pos, vx, vy);
            } while (exceedLim(pos, ideal, ci[0], Direction.DIRECTION_CCW));
            ci[1] = ci[0]+1;
            if (ci[1] == sides) ci[1] = 0;
        }
        else if (exceedLim(pos, ideal, ci[1], Direction.DIRECTION_CW)) {
            do {
                ci[1]++;
                if (ci[1] == sides) ci[1] = 0;
                ideal = solveCI(pos, vx, vy);
            } while (exceedLim(pos, ideal, ci[1], Direction.DIRECTION_CW));
            ci[0] = ci[1]-1;
            if (ci[0] == -1) ci[0] = sides-1;
        }

        return (Math.pow(vx, 2) + Math.pow(vy, 2) / (2 * Math.sqrt(Math.pow(ideal.x, 2) + Math.pow(ideal.y, 2))));
    }
}