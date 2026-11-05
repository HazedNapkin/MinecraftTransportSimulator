package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.List;

/**
 * Exact ways to drive from one pose to another, ignoring what's in the way, made of arcs at full lock and straights.
 * The route search uses these to join a route exactly onto the run-in it ends with, once it's close.
 * <p>
 * Dubins curves only drive forward: a turn, a straight or turn the other way, and a turn.  Reeds-Shepp curves may
 * also back up, changing direction between pieces, so they can turn round in much less room.  Every path is checked
 * by driving it, so only ones that really end at the pose are returned, shortest first.
 * <p>
 * Poses are as the search has them: x and z, and a yaw in degrees that rises turning left.  Lengths are in blocks,
 * negative backing up.
 */
final class AnalyticPaths {
    /**How far off the end pose a path may end, in blocks and degrees, and still count.**/
    private static final double END_TOLERANCE = 1e-4;
    private static final double END_YAW_TOLERANCE = 1e-3;
    private static final double ZERO = 1e-9;
    private static final double HALF_PI = Math.PI / 2;
    private static final double TWO_PI = Math.PI * 2;

    /**One way to drive between the poses: pieces of each kind, turning left, right or straight, and how far each goes.**/
    static final class Path {
        /**'L', 'R' or 'S' for each piece.**/
        final char[] types;
        /**How far each piece drives, in blocks, negative backing up.**/
        final double[] lengths;
        /**How far it drives in all, either way, in blocks.**/
        final double length;
        final double radius;

        Path(char[] types, double[] lengths, double radius) {
            this.types = types;
            this.lengths = lengths;
            this.radius = radius;
            double total = 0;
            for (double each : lengths) {
                total += Math.abs(each);
            }
            this.length = total;
        }

        /**The curvature of the piece, in radians per block driven forward, positive turning left.**/
        double getCurvature(int piece) {
            return types[piece] == 'L' ? 1 / radius : types[piece] == 'R' ? -1 / radius : 0;
        }

        /**True if it backs up anywhere.**/
        boolean reverses() {
            for (double each : lengths) {
                if (each < -ZERO) {
                    return true;
                }
            }
            return false;
        }

        /**Where driving it from the pose ends, as {x, z, yaw}.**/
        double[] drive(double x, double z, double yaw) {
            double[] pose = { x, z, yaw };
            for (int i = 0; i < types.length; ++i) {
                pose = HybridSearch.arc(pose[0], pose[1], pose[2], getCurvature(i), lengths[i]);
            }
            return pose;
        }
    }

    private AnalyticPaths() {
    }

    /**Every forward-only path from the first pose to the second turning at the radius, shortest first.**/
    static List<Path> dubins(double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        double[] local = toLocal(x0, z0, yaw0, x1, z1, yaw1, radius);
        List<double[]> raw = new ArrayList<>();
        List<char[]> types = new ArrayList<>();
        for (int reflect = 0; reflect < 2; ++reflect) {
            //Turning right is turning left in the mirror image.
            double x = local[0];
            double y = reflect == 0 ? local[1] : -local[1];
            double phi = reflect == 0 ? local[2] : -local[2];
            addDubins(raw, types, dubinsLSL(x, y, phi), "LSL", reflect == 1);
            addDubins(raw, types, dubinsLSR(x, y, phi), "LSR", reflect == 1);
            for (int side = -1; side <= 1; side += 2) {
                addDubins(raw, types, dubinsLRL(x, y, phi, side), "LRL", reflect == 1);
            }
        }
        return verify(raw, types, x0, z0, yaw0, x1, z1, yaw1, radius);
    }

    /**Every path from the first pose to the second turning at the radius that may back up, shortest first.**/
    static List<Path> reedsShepp(double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        double[] local = toLocal(x0, z0, yaw0, x1, z1, yaw1, radius);
        double x = local[0];
        double y = local[1];
        double phi = local[2];
        //The same pieces driven in the opposite order, which some families only solve this way round.
        double xb = x * Math.cos(phi) + y * Math.sin(phi);
        double yb = x * Math.sin(phi) - y * Math.cos(phi);
        List<double[]> raw = new ArrayList<>();
        List<char[]> types = new ArrayList<>();
        for (int family = 0; family < 8; ++family) {
            for (int backwards = 0; backwards < 2; ++backwards) {
                double fx = backwards == 0 ? x : xb;
                double fy = backwards == 0 ? y : yb;
                for (int transform = 0; transform < 4; ++transform) {
                    //Driving it all the other way flips x and phi; mirroring it flips y and phi.
                    boolean timeflip = (transform & 1) != 0;
                    boolean reflect = (transform & 2) != 0;
                    double tx = timeflip ? -fx : fx;
                    double ty = reflect ? -fy : fy;
                    double tphi = timeflip != reflect ? -phi : phi;
                    double[] lengths = solveFamily(family, tx, ty, tphi);
                    if (lengths == null) {
                        continue;
                    }
                    char[] familyTypes = FAMILY_TYPES[family].toCharArray();
                    double[] each = lengths.clone();
                    char[] eachTypes = familyTypes.clone();
                    for (int i = 0; i < each.length; ++i) {
                        if (timeflip) {
                            each[i] = -each[i];
                        }
                        if (reflect && eachTypes[i] != 'S') {
                            eachTypes[i] = eachTypes[i] == 'L' ? 'R' : 'L';
                        }
                    }
                    if (backwards == 1) {
                        reverse(each);
                        reverse(eachTypes);
                    }
                    raw.add(each);
                    types.add(eachTypes);
                }
            }
        }
        return verify(raw, types, x0, z0, yaw0, x1, z1, yaw1, radius);
    }

    /**
     * How long the shortest forward-only path from the first pose to the second is, in blocks.  Quick enough to use as
     * an estimate for every state near a goal, as it builds no paths.
     */
    static double dubinsLength(double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        double[] local = toLocal(x0, z0, yaw0, x1, z1, yaw1, radius);
        double shortest = Double.POSITIVE_INFINITY;
        for (int reflect = 0; reflect < 2; ++reflect) {
            double x = local[0];
            double y = reflect == 0 ? local[1] : -local[1];
            double phi = reflect == 0 ? local[2] : -local[2];
            shortest = Math.min(shortest, sum(dubinsLSL(x, y, phi)));
            shortest = Math.min(shortest, sum(dubinsLSR(x, y, phi)));
            shortest = Math.min(shortest, sum(dubinsLRL(x, y, phi, -1)));
            shortest = Math.min(shortest, sum(dubinsLRL(x, y, phi, 1)));
        }
        return shortest * radius;
    }

    /**How long the shortest path from the first pose to the second that may back up is, in blocks, as quick as {@link #dubinsLength}.**/
    static double reedsSheppLength(double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        double[] local = toLocal(x0, z0, yaw0, x1, z1, yaw1, radius);
        double x = local[0];
        double y = local[1];
        double phi = local[2];
        double xb = x * Math.cos(phi) + y * Math.sin(phi);
        double yb = x * Math.sin(phi) - y * Math.cos(phi);
        double shortest = Double.POSITIVE_INFINITY;
        for (int family = 0; family < 8; ++family) {
            for (int backwards = 0; backwards < 2; ++backwards) {
                double fx = backwards == 0 ? x : xb;
                double fy = backwards == 0 ? y : yb;
                for (int transform = 0; transform < 4; ++transform) {
                    boolean timeflip = (transform & 1) != 0;
                    boolean reflect = (transform & 2) != 0;
                    shortest = Math.min(shortest, sum(solveFamily(family, timeflip ? -fx : fx, reflect ? -fy : fy, timeflip != reflect ? -phi : phi)));
                }
            }
        }
        return shortest * radius;
    }

    /**The total of the lengths, either way, or infinity if there are none.**/
    private static double sum(double[] lengths) {
        if (lengths == null) {
            return Double.POSITIVE_INFINITY;
        }
        double total = 0;
        for (double each : lengths) {
            total += Math.abs(each);
        }
        return total;
    }

    /**The end pose in the start's frame scaled to a turn radius of 1, as {forward, left, turn in radians}.**/
    private static double[] toLocal(double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        double dx = x1 - x0;
        double dz = z1 - z0;
        double sin = Math.sin(Math.toRadians(yaw0));
        double cos = Math.cos(Math.toRadians(yaw0));
        //Forward is (sin, cos) of the yaw, and left (cos, -sin).
        return new double[] { (dx * sin + dz * cos) / radius, (dx * cos - dz * sin) / radius, Math.toRadians(VehicleSummonFlight.clampAngle(yaw1 - yaw0)) };
    }

    /**Drives each path to check it really ends at the pose, and returns those that do, in blocks, shortest first.**/
    private static List<Path> verify(List<double[]> raw, List<char[]> types, double x0, double z0, double yaw0, double x1, double z1, double yaw1, double radius) {
        List<Path> paths = new ArrayList<>();
        for (int i = 0; i < raw.size(); ++i) {
            double[] lengths = raw.get(i).clone();
            for (int j = 0; j < lengths.length; ++j) {
                lengths[j] *= radius;
            }
            Path path = new Path(types.get(i), lengths, radius);
            double[] end = path.drive(x0, z0, yaw0);
            if (Math.hypot(end[0] - x1, end[1] - z1) < END_TOLERANCE * Math.max(1, radius) && Math.abs(VehicleSummonFlight.clampAngle(end[2] - yaw1)) < END_YAW_TOLERANCE) {
                paths.add(path);
            }
        }
        paths.sort((a, b) -> Double.compare(a.length, b.length));
        return paths;
    }

    private static void addDubins(List<double[]> raw, List<char[]> types, double[] lengths, String word, boolean reflect) {
        if (lengths == null) {
            return;
        }
        char[] chars = word.toCharArray();
        if (reflect) {
            for (int i = 0; i < chars.length; ++i) {
                chars[i] = chars[i] == 'L' ? 'R' : chars[i] == 'R' ? 'L' : 'S';
            }
        }
        raw.add(lengths);
        types.add(chars);
    }

    //-----DUBINS, radius 1, starting at the origin facing +x, its left +y-----

    /**Turn left, straight, turn left.  The circles turned round are left of the start and of the end.**/
    private static double[] dubinsLSL(double x, double y, double phi) {
        double cx = x - Math.sin(phi);
        double cy = y + Math.cos(phi) - 1;
        double heading = Math.atan2(cy, cx);
        return new double[] { mod2piPositive(heading), Math.hypot(cx, cy), mod2piPositive(phi - heading) };
    }

    /**Turn left, straight, turn right, along the tangent crossing between the circles.**/
    private static double[] dubinsLSR(double x, double y, double phi) {
        double cx = x + Math.sin(phi);
        double cy = y - Math.cos(phi) - 1;
        double distanceSquared = cx * cx + cy * cy;
        if (distanceSquared < 4) {
            return null;
        }
        double straight = Math.sqrt(distanceSquared - 4);
        double heading = Math.atan2(cy, cx) + Math.atan2(2, straight);
        return new double[] { mod2piPositive(heading), straight, mod2piPositive(heading - phi) };
    }

    /**Turn left, right round a circle touching both, then left, with the middle circle to the given side.**/
    private static double[] dubinsLRL(double x, double y, double phi, int side) {
        double cx = x - Math.sin(phi);
        double cy = y + Math.cos(phi) - 1;
        double distance = Math.hypot(cx, cy);
        if (distance > 4 || distance < ZERO) {
            return null;
        }
        double alpha = Math.atan2(cy, cx) + side * Math.acos(distance / 4);
        double middleX = 2 * Math.cos(alpha);
        double middleY = 1 + 2 * Math.sin(alpha);
        double beta = Math.atan2(cy + 1 - middleY, cx - middleX);
        double first = alpha + HALF_PI;
        double second = beta - HALF_PI;
        return new double[] { mod2piPositive(first), mod2piPositive(first - second), mod2piPositive(phi - second) };
    }

    //-----REEDS-SHEPP, radius 1, as Reeds and Shepp's formulas 8.1 to 8.11-----

    /**The pieces of each family's base form, with + driving forward and - back in its formulas.**/
    private static final String[] FAMILY_TYPES = { "LSL", "LSR", "LRL", "LRLR", "LRLR", "LRSL", "LRSR", "LRSLR" };

    private static double[] solveFamily(int family, double x, double y, double phi) {
        switch (family) {
            case 0:
                return lpSpLp(x, y, phi);
            case 1:
                return lpSpRp(x, y, phi);
            case 2:
                return lpRmL(x, y, phi);
            case 3:
                return lpRupLumRm(x, y, phi);
            case 4:
                return lpRumLumRp(x, y, phi);
            case 5:
                return lpRmSmLm(x, y, phi);
            case 6:
                return lpRmSmRm(x, y, phi);
            default:
                return lpRmSLmRp(x, y, phi);
        }
    }

    private static double[] lpSpLp(double x, double y, double phi) {
        double u = Math.hypot(x - Math.sin(phi), y - 1 + Math.cos(phi));
        double t = Math.atan2(y - 1 + Math.cos(phi), x - Math.sin(phi));
        if (t >= -ZERO) {
            double v = mod2pi(phi - t);
            if (v >= -ZERO) {
                return new double[] { t, u, v };
            }
        }
        return null;
    }

    private static double[] lpSpRp(double x, double y, double phi) {
        double ex = x + Math.sin(phi);
        double ey = y - 1 - Math.cos(phi);
        double u1 = ex * ex + ey * ey;
        if (u1 >= 4) {
            double u = Math.sqrt(u1 - 4);
            double t = mod2pi(Math.atan2(ey, ex) + Math.atan2(2, u));
            double v = mod2pi(t - phi);
            if (t >= -ZERO && v >= -ZERO) {
                return new double[] { t, u, v };
            }
        }
        return null;
    }

    private static double[] lpRmL(double x, double y, double phi) {
        double xi = x - Math.sin(phi);
        double eta = y - 1 + Math.cos(phi);
        double u1 = Math.hypot(xi, eta);
        if (u1 <= 4) {
            double u = -2 * Math.asin(u1 / 4);
            double t = mod2pi(Math.atan2(eta, xi) + u / 2 + Math.PI);
            double v = mod2pi(phi - t + u);
            if (t >= -ZERO && u <= ZERO) {
                return new double[] { t, u, v };
            }
        }
        return null;
    }

    private static double[] lpRupLumRm(double x, double y, double phi) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = (2 + Math.hypot(xi, eta)) / 4;
        if (rho <= 1) {
            double u = Math.acos(rho);
            double[] tv = tauOmega(u, -u, xi, eta, phi);
            if (tv[0] >= -ZERO && tv[1] <= ZERO) {
                return new double[] { tv[0], u, -u, tv[1] };
            }
        }
        return null;
    }

    private static double[] lpRumLumRp(double x, double y, double phi) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = (20 - xi * xi - eta * eta) / 16;
        if (rho >= 0 && rho <= 1) {
            double u = -Math.acos(rho);
            if (u >= -HALF_PI) {
                double[] tv = tauOmega(u, u, xi, eta, phi);
                if (tv[0] >= -ZERO && tv[1] >= -ZERO) {
                    return new double[] { tv[0], u, u, tv[1] };
                }
            }
        }
        return null;
    }

    private static double[] lpRmSmLm(double x, double y, double phi) {
        double xi = x - Math.sin(phi);
        double eta = y - 1 + Math.cos(phi);
        double rho = Math.hypot(xi, eta);
        if (rho >= 2) {
            double r = Math.sqrt(rho * rho - 4);
            double u = 2 - r;
            double t = mod2pi(Math.atan2(eta, xi) + Math.atan2(r, -2));
            double v = mod2pi(phi - HALF_PI - t);
            if (t >= -ZERO && u <= ZERO && v <= ZERO) {
                return new double[] { t, -HALF_PI, u, v };
            }
        }
        return null;
    }

    private static double[] lpRmSmRm(double x, double y, double phi) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = Math.hypot(-eta, xi);
        if (rho >= 2) {
            double t = Math.atan2(xi, -eta);
            double u = 2 - rho;
            double v = mod2pi(t + HALF_PI - phi);
            if (t >= -ZERO && u <= ZERO && v <= ZERO) {
                return new double[] { t, -HALF_PI, u, v };
            }
        }
        return null;
    }

    private static double[] lpRmSLmRp(double x, double y, double phi) {
        double xi = x + Math.sin(phi);
        double eta = y - 1 - Math.cos(phi);
        double rho = Math.hypot(xi, eta);
        if (rho >= 2) {
            double u = 4 - Math.sqrt(rho * rho - 4);
            if (u <= ZERO) {
                double t = mod2pi(Math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                double v = mod2pi(t - phi);
                if (t >= -ZERO && v >= -ZERO) {
                    return new double[] { t, -HALF_PI, u, -HALF_PI, v };
                }
            }
        }
        return null;
    }

    private static double[] tauOmega(double u, double v, double xi, double eta, double phi) {
        double delta = mod2pi(u - v);
        double a = Math.sin(u) - Math.sin(delta);
        double b = Math.cos(u) - Math.cos(delta) - 1;
        double t1 = Math.atan2(eta * a - xi * b, xi * a + eta * b);
        double t2 = 2 * (Math.cos(delta) - Math.cos(v) - Math.cos(u)) + 3;
        double tau = t2 < 0 ? mod2pi(t1 + Math.PI) : mod2pi(t1);
        return new double[] { tau, mod2pi(tau - u + v - phi) };
    }

    /**The angle brought into -pi to pi.**/
    private static double mod2pi(double angle) {
        double wrapped = angle % TWO_PI;
        if (wrapped < -Math.PI) {
            wrapped += TWO_PI;
        } else if (wrapped > Math.PI) {
            wrapped -= TWO_PI;
        }
        return wrapped;
    }

    /**The angle brought into 0 to 2 pi, near 2 pi counting as 0.**/
    private static double mod2piPositive(double angle) {
        double wrapped = angle % TWO_PI;
        if (wrapped < 0) {
            wrapped += TWO_PI;
        }
        return wrapped > TWO_PI - ZERO ? 0 : wrapped;
    }

    private static void reverse(double[] values) {
        for (int i = 0, j = values.length - 1; i < j; ++i, --j) {
            double swap = values[i];
            values[i] = values[j];
            values[j] = swap;
        }
    }

    private static void reverse(char[] values) {
        for (int i = 0, j = values.length - 1; i < j; ++i, --j) {
            char swap = values[i];
            values[i] = values[j];
            values[j] = swap;
        }
    }
}
