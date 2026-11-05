package minecrafttransportsimulator.systems.summoning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * How far a vehicle has to drive between each surface cell near the player and a set of sources, going round
 * whatever's in the way and up and down hills.  It spreads out from the sources a slice at a time, like a flood.
 * <p>
 * It's quick and rough: it treats the vehicle as a point that needs {@link Settings#clearance} blocks of room round
 * it, and ignores how the vehicle turns.  That makes its distances never more than the real drive, so route searches
 * can use them as their estimate of the distance left, and it shows which way the drive goes from anywhere it reaches.
 * <p>
 * Cells are a block column and a surface height in it, so a road under a bridge and the bridge are different cells.
 * Moving to a neighbouring cell can climb {@link Settings#maxUp} and drop {@link Settings#maxDown}, in the direction
 * the vehicle drives.  A vehicle drives either to the sources, as to a goal, or away from them, as from a start.
 */
final class FlowField {
    /**Furthest from a cell we can look for things in the way when working out how much room it has, in blocks.**/
    static final int CLEARANCE_RANGE = 4;
    /**
     * The columns within {@link #CLEARANCE_RANGE}, other than the center, as {dx, dz}, nearest first, with the
     * distance from the center of the middle column to the nearest edge of each.
     */
    private static final int[][] CLEARANCE_OFFSETS;
    private static final double[] CLEARANCE_EDGES;
    static {
        List<int[]> offsets = new ArrayList<>();
        for (int dx = -CLEARANCE_RANGE; dx <= CLEARANCE_RANGE; ++dx) {
            for (int dz = -CLEARANCE_RANGE; dz <= CLEARANCE_RANGE; ++dz) {
                if ((dx != 0 || dz != 0) && Math.hypot(dx, dz) - 0.5 < CLEARANCE_RANGE) {
                    offsets.add(new int[] { dx, dz });
                }
            }
        }
        offsets.sort((a, b) -> Double.compare(Math.hypot(a[0], a[1]), Math.hypot(b[0], b[1])));
        CLEARANCE_OFFSETS = offsets.toArray(new int[0][]);
        CLEARANCE_EDGES = new double[CLEARANCE_OFFSETS.length];
        for (int i = 0; i < CLEARANCE_OFFSETS.length; ++i) {
            CLEARANCE_EDGES[i] = Math.hypot(CLEARANCE_OFFSETS[i][0], CLEARANCE_OFFSETS[i][1]) - 0.5;
        }
    }
    /**The 8 neighbouring columns, orthogonal first.**/
    private static final int[][] NEIGHBOURS = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

    /**What the vehicle needs, and how far the flood spreads.**/
    static final class Settings {
        /**Room the vehicle needs either side of the cell it's in, in blocks from the cell center to the nearest thing in the way.**/
        double clearance;
        /**Height from the ground the vehicle needs clear, in blocks.**/
        double headroom = 2;
        int maxUp = 1;
        int maxDown = 2;
        /**True if the vehicle drives to the sources, false if it drives away from them.**/
        boolean towardSources = true;
        /**
         * How far from a cell to look for things in the way, in blocks, up to {@link #CLEARANCE_RANGE}.  Cells with
         * more room than this count as having this much.  Looking only as far as matters saves a lot of checks.
         */
        double clearanceRange = CLEARANCE_RANGE;
        /**How far from the center the flood may spread, in blocks across.**/
        double radius = 64;
        /**Furthest drive from the sources the flood spreads to, in blocks.**/
        double maxDistance = Double.POSITIVE_INFINITY;
        /**Most cells to reach.**/
        int maxCells = 40000;
    }

    private final TerrainMap terrain;
    private final Settings settings;
    private final int centerX;
    private final int centerZ;
    private final Map<Long, Cell> cells = new HashMap<>();
    /**How much room each surface has, and whether each column's open at each height.  Floods with the same settings can share these.**/
    private Map<Long, Double> clearanceCache = new HashMap<>();
    private Map<Long, Boolean> openCache = new HashMap<>();
    /**Cells to spread from, each with the distance it was queued at, nearest first.**/
    private final PriorityQueue<Queued> open = new PriorityQueue<>((a, b) -> Double.compare(a.distance, b.distance));
    private boolean finished;
    /**Where the flood only needs to reach, and how far past it to carry on, if set: see {@link #setTarget}.**/
    private int[] target;
    private double targetMargin;
    private Cell targetCell;

    /**Makes a flood spreading out from no further than settings.radius from the center.  Add sources before stepping it.**/
    FlowField(TerrainMap terrain, Settings settings, int centerX, int centerZ) {
        this.terrain = terrain;
        this.settings = settings;
        this.centerX = centerX;
        this.centerZ = centerZ;
    }

    /**
     * Shares the other flood's record of how much room each surface has, so they don't both work it out.  Its settings
     * for room, headroom and climbing must be the same as this one's.  Call before adding sources.
     */
    void shareRoom(FlowField other) {
        clearanceCache = other.clearanceCache;
        openCache = other.openCache;
    }

    /**
     * Adds a source, at the extra distance.  Returns false and adds nothing if the vehicle can't be there, as
     * it isn't a surface or hasn't the room.
     */
    boolean addSource(int x, int y, int z, double extraDistance) {
        if (!isPassable(x, y, z)) {
            return false;
        }
        reach(x, y, z, null, extraDistance);
        return true;
    }

    /**
     * Has the flood stop once it's spread margin blocks past the first cell it reaches within radius columns of x,z and a
     * block of y, rather than spreading as far as it may.  For floods that only need to reach one place, like where a
     * vehicle that's already nearby is.  Call before stepping.
     */
    void setTarget(int x, int y, int z, int radius, double margin) {
        target = new int[] { x, y, z, radius };
        targetMargin = margin;
    }

    /**The cell the flood reached nearest its target, if it's set and the flood got there, or null.**/
    Cell getTargetCell() {
        return targetCell;
    }

    /**Spreads the flood until it's done or maxNanos pass.  Returns true once it's done.**/
    boolean step(long maxNanos) {
        long startTime = System.nanoTime();
        while (!finished && System.nanoTime() - startTime < maxNanos) {
            Queued queued = open.poll();
            if (queued == null || cells.size() >= settings.maxCells) {
                finished = true;
                break;
            }
            Cell current = queued.cell;
            //Skip cells already spread from, or reached more cheaply since this was queued.
            if (current.closed || queued.distance > current.distance) {
                continue;
            }
            current.closed = true;
            if (target != null && targetCell == null && Math.abs(current.x - target[0]) <= target[3] && Math.abs(current.z - target[2]) <= target[3] && Math.abs(current.y - target[1]) <= 1) {
                targetCell = current;
                settings.maxDistance = Math.min(settings.maxDistance, current.distance + targetMargin);
            }
            for (int i = 0; i < NEIGHBOURS.length; ++i) {
                spreadTo(current, NEIGHBOURS[i][0], NEIGHBOURS[i][1], i >= 4);
            }
        }
        return finished;
    }

    /**Spreads the flood until it's done.  For tests and anything else that can wait.**/
    void finish() {
        while (!step(Long.MAX_VALUE)) {
        }
    }

    boolean isFinished() {
        return finished;
    }

    /**
     * Reaches the neighbouring column from the cell, at each surface in it the vehicle could drive between it and
     * the cell.  Going diagonally needs both columns beside the corner open too, so it doesn't cut corners.
     */
    private void spreadTo(Cell current, int dx, int dz, boolean diagonal) {
        int x = current.x + dx;
        int z = current.z + dz;
        if (Math.hypot(x - centerX, z - centerZ) > settings.radius) {
            return;
        }
        //Heights the neighbour may be at.  Driving from it to us climbs what it's below us, and the other way round.
        int below = settings.towardSources ? settings.maxUp : settings.maxDown;
        int above = settings.towardSources ? settings.maxDown : settings.maxUp;
        //The columns beside the corner are passed on the way, so they're open by the same heights.
        if (diagonal && (!terrain.isOpen(current.x + dx, current.y, current.z, above, below, settings.headroom) || !terrain.isOpen(current.x, current.y, current.z + dz, above, below, settings.headroom))) {
            return;
        }
        double distance = current.distance + (diagonal ? Math.sqrt(2) : 1);
        if (distance > settings.maxDistance) {
            return;
        }
        for (int y = current.y + above; y >= current.y - below; --y) {
            if (isPassable(x, y, z)) {
                reach(x, y, z, current, distance);
            }
        }
    }

    private void reach(int x, int y, int z, Cell from, double distance) {
        long key = TerrainMap.blockKey(x, y, z);
        Cell cell = cells.get(key);
        if (cell == null) {
            cell = new Cell(x, y, z);
            cells.put(key, cell);
        } else if (cell.closed || distance >= cell.distance) {
            return;
        }
        cell.distance = distance;
        cell.next = from;
        open.add(new Queued(cell, distance));
    }

    /**Returns true if the vehicle could be at the surface: it's a surface, and it has the room.**/
    boolean isPassable(int x, int y, int z) {
        return terrain.isSurface(x, y, z, settings.headroom) && getClearance(x, y, z) >= settings.clearance;
    }

    private boolean isOpen(int x, int y, int z) {
        return openCache.computeIfAbsent(TerrainMap.blockKey(x, y, z), k -> terrain.isOpen(x, y, z, settings.maxUp, settings.maxDown, settings.headroom));
    }

    /**
     * How much room the vehicle has at the surface: the distance from the cell's center to the nearest edge of a
     * column it couldn't have a wheel in at this height, up to {@link Settings#clearanceRange}.
     */
    double getClearance(int x, int y, int z) {
        long key = TerrainMap.blockKey(x, y, z);
        Double cached = clearanceCache.get(key);
        if (cached == null) {
            //Columns are checked nearest first, so the first that isn't open is the nearest.
            double range = Math.min(settings.clearanceRange, CLEARANCE_RANGE);
            cached = range;
            for (int i = 0; i < CLEARANCE_OFFSETS.length && CLEARANCE_EDGES[i] < range; ++i) {
                if (!isOpen(x + CLEARANCE_OFFSETS[i][0], y, z + CLEARANCE_OFFSETS[i][1])) {
                    cached = CLEARANCE_EDGES[i];
                    break;
                }
            }
            clearanceCache.put(key, cached);
        }
        return cached;
    }

    /**The cell at the surface, or null if the flood hasn't reached it.**/
    Cell getCell(int x, int y, int z) {
        return cells.get(TerrainMap.blockKey(x, y, z));
    }

    /**
     * The distance to the sources from the column, at the surface nearest y within tolerance blocks of it, or
     * infinity if the flood hasn't reached any of those.
     */
    double getDistance(int x, int y, int z, int tolerance) {
        for (int offset = 0; offset <= tolerance; ++offset) {
            Cell cell = getCell(x, y + offset, z);
            if (cell == null && offset > 0) {
                cell = getCell(x, y - offset, z);
            }
            if (cell != null) {
                return cell.distance;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    /**How many cells the flood has reached so far.**/
    int getCellCount() {
        return cells.size();
    }

    /**Every cell the flood has reached so far.**/
    List<Cell> getCells() {
        return new ArrayList<>(cells.values());
    }

    private static final class Queued {
        final Cell cell;
        final double distance;

        Queued(Cell cell, double distance) {
            this.cell = cell;
            this.distance = distance;
        }
    }

    /**A surface the flood has reached, how far it is from the sources, and the next cell towards them.**/
    static final class Cell {
        final int x;
        final int y;
        final int z;
        double distance = Double.POSITIVE_INFINITY;
        /**The next cell on the way to the sources, or null for a source.**/
        Cell next;
        private boolean closed;

        private Cell(int x, int y, int z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /**
         * The yaw from this cell to the next one, which is the way the vehicle drives if the flood goes towards
         * the sources, or the way it comes from if it goes away from them.  NaN for a source.
         */
        double getYawToNext() {
            return next == null ? Double.NaN : Math.toDegrees(Math.atan2(next.x - x, next.z - z));
        }

        boolean isClosed() {
            return closed;
        }
    }
}
