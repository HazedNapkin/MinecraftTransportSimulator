package minecrafttransportsimulator.systems.navigation;

import java.util.Arrays;
import java.util.List;

/**
 * Vehicle shapes shared by the navigation and summoning tests.
 *
 * @author HazedNapkin
 */
public final class TestShapes {
    private TestShapes() {
    }

    /**A car about 2 blocks wide and 3 long, with a wheel at each corner.**/
    public static VehicleShape car() {
        List<double[]> boxes = Arrays.asList(new double[] { 0, 0.75, -1, 1, 0.5 }, new double[] { 0, 0.75, 1, 1, 0.5 });
        List<double[]> wheels = Arrays.asList(new double[] { 0.8, 0.25, 1.3, 0.25, 0.25 }, new double[] { -0.8, 0.25, 1.3, 0.25, 0.25 }, new double[] { 0.8, 0.25, -1.3, 0.25, 0.25 }, new double[] { -0.8, 0.25, -1.3, 0.25, 0.25 });
        return VehicleShape.of(boxes, wheels);
    }
}
