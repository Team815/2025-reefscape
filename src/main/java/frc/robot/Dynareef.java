package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Arrays;

public final class Dynareef {

//    public static Command buildAuto() {
//            return Commands.deferredProxy(Dynareef::buildAutoPrivate);
//    }

    public static Command buildAuto() {
        var paths = Arrays.stream(
                NetworkTableInstance
                    .getDefault()
                    .getTable("dynareef")
                    .getEntry("path")
                    .getIntegerArray(new long[0])
            )
            .mapToObj(pathId -> {
                try {
                    return PathPlannerPath.fromPathFile(Dynareef.getPathName((int) pathId));
                } catch (IOException | ParseException e) {
                    throw new RuntimeException(e);
                }
            })
            .toArray(PathPlannerPath[]::new);

        var autoCommand = paths.length > 0
            ? AutoBuilder.resetOdom(paths[0].getStartingHolonomicPose().orElseThrow())
            : Commands.none();

        for (var path : paths) {
            autoCommand = autoCommand.andThen(AutoBuilder.followPath(path));
        }

        return autoCommand;
    }

    private static String getPathName(int pathId) {
        return switch (pathId) {

            // Start Paths

            case 600040, 600041, 600042, 600043 -> "Start To BR Reef L Branch";
            case 600050, 600051, 600052, 600053 -> "Start To BR Reef R Branch";
            case 500060, 500061, 500062, 500063 -> "Start To R Reef B Branch";
            case 500070, 500071, 500072, 500073 -> "Start To R Reef T Branch";
            case 400080, 400081, 400082, 400083 -> "Start To TR Reef R Branch";
            case 400090, 400091, 400092, 400093 -> "Start To TR Reef L Branch";

            // Top Station Paths

            case 200000, 200001, 200002, 200003 -> "Top Station To L Reef T Branch";
            case 200, 1200, 2200, 3200 -> "L Reef T Branch To Top Station";
            case 200010, 200011, 200012, 200013 -> "Top Station To L Reef B Branch";
            case 10200, 11200, 12200, 13200 -> "L Reef B Branch To Top Station";
            case 200060, 200061, 200062, 200063 -> "Top Station To R Reef B Branch";
            case 60200, 61200, 62200, 63200 -> "R Reef B Branch To Top Station";
            case 200070, 200071, 200072, 200073 -> "Top Station To R Reef T Branch";
            case 70200, 71200, 72200, 73200 -> "R Reef T Branch To Top Station";
            case 200080, 200081, 200082, 200083 -> "Top Station To TR Reef R Branch";
            case 80200, 81200, 82200, 83200 -> "TR Reef R Branch To Top Station";
            case 200090, 200091, 200092, 200093 -> "Top Station To TR Reef L Branch";
            case 90200, 91200, 92200, 93200 -> "TR Reef L Branch To Top Station";
            case 200100, 200101, 200102, 200103 -> "Top Station To TL Reef R Branch";
            case 100200, 101200, 102200, 103200 -> "TL Reef R Branch To Top Station";
            case 200110, 200111, 200112, 200113 -> "Top Station To TL Reef L Branch";
            case 110200, 111200, 112200, 113200 -> "TL Reef L Branch To Top Station";

            // Bottom Station Paths

            case 300000, 300001, 300002, 300003 -> "Bottom Station To L Reef T Branch";
            case 300, 1300, 2300, 3300 -> "L Reef T Branch To Bottom Station";
            case 300010, 300011, 300012, 300013 -> "Bottom Station To L Reef B Branch";
            case 10300, 11300, 12300, 13300 -> "L Reef B Branch To Bottom Station";
            case 300020, 300021, 300022, 300023 -> "Bottom Station To BL Reef L Branch";
            case 20300, 21300, 22300, 23300 -> "BL Reef L Branch To Bottom Station";
            case 300030, 300031, 300032, 300033 -> "Bottom Station To BL Reef R Branch";
            case 30300, 31300, 32300, 33300 -> "BL Reef R Branch To Bottom Station";
            case 300040, 300041, 300042, 300043 -> "Bottom Station To BR Reef L Branch";
            case 40300, 41300, 42300, 43300 -> "BR Reef L Branch To Bottom Station";
            case 300050, 300051, 300052, 300053 -> "Bottom Station To BR Reef R Branch";
            case 50300, 51300, 52300, 53300 -> "BR Reef R Branch To Bottom Station";
            case 300060, 300061, 300062, 300063 -> "Bottom Station To R Reef B Branch";
            case 60300, 61300, 62300, 63300 -> "R Reef B Branch To Bottom Station";
            case 300070, 300071, 300072, 300073 -> "Bottom Station To R Reef T Branch";
            case 70300, 71300, 72300, 73300 -> "R Reef T Branch To Bottom Station";

            // Invalid

            default -> "INVALID PATH ID";
        };
    }
}
