package frc.ExternalLib.JackInTheBotLib.io;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import org.ejml.simple.SimpleMatrix;

import frc.ExternalLib.JackInTheBotLib.control.Path;
import frc.ExternalLib.JackInTheBotLib.control.PathSegment;
import frc.ExternalLib.JackInTheBotLib.io.json.InterpolatingDoubleJsonHandler;
import frc.ExternalLib.JackInTheBotLib.io.json.PathSegmentJsonHandler;
import frc.ExternalLib.JackInTheBotLib.io.json.Rotation2JsonHandler;
import frc.ExternalLib.JackInTheBotLib.io.json.SimpleMatrixJsonHandler;
import frc.ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.ExternalLib.JackInTheBotLib.util.InterpolatingDouble;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;

import java.io.IOException;
import java.io.Reader;
import java.lang.reflect.Type;
import java.util.Map;
import java.util.TreeMap;

public final class PathReader implements AutoCloseable {
    private final Gson gson;
    private final Reader in;

    public PathReader(Reader in) {
        this.gson = new GsonBuilder()
                .registerTypeAdapter(InterpolatingDouble.class, new InterpolatingDoubleJsonHandler())
                .registerTypeHierarchyAdapter(PathSegment.class, new PathSegmentJsonHandler())
                .registerTypeAdapter(Rotation2.class, new Rotation2JsonHandler())
                .registerTypeAdapter(SimpleMatrix.class, new SimpleMatrixJsonHandler())
                .create();
        this.in = in;
    }

    public Path read() throws IOException {
        try {
            JsonElement rootElement = JsonParser.parseReader(in);
            if (!rootElement.isJsonObject()) {
                throw new IOException("Path must be a JSON object");
            }

            JsonObject root = rootElement.getAsJsonObject();
            if (!root.has("segments") || !root.has("rotations")) {
                throw new IOException("Path is not valid");
            }

            PathSegment[] pathSegments = gson.fromJson(root.get("segments"), PathSegment[].class);

            Type rotationMapType = new TypeToken<TreeMap<Double, Rotation2>>() {
            }.getType();
            Map<Double, NWRotation2d> rotations = gson.fromJson(root.get("rotations"), rotationMapType);

            return new Path(pathSegments, rotations);
        } catch (JsonIOException | JsonSyntaxException e) {
            throw new IOException(e);
        }
    }

    @Override
    public void close() throws IOException {
        in.close();
    }
}
