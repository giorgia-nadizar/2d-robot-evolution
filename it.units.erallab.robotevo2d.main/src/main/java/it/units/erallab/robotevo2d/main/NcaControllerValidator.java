package it.units.erallab.robotevo2d.main;

import it.units.erallab.mrsim2d.buildable.builders.GridShapes;
import it.units.erallab.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import it.units.erallab.mrsim2d.core.engine.Engine;
import it.units.erallab.mrsim2d.core.functions.CompositeTRF;
import it.units.erallab.mrsim2d.core.functions.TimedRealFunction;
import it.units.erallab.mrsim2d.core.tasks.Outcome;
import it.units.erallab.mrsim2d.core.tasks.locomotion.Locomotion;
import it.units.erallab.mrsim2d.core.util.Grid;
import it.units.malelab.jnb.core.NamedBuilder;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.csv.CSVRecord;

import java.io.*;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class NcaControllerValidator {

  private record ControllerKey(int seed, int targetSet, int shapeId) {
  }


  private final Map<ControllerKey, double[]> controllersArchive;
  private final Map<Integer, Map<Integer, String>> shapes;
  private final List<CSVRecord> classificationOutcomes;
  private final String taskDescription;


  public NcaControllerValidator(String archiveFilePath, String classificationFilePath, String taskDescription) throws IOException {
    controllersArchive = parseArchiveFromFile(archiveFilePath);
    shapes = parseShapesFromFile(archiveFilePath);
    classificationOutcomes = new CSVParser(new FileReader(classificationFilePath), CSVFormat.DEFAULT.withFirstRecordAsHeader())
        .getRecords();
    this.taskDescription = taskDescription;
  }

  public NcaControllerValidator(String archiveFilePath, String classificationFilePath) throws IOException {
    this(archiveFilePath, classificationFilePath, "s.task.locomotion(terrain = s.t.flat(); duration = 30))");
  }

  private static Grid<ControllerKey> computeClassificationGrid(CSVRecord record, int seed) {
    int targetSet = Integer.parseInt(record.get("target_set"));
    String[] classifications = record.get("classification").split("-");
    int width = 1 + Arrays.stream(classifications).map(s -> s.split(";")[0]).mapToInt(Integer::parseInt).max().orElseThrow();
    int height = 1 + Arrays.stream(classifications).map(s -> s.split(";")[1]).mapToInt(Integer::parseInt).max().orElseThrow();
    Grid<ControllerKey> classificationGrid = Grid.create(width, height);
    Arrays.stream(classifications).forEach(s -> {
      int[] cc = Arrays.stream(s.split(";")).mapToInt(Integer::parseInt).toArray();
      classificationGrid.set(cc[0], cc[1], new ControllerKey(seed, targetSet, cc[2]));
    });
    return classificationGrid;
  }

  private DistributedNumGridVSR getRobotFromControllerKeyGrid(String readableShape, Grid<ControllerKey> controllerKeyGrid) {
    NamedBuilder<Object> nb = PreparedNamedBuilder.get();
    String robotSkeleton = "s.a.distributedNumGridVSR(body=s.a.vsr.gridBody(sensorizingFunction=s.a.vsr.sf.uniform(sensors=[s.s.ar();s.s.rv(a=0.0);s.s.rv(a=90.0);s.s.d(a=-90.0)]);shape=s.a.vsr.s.free(s=\"" + readableShape + "\"));function=s.f.stepOut(innerFunction=s.f.mlp();stepT=0.2);signals=1.0)";
    DistributedNumGridVSR distributedNumGridVSR = (DistributedNumGridVSR) nb.build(robotSkeleton);

    Grid<TimedRealFunction> brainsGrid = distributedNumGridVSR.brainsGrid();
    for (Grid.Entry<TimedRealFunction> entry : brainsGrid) {
      ControllerKey controllerKey = controllerKeyGrid.get(entry.key());
      if (entry.value() != null) {
        ((CompositeTRF) entry.value()).setParams(controllersArchive.get(controllerKey));
      }
    }
    return distributedNumGridVSR;
  }


  private Grid<ControllerKey> computeControllerKeyGridWithMixedSeeds(int targetSet, int shapeId, int mainSeed, double otherSeedsRate) {
    Random random = new Random();
    int[] otherSeeds = IntStream.range(1, 6).filter(s -> s != mainSeed).toArray();
    int nVoxels = (int) shapes.get(targetSet).get(shapeId).chars().filter(ch -> ch == '1').count();
    List<Integer> chosenSeeds = new ArrayList<>(IntStream.range(0, nVoxels).map(i ->
        i < otherSeedsRate * nVoxels ? otherSeeds[random.nextInt(otherSeeds.length)] : mainSeed
    ).boxed().toList());
    Collections.shuffle(chosenSeeds, random);
    int counter = 0;
    Grid<Boolean> bodyGrid = GridShapes.free(shapes.get(targetSet).get(shapeId));
    Grid<ControllerKey> controllerKeyGrid = Grid.create(bodyGrid);
    for (Grid.Entry<Boolean> entry : bodyGrid) {
      if (entry.value()) {
        controllerKeyGrid.set(entry.key(), new ControllerKey(chosenSeeds.get(counter), targetSet, shapeId));
        counter++;
      }
    }
    return controllerKeyGrid;
  }

  private Outcome evaluateRobot(DistributedNumGridVSR vsr) {
    Locomotion locomotion = (Locomotion) PreparedNamedBuilder.get().build(taskDescription);
    return locomotion.run(() -> vsr, ServiceLoader.load(Engine.class).findFirst().orElseThrow());
  }

  public void evaluateAllControllersWithMixedSeeds(String targetFileName, Collection<Double> rates, int nRepetitions) throws IOException {
    CSVPrinter printer = new CSVPrinter(new FileWriter(targetFileName), CSVFormat.DEFAULT);
    printer.printRecord("target_set", "shape_id", "main_seed", "readable_shape", "baseline_vx", "rate", "rep", "vx", "seed_shape");

    int counter = 0;
    for (int targetSet : shapes.keySet()) {
      for (int shapeId : shapes.get(targetSet).keySet()) {
        for (int mainSeed : IntStream.range(1, 6).toArray()) {
          DistributedNumGridVSR vsr = getRobotFromControllerKeyGrid(shapes.get(targetSet).get(shapeId), computeControllerKeyGridWithMixedSeeds(targetSet, shapeId, mainSeed, 0d));
          double baselineVx = evaluateRobot(vsr).firstAgentXVelocity();
          String readableShape = shapes.get(targetSet).get(shapeId);
          List<List<Object>> rows = new ArrayList<>();
          rates.stream().parallel().forEach(rate ->
              IntStream.range(0, nRepetitions).parallel().forEach(idx -> {
                Grid<ControllerKey> controllerKeyGrid = computeControllerKeyGridWithMixedSeeds(targetSet, shapeId, mainSeed, rate);
                DistributedNumGridVSR vsrMixedSeed = getRobotFromControllerKeyGrid(shapes.get(targetSet).get(shapeId), controllerKeyGrid);
                double vx = evaluateRobot(vsrMixedSeed).firstAgentXVelocity();
                Function<Grid.Entry<ControllerKey>, Character> cf = e -> e.value() == null ? 'x' : (char) (e.value().seed + '0');
                String[] seedRows = Grid.toString(controllerKeyGrid, cf, "-").split("-");
                String seedGrid = IntStream.range(0, seedRows.length).mapToObj(i -> seedRows[seedRows.length - 1 - i]).collect(Collectors.joining("-"));
                rows.add(List.of(targetSet, shapeId, mainSeed, readableShape, baselineVx, rate, idx, vx, seedGrid));
              })
          );
          for (List<Object> r : rows) {
            printer.printRecord(r);
          }
          System.out.printf("%d) set:%d, shape:%d, seed:%d\n", counter, targetSet, shapeId, mainSeed);
          counter++;
        }
      }
    }

  }


  public void evaluateClassifiedRobots(String targetFileName) throws IOException {
    List<String> headers = new ArrayList<>(classificationOutcomes.get(0).toMap().keySet().stream().toList());
    headers.addAll(List.of("seed", "classification_vx", "majority_vote_vx"));

    CSVPrinter printer = new CSVPrinter(new FileWriter(targetFileName), CSVFormat.DEFAULT);
    printer.printRecord(headers);

    int counter = 0;
    long start = System.currentTimeMillis();
    for (CSVRecord record : classificationOutcomes) {
      List<List<Object>> rows = IntStream.range(1, 6).parallel().mapToObj(seed -> {
        String readableShape = record.get("readable_shape");
        // without majority voting
        Grid<ControllerKey> controllerKeyGrid = computeClassificationGrid(record, seed);
        DistributedNumGridVSR vsr = getRobotFromControllerKeyGrid(readableShape, controllerKeyGrid);
        double classificationVx = evaluateRobot(vsr).firstAgentXVelocity();

        // with majority voting
        int majorityVote = Integer.parseInt(record.get("majority_vote"));
        Grid<ControllerKey> majorityVoteControllerKeyGrid = Grid.create(controllerKeyGrid, k -> k == null ? null : new ControllerKey(k.seed, k.targetSet, majorityVote));
        DistributedNumGridVSR vsrMajorityVoteController = getRobotFromControllerKeyGrid(readableShape, majorityVoteControllerKeyGrid);
        double majorityVoteVx = evaluateRobot(vsrMajorityVoteController).firstAgentXVelocity();

        List<Object> row = new ArrayList<>();
        row.addAll(record.stream().toList());
        row.addAll(List.of(seed, classificationVx, majorityVoteVx));
        return row;

      }).toList();
      for (List<Object> r : rows) {
        printer.printRecord(r);
      }
      counter++;
      System.out.printf("Record %d/%d, %f\n", counter, classificationOutcomes.size(), (double) (System.currentTimeMillis() - start) / 1000);
    }
  }


  @SuppressWarnings("unchecked")
  private static Map<ControllerKey, double[]> parseArchiveFromFile(String archiveFilePath) throws IOException {
    CSVParser csvParser = new CSVParser(new FileReader(archiveFilePath), CSVFormat.DEFAULT.withFirstRecordAsHeader());
    List<CSVRecord> records = csvParser.getRecords();
    Map<ControllerKey, double[]> map = new HashMap<>();
    for (CSVRecord record : records) {
      int seed = Integer.parseInt(record.get("seed"));
      int targetSet = Integer.parseInt(record.get("target_set"));
      int shapeId = Integer.parseInt(record.get("shape_id"));
      ControllerKey key = new ControllerKey(seed, targetSet, shapeId);
      double[] controller = ((List<Double>) base64Deserializer(record.get("controller"))).stream().mapToDouble(d -> d).toArray();
      map.put(key, controller);
    }
    return map;
  }

  private static Map<Integer, Map<Integer, String>> parseShapesFromFile(String shapesFile) throws IOException {
    CSVParser csvParser = new CSVParser(new FileReader(shapesFile), CSVFormat.DEFAULT.withFirstRecordAsHeader());
    List<CSVRecord> records = csvParser.getRecords();
    Map<Integer, Map<Integer, String>> map = new HashMap<>();
    for (CSVRecord record : records) {
      int targetSet = Integer.parseInt(record.get("target_set"));
      int shapeId = Integer.parseInt(record.get("shape_id"));
      String readableShape = record.get("readable_shape");
      map.computeIfAbsent(targetSet, d -> new HashMap<>());
      map.get(targetSet).put(shapeId, readableShape);
    }
    return map;
  }

  public static Object base64Deserializer(String serialized) {
    byte[] bytes = Base64.getDecoder().decode(serialized);
    try (ObjectInputStream oois = new ObjectInputStream(new ByteArrayInputStream(bytes))) {
      return oois.readObject();
    } catch (IOException | ClassNotFoundException e) {
      throw new RuntimeException(e);
    }
  }

  public static void main(String[] args) throws Exception {
    String controllersFile = "C:/Users/giorg/PycharmProjects/nca-vsr-classifier/vsrs/controllers_archive.csv";
    String classificationFile = "C:/Users/giorg/PycharmProjects/nca-vsr-classifier/classifications/larger_net_classification.csv";
    String targetFile = "classification_velocities.csv";
    if (args != null && args.length > 0) {
      for (String arg : args) {
        if (arg.startsWith("controllers=")) {
          controllersFile = arg.replace("controllers=", "");
        }
        if (arg.startsWith("classification=")) {
          classificationFile = arg.replace("classification=", "");
        }
        if (arg.startsWith("target=")) {
          targetFile = arg.replace("target=", "");
        }
      }
    }
    NcaControllerValidator a = new NcaControllerValidator(controllersFile, classificationFile);
    a.evaluateClassifiedRobots(targetFile);
  }


}
