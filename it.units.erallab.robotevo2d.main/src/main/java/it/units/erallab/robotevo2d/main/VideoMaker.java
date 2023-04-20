package it.units.erallab.robotevo2d.main;

import it.units.erallab.mrsim2d.core.Snapshot;
import it.units.erallab.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import it.units.erallab.mrsim2d.core.engine.Engine;
import it.units.erallab.mrsim2d.core.functions.CompositeTRF;
import it.units.erallab.mrsim2d.core.functions.TimedRealFunction;
import it.units.erallab.mrsim2d.core.tasks.locomotion.Locomotion;
import it.units.erallab.mrsim2d.core.util.Grid;
import it.units.erallab.mrsim2d.viewer.Drawer;
import it.units.erallab.mrsim2d.viewer.VideoBuilder;
import it.units.erallab.mrsim2d.viewer.VideoUtils;
import it.units.erallab.robotevo2d.main.builders.Misc;
import it.units.malelab.jnb.core.NamedBuilder;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class VideoMaker {

  private static final String DRAWER_DESCRIPTION = "s.drawer(actions = false; miniAgents=none; enlargement = 1.5; engineProfiling = false)";

  @SuppressWarnings("unchecked")
  public static DistributedNumGridVSR getRobot(String readableShape, String serializedController) {
    NamedBuilder<Object> nb = PreparedNamedBuilder.get();
    String robotSkeleton = "s.a.distributedNumGridVSR(body=s.a.vsr.gridBody(sensorizingFunction=s.a.vsr.sf.uniform(sensors=[s.s.ar();s.s.rv(a=0.0);s.s.rv(a=90.0);s.s.d(a=-90.0)]);shape=s.a.vsr.s.free(s=\"" + readableShape + "\"));function=s.f.stepOut(innerFunction=s.f.mlp();stepT=0.2);signals=1.0)";
    DistributedNumGridVSR distributedNumGridVSR = (DistributedNumGridVSR) nb.build(robotSkeleton);
    Function<Object, Object> genotypeFunction = Misc.fromBase64(serializedController);
    double[] genotype = ((List<Double>) genotypeFunction.apply(null)).stream().mapToDouble(d -> d).toArray();

    Grid<TimedRealFunction> brainsGrid = distributedNumGridVSR.brainsGrid();
    for (Grid.Entry<TimedRealFunction> entry : brainsGrid) {
      if (entry.value() != null) {
        ((CompositeTRF) entry.value()).setParams(genotype);
      }
    }
    return distributedNumGridVSR;
  }

  @SuppressWarnings("unchecked")
  public static void main(String[] args) throws IOException {
    String robotFilePath = "C:\\Users\\giorg\\PycharmProjects\\nca-vsr-classifier\\vsrs\\video_file.csv";
    List<CSVRecord> records = new CSVParser(new FileReader(robotFilePath), CSVFormat.DEFAULT.withFirstRecordAsHeader())
        .getRecords();
    for (CSVRecord record : records) {
      int seed = Integer.parseInt(record.get("seed"));
      String shape = record.get("shape");
      String serializedController = record.get("controller");
      DistributedNumGridVSR distributedNumGridVSR = getRobot(shape, serializedController);
      System.out.println("Starting " + seed + " " + shape);

      String filename = "D:\\Research\\VSR+NCA\\videos\\" + seed + "_" + shape + ".mov";
      File f = new File(filename);
      if(f.exists() && !f.isDirectory()) {
        System.out.println("File already exists");
        continue;
      }


//      Function<String, Drawer> drawer = it.units.erallab.mrsim2d.buildable.builders.Misc.drawer(
//          1.5, 2, 1, 10, false, it.units.erallab.mrsim2d.buildable.builders.Misc.MiniAgentInfo.NONE, false, false
//      );



      Function<String, Drawer> drawer = (Function<String, Drawer>) PreparedNamedBuilder.get().build(DRAWER_DESCRIPTION);

      VideoBuilder consumer = new VideoBuilder(
          1200,
          800,
          0,
          30,
          30,
          VideoUtils.EncoderFacility.JCODEC,
          new File(filename),
          drawer.apply("")
      );

      Locomotion locomotion = (Locomotion) PreparedNamedBuilder.get().build("s.task.locomotion(terrain = s.t.flat(); duration = 30))");
      Supplier<Engine> engineSupplier = (Supplier<Engine>) PreparedNamedBuilder.get().build("sim.engine()");

      Object outcome = locomotion.run(() -> distributedNumGridVSR, engineSupplier.get(), consumer);
      File file = consumer.get();
      System.out.println(file.getAbsolutePath());

    }
  }

}
