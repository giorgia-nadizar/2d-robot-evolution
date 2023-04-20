/*
 * Copyright 2022 eric
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.units.erallab.robotevo2d.main;

import it.units.erallab.mrsim2d.core.Agent;
import it.units.erallab.mrsim2d.core.agents.gridvsr.DistributedNumGridVSR;
import it.units.erallab.mrsim2d.core.engine.Engine;
import it.units.erallab.mrsim2d.core.tasks.Task;
import it.units.erallab.mrsim2d.viewer.Drawer;
import it.units.erallab.mrsim2d.viewer.FramesImageBuilder;
import it.units.erallab.robotevo2d.main.PreparedNamedBuilder;
import it.units.malelab.jnb.core.NamedBuilder;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class ImageMaker {

  private static final String ENGINE_DESCRIPTION = "s.engine()";
  private static final String DRAWER_DESCRIPTION = "s.drawer(actions = false; miniAgents=none; enlargement = 1.5; engineProfiling = false)";

  private static final int W = 2000;
  private static final int H = 1500;
  private static final int N = 1;
  private static final double D_T = 0.25;
  private static final double T0 = 5d;

  private static final String TASK_DESCRIPTION = String.format(
      Locale.ROOT,
      "s.task.locomotion(terrain = s.t.flat(); duration = %f)",
      T0 + (N + 1d) * D_T
  );

  public static void main(String[] args) throws IOException {
    NamedBuilder<?> nb = PreparedNamedBuilder.get();
    String imgsPath = "D:/Research/VSR+NCA/images/";
    String robotFilePath = "C:/Users/giorg/PycharmProjects/nca-vsr-classifier/vsrs/video_file.csv";

    @SuppressWarnings("unchecked")
    Function<String, Drawer> drawer = (Function<String, Drawer>) nb.build(DRAWER_DESCRIPTION);
    @SuppressWarnings("unchecked")
    Supplier<Engine> engine = (Supplier<Engine>) nb.build(ENGINE_DESCRIPTION);
    @SuppressWarnings("unchecked")
    Task<Supplier<Agent>, ?> task = (Task<Supplier<Agent>, ?>) nb.build(TASK_DESCRIPTION);

    List<CSVRecord> records = new CSVParser(new FileReader(robotFilePath), CSVFormat.DEFAULT.withFirstRecordAsHeader())
        .getRecords();
    System.out.printf("Going to generate and save %d images.%n", records.size());
    for (CSVRecord record : records) {
      int seed = Integer.parseInt(record.get("seed"));
      String shape = record.get("shape");
      String serializedController = record.get("controller");
      DistributedNumGridVSR distributedNumGridVSR = VideoMaker.getRobot(shape, serializedController);
      String name = seed + "_" + shape;
      System.out.printf("Doing %s.%n", name);
      //noinspection DataFlowIssue

      Supplier<Agent> agent = () -> distributedNumGridVSR;
      FramesImageBuilder fib = new FramesImageBuilder(
          W,
          H,
          N,
          D_T,
          T0,
          FramesImageBuilder.Direction.HORIZONTAL,
          true,
          drawer.apply(name)
      );
      task.run(agent, engine.get(), fib);
      BufferedImage bufferedImage = fib.get();
      File imgFile = new File(imgsPath + File.separator + name + ".png");
      ImageIO.write(bufferedImage, "png", imgFile);
      System.out.printf("Image saved to %s.%n", imgFile);

    }
  }
}
