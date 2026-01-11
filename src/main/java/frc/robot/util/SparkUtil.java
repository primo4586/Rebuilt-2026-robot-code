// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /**
   * Prints the last error and sets the sticky fault flag for the given {@link SparkBase}.
   *
   * @param spark the {@link SparkBase} with the error
   */
  private static void printError(SparkBase spark) {
    System.out.println("spark " + spark.getDeviceId() + "is being a bitch ");
    System.out.println(spark.getLastError());
    sparkStickyFault = true;
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      printError(spark);
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        printError(spark);
        return;
      }
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        System.out.println(
            "spark " + spark.getDeviceId() + "is being a bitch and wont upload config");
        sparkStickyFault = true;
      }
    }
  }
}
