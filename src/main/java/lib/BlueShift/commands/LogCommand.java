package lib.BlueShift.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class LogCommand extends Command {
  Supplier<String> logSupplier;

  public LogCommand(Supplier<String> logSupplier) {
    this.logSupplier = logSupplier;
  }

  public LogCommand(String toLog) {
    this.logSupplier = () -> toLog;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println(this.logSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
