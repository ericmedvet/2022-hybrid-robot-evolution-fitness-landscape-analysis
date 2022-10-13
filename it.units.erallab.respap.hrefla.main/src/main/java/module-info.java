module it.units.erallab.respap.hrefla.main {
  requires jcommander;
  requires java.logging;
  requires commons.csv;
  uses it.units.erallab.mrsim2d.core.engine.Engine;
  requires it.units.erallab.mrsim2d.core;
  requires it.units.erallab.mrsim2d.builder;
  requires it.units.erallab.robotevo2d.main;
  exports it.units.erallab.respap.hrefla to jcommander;
  opens it.units.erallab.respap.hrefla to jcommander;
}