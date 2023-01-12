module it.units.erallab.respap.hrefla.main {
  requires jcommander;
  requires java.logging;
  requires commons.csv;
  //uses io.github.ericmedvet.mrsim2d.core.engine.Engine;
  //requires it.units.erallab.mrsim2d.core;
  //requires it.units.malelab.jgea.core;
  //requires it.units.erallab.mrsim2d.builder;
  //requires it.units.erallab.robotevo2d.main;
  requires io.github.ericmedvet.jnb.core;
  requires io.github.ericmedvet.robotevo2d.main;
  requires io.github.ericmedvet.mrsim2d.core;
  requires io.github.ericmedvet.jgea.core;
  uses io.github.ericmedvet.mrsim2d.core.engine.Engine;
  exports it.units.erallab.respap.hrefla to jcommander;
  opens it.units.erallab.respap.hrefla to jcommander;
}