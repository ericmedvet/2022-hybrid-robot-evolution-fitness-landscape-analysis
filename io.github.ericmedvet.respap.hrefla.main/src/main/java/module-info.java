module io.github.ericmedvet.respap.hrefla.main {
  requires jcommander;
  requires java.logging;
  requires commons.csv;
  requires io.github.ericmedvet.jnb.core;
  requires io.github.ericmedvet.robotevo2d.main;
  requires io.github.ericmedvet.mrsim2d.core;
  requires io.github.ericmedvet.jgea.core;
  requires io.github.ericmedvet.jgea.experimenter;
  uses io.github.ericmedvet.mrsim2d.core.engine.Engine;
  exports io.github.ericmedvet.respap.hrefla to jcommander;
  opens io.github.ericmedvet.respap.hrefla to jcommander;
}