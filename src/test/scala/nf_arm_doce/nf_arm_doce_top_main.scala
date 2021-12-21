package nf_arm_doce

import chisel3._
import numa.V2LLayer
import bfs._

object TOPgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new zynq_deoi_overlay_x86_xgbe())
  println(verilogString)
}

object V2Lgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new V2LLayer(64, 16, 5, 3), args)
  //println(verilogString)
}
//sbt 'test:runMain nf_arm_doce.V2Lgen -td ./generated/V2L'

object BFSgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new BFS(), args)
  //println(verilogString)
}