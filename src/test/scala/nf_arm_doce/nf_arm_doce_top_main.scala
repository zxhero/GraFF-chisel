package nf_arm_doce

import chisel3._

object TOPgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new zynq_deoi_overlay_x86_xgbe())
  println(verilogString)
}