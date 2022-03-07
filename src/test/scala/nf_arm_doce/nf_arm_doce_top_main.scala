package nf_arm_doce

import chisel3._
import numa.V2LLayer
import bfs._
import chisel3.util.Cat

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

class BFS_ps(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmemory = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH)))
    val PSmemory = Vec(4, Flipped(new axidata(AXI_ADDR_WIDTH, 16, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  })

  val controls = Module(new controller(AXI_ADDR_WIDTH, 16))

  val pl_mc = Module(new multi_port_mc(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH, 16))
  val Scatters = Seq.tabulate(16)(
    i => Module(new Scatter(4, i, AXI_DATA_WIDTH))
  )
  val Gathers = Module(new Gather(64, 4))
  val Applys = Module(new Apply(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH))
  val Broadcasts = Seq.tabulate(4)(
    i => Module(new Broadcast(64, 16, 6, 3,
      14, 4, i))
  )
  val bxbar = Module(new broadcast_xbar(16, 16, 4))

  io.PLmemory <> pl_mc.io.ddr_out
  Gathers.io.ddr_in <> pl_mc.io.cacheable_out
  Applys.io.gather_in <> Gathers.io.gather_out(4)
  Applys.io.ddr_w <> pl_mc.io.non_cacheable_in.w
  Applys.io.ddr_aw <> pl_mc.io.non_cacheable_in.aw
  Applys.io.ddr_b <> pl_mc.io.non_cacheable_in.b
  Broadcasts.zipWithIndex.map{
    case (b, i) => {
      b.io.gather_in <> Gathers.io.gather_out(i)
      b.io.ddr_r <> io.PSmemory(i).r
      b.io.ddr_ar <> io.PSmemory(i).ar
      bxbar.io.ddr_in(i) <> b.io.xbar_out
      b.io.recv_sync := VecInit(Broadcasts.map{i => i.io.issue_sync}).asUInt()
    }
  }
  Scatters.zipWithIndex.map{
    case (pe, i) => {
      pe.io.xbar_in <> bxbar.io.pe_out(i)
      pe.io.ddr_out <> pl_mc.io.cacheable_in(i)
      controls.io.fin(i) := pe.io.end
    }
  }

  //tie off unnecessary ports
  io.PSmemory.map{
    i => {
      i.b.ready := false.B
      i.aw.bits.tie_off(0.U)
      i.aw.valid := false.B
      i.w.bits.tie_off()
      i.w.valid := false.B
    }
  }
  pl_mc.io.non_cacheable_in.r.ready := false.B
  pl_mc.io.non_cacheable_in.ar.bits.tie_off((1.U << AXI_ID_WIDTH).asUInt())
  pl_mc.io.non_cacheable_in.ar.valid := false.B

  //control path
  controls.io.config <> io.config
  controls.io.traveled_edges := Broadcasts.map{i => i.io.traveled_edges}.reduce(_+_)
  controls.io.unvisited_size := pl_mc.io.unvisited_size
  controls.io.flush_cache_end := Applys.io.end
  Gathers.io.signal := controls.io.signal
  Applys.io.flush := controls.io.flush_cache
  Applys.io.level_base_addr := Cat(controls.io.data(6), controls.io.data(5))
  Applys.io.level := controls.io.level
  Broadcasts.zipWithIndex.map{
    case(b, i) => {
      b.io.root := controls.io.data(7)
      b.io.signal := controls.io.signal
      b.io.embedding_base_addr := Cat(controls.io.data(2), controls.io.data(1))
      b.io.edge_base_addr := Cat(controls.io.data(4), controls.io.data(3))
      if(i == 0){
        b.io.start := controls.io.start
      }else{
        b.io.start := false.B
      }
    }
  }
  pl_mc.io.tiers_base_addr(0) := Cat(controls.io.data(9), controls.io.data(8))
  pl_mc.io.tiers_base_addr(1) := Cat(controls.io.data(11), controls.io.data(10))
  pl_mc.io.signal := controls.io.signal
  pl_mc.io.start := controls.io.start
  pl_mc.io.end := controls.io.data(0)(1)
}

object BFSPSgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new BFS_ps(), args)
  //println(verilogString)
}