package nf_arm_doce

import chisel3._
import numa.V2LLayer
import bfs._
import chisel3.util.Cat
import doce.{aw_decode, aw_width_converter, rx_depacketing, tx_packeting}
import remote._

object TOPgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new zynq_deoi_overlay_x86_xgbe())
  println(verilogString)
}

object V2Lgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new V2LLayer(64, 16, 5, 3), args)
  //println(verilogString)
}
//sbt 'test:runMain nf_arm_doce.V2Lgen -td ./generated/V2L'

class BFS_ps(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmemory = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH)))
    val PSmemory = Vec(4, Flipped(new axidata(AXI_ADDR_WIDTH, 16, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
    val Re_memory_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val Re_memory_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 4, AXI_SIZE_WIDTH)
  })

  val controls = Module(new controller(AXI_ADDR_WIDTH, 16,
    Map("fpga_id" -> 16, "Rlevel_lo" -> 17, "Rlevel_hi" -> 18)))

  val MemController = Module(new multi_port_mc(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH, 16))
  val Applys = Seq.tabulate(16)(
    i => Module(new Scatter(4, i, 64, 1, 16))
  )
  val Gathers = Module(new Gather(64, 4))
  val LevelCache = Module(new Apply(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH, 1))
  val Scatters = Seq.tabulate(4)(
    i => Module(new Broadcast(64, 16, 6, 3,
      14, 4, i, 1))
  )
  val Broadcaster = Module(new broadcast_xbar(16, 16, 4))

  io.PLmemory <> MemController.io.ddr_out
  Gathers.io.ddr_in <> MemController.io.cacheable_out
  LevelCache.io.gather_in <> Gathers.io.level_cache_out
  LevelCache.io.ddr_w <> MemController.io.non_cacheable_in.w
  LevelCache.io.ddr_aw <> MemController.io.non_cacheable_in.aw
  LevelCache.io.ddr_b <> MemController.io.non_cacheable_in.b
  Scatters.zipWithIndex.map{
    case (b, i) => {
      b.io.gather_in <> Gathers.io.gather_out(i)
      b.io.ddr_r <> io.PSmemory(i).r
      b.io.ddr_ar <> io.PSmemory(i).ar
      Broadcaster.io.ddr_in(i) <> b.io.xbar_out
    }
  }
  Applys.zipWithIndex.map{
    case (pe, i) => {
      pe.io.xbar_in <> Broadcaster.io.pe_out(i)
      pe.io.ddr_out <> MemController.io.cacheable_in(i)
      controls.io.fin(i) := pe.io.end
    }
  }

  //tie off remote port
  io.Re_memory_in.r.bits.tie_off()
  io.Re_memory_in.r.valid := false.B
  io.Re_memory_in.b.bits.tie_off()
  io.Re_memory_in.b.valid := false.B
  io.Re_memory_in.ar.ready := true.B
  io.Re_memory_in.aw.ready := true.B
  io.Re_memory_in.w.ready := true.B
  io.Re_memory_out.aw.bits.tie_off(0.U)
  io.Re_memory_out.aw.valid := false.B
  io.Re_memory_out.ar.bits.tie_off(0.U)
  io.Re_memory_out.ar.valid := false.B
  io.Re_memory_out.w.bits.tie_off()
  io.Re_memory_out.w.valid := false.B
  io.Re_memory_out.r.ready := true.B
  io.Re_memory_out.b.ready := true.B

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
  MemController.io.non_cacheable_in.r.ready := false.B
  MemController.io.non_cacheable_in.ar.bits.tie_off((1.U << AXI_ID_WIDTH).asUInt())
  MemController.io.non_cacheable_in.ar.valid := false.B

  //control path
  controls.io.config <> io.config
  controls.io.traveled_edges := Scatters.map{i => i.io.traveled_edges}.reduce(_+_)
  controls.io.unvisited_size := MemController.io.unvisited_size
  controls.io.flush_cache_end := LevelCache.io.end
  controls.io.signal_ack := MemController.io.signal_ack
  //controls.io.fin.last := ReApply.io.end
  controls.io.performance(0) := false.B
  LevelCache.io.flush := controls.io.flush_cache
  LevelCache.io.level_base_addr := Cat(controls.io.data(6), controls.io.data(5))
  LevelCache.io.level := controls.io.level
  Scatters.zipWithIndex.map{
    case(b, i) => {
      b.io.root := controls.io.data(7)
      b.io.signal := (controls.io.signal && controls.io.signal_ack) | controls.io.start
      b.io.embedding_base_addr := Cat(controls.io.data(2), controls.io.data(1))
      b.io.edge_base_addr := Cat(controls.io.data(4), controls.io.data(3))
      if(i == 0){
        b.io.start := controls.io.start
      }else{
        b.io.start := false.B
      }
      b.io.recv_sync := VecInit(Scatters.map{i => i.io.issue_sync}).asUInt()
    }
  }
  MemController.io.tiers_base_addr(0) := Cat(controls.io.data(9), controls.io.data(8))
  MemController.io.tiers_base_addr(1) := Cat(controls.io.data(11), controls.io.data(10))
  MemController.io.signal := controls.io.signal && controls.io.signal_ack
  MemController.io.start := controls.io.start
  MemController.io.end := controls.io.data(0)(1)
  Applys.map{x => x.io.local_fpga_id := controls.GetRegByName("fpga_id")}
}

object BFSPSgen extends App {
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new BFS_ps(), args)
  //println(verilogString)
}

object DoCEgen extends App {
  val rootdir = args(1)
  args(1) = rootdir + "transaction_layer/"
  print(args(1))
  val verilogString = (new chisel3.stage.ChiselStage).emitVerilog(new aw_width_converter(W_Channel_Width=512+64, Dout_Width=512), args)
  val verilogString2 = (new chisel3.stage.ChiselStage).emitVerilog(new aw_decode(W_Channel_Width=512+64, Din_Width=512), args)
  //println(verilogString)
  args(1) = rootdir + "transport_layer/"
  print(args(1))
  val verilogString3 = (new chisel3.stage.ChiselStage).emitVerilog(new tx_packeting(AXIS_DATA_WIDTH=64), args)
  val verilogString4 = (new chisel3.stage.ChiselStage).emitVerilog(new rx_depacketing(AXIS_DATA_WIDTH=64), args)
}