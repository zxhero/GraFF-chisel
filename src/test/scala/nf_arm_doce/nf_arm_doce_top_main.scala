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

  val controls = Module(new controller(AXI_ADDR_WIDTH, 16+4,
    Map("fpga_id" -> 16, "Rlevel_lo_0" -> 17, "Rlevel_hi_0" -> 18,
      "Rlevel_lo_1" -> 19, "Rlevel_hi_1" -> 20,
      "Rlevel_lo_2" -> 21, "Rlevel_hi_2" -> 22,
      "Rlevel_lo_3" -> 23, "Rlevel_hi_3" -> 24,
      "packet_size" -> 25, "pending_time" -> 26)))

  val MemController = Module(new multi_port_mc(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1,
    AXI_SIZE_WIDTH, 16))
  val Applys = Seq.tabulate(16)(
    i => Module(new Scatter(4, i, 64, 4, 16))
  )
  val Gathers = Module(new Gather(64, 4))
  val LevelCache = Module(new Apply(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH, 4))
  val Scatters = Seq.tabulate(4)(
    i => Module(new Broadcast(64, 16, 6, 3,
      14, 5, i, 4))
  )
  val Broadcaster = Module(new Remote_xbar(16, 16, 4, 4))
  val ReApplys = Seq.tabulate(4)(
    x => Module(new Remote_Apply(64, 4, 4, x)
  ))
  val ReScatter = Module(new Remote_Scatter(AXI_ADDR_WIDTH, 64, 6, 3,
    64, 4))
  val ReSwitch = Module(new axis_to_axi(64, 64, 6, 3,
    64, 4))

  io.PLmemory <> MemController.io.ddr_out
  Gathers.io.ddr_in <> MemController.io.cacheable_out
  LevelCache.io.gather_in <> Gathers.io.gather_out(4)
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
  Broadcaster.io.remote_in <> ReScatter.io.xbar_out
  ReScatter.io.remote_in <> io.Re_memory_in
  ReApplys.zipWithIndex.map{
    case (r, i) => {
      r.io.xbar_in <> Broadcaster.io.remote_out(i)
      ReSwitch.io.xbar_in(i) <> r.io.remote_out
    }
  }
  io.Re_memory_out <> ReSwitch.io.remote_out

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
  controls.io.unvisited_size := MemController.io.unvisited_size + ReScatter.io.remote_unvisited_size
  controls.io.flush_cache_end := LevelCache.io.end
  controls.io.signal_ack := MemController.io.signal_ack && ReApplys.zipWithIndex.map{
    case(x,i) => x.io.signal_ack | (i.U === controls.GetRegByName("fpga_id"))
  }.reduce(_&_)
  controls.io.performance(0) := 0.U
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
      b.io.recv_sync := VecInit(Scatters.map{i => i.io.issue_sync}.+:(ReScatter.io.issue_sync)).asUInt()
    }
  }
  MemController.io.tiers_base_addr(0) := Cat(controls.io.data(9), controls.io.data(8))
  MemController.io.tiers_base_addr(1) := Cat(controls.io.data(11), controls.io.data(10))
  MemController.io.signal := controls.io.signal && controls.io.signal_ack
  MemController.io.start := controls.io.start
  MemController.io.end := controls.io.data(0)(1)
  ReApplys.zipWithIndex.map{
    case (r, i) => {
      controls.io.fin(16+i) := r.io.end | (i.U === controls.GetRegByName("fpga_id"))
      r.io.recv_sync := VecInit(Scatters.map{i => i.io.issue_sync}).asUInt()
      r.io.local_fpga_id := controls.GetRegByName("fpga_id")
      r.io.signal := controls.io.signal && controls.io.signal_ack
      r.io.local_unvisited_size := MemController.io.unvisited_size
      r.io.recv_sync_phase2 := ReScatter.io.issue_sync_phase2(i)
      r.io.packet_size := controls.GetRegByName("packet_size")
      r.io.level := controls.io.level
      r.io.pending_time := controls.GetRegByName("pending_time")
    }
  }
  Applys.map{x => x.io.local_fpga_id := controls.GetRegByName("fpga_id")}
  ReScatter.io.start := controls.io.start
  ReScatter.io.signal := controls.io.signal && controls.io.signal_ack
  ReScatter.io.local_fpga_id := controls.GetRegByName("fpga_id")
  ReSwitch.io.level_base_addr := VecInit(Seq.tabulate(4)(
    x => Cat(controls.GetRegByName("Rlevel_hi_"+x.toString),
      controls.GetRegByName("Rlevel_lo_"+x.toString))
  ))
  Broadcaster.io.local_fpga_id := controls.GetRegByName("fpga_id")
  Broadcaster.io.flush := Scatters.map{i => i.io.issue_sync}.reduce(_&_)
  Broadcaster.io.signal := controls.io.signal && controls.io.signal_ack
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