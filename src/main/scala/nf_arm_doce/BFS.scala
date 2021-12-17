package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce.{axidata, axilitedata, streamdata}
import chisel3.util.Decoupled
import chisel3.util._
import utils._
import numa.LookupTable

/*class BFS(AXI_ADDR_WIDTH : Int = 44, AXI_DATA_WIDTH: Int = 16, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmempory = Flipped(new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val PSmempory = Flipped(new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))

  })

  //1st entry for start register
  val controls = Module(new LookupTable(depth = 2, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  controls.config <> io.config
  val start = controls.io.data(0)(0)


}*/
class xbar(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 1, AXI_SIZE_WIDTH: Int = 3) extends BlackBox {
  val io = IO(new Bundle {
    val s_axi = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 2)
    val m_axi = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val aclk = Input(Bool())
    val aresetn = Input(Bool())
  })
}

class memory(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 8, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val data_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val count = Output(UInt(32.W))
    val base_addr = Input(UInt(64.W))
    //val data_in = (new axidata(64, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  })

  val xbar = Module(new xbar(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  xbar.io.aclk := clock.asBool()
  xbar.io.aresetn := ~reset.asBool()
  xbar.io.m_axi.rlast := io.data_out.rlast
  xbar.io.m_axi.rvalid := io.data_out.rvalid
  xbar.io.m_axi.rdata := io.data_out.rdata
  xbar.io.m_axi.rid := io.data_out.rid
  xbar.io.m_axi.rresp := io.data_out.rresp
  io.data_out.rready := xbar.io.m_axi.rready

  io.data_out.araddr := xbar.io.m_axi.araddr
  io.data_out.arvalid := xbar.io.m_axi.arvalid
  io.data_out.arid := xbar.io.m_axi.arid
  io.data_out.arsize := xbar.io.m_axi.arsize
  io.data_out.arlen := xbar.io.m_axi.arlen
  io.data_out.arlock := xbar.io.m_axi.arlock
  io.data_out.arburst := xbar.io.m_axi.arburst
  xbar.io.m_axi.arready := io.data_out.arready
  //read offset array
  val count = RegInit(0.U(32.W))
  val offset_arvalid = RegInit(false.B)
  when(offset_arvalid && xbar.io.s_axi.arready(1) && xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    count := count
  }.elsewhen(offset_arvalid && xbar.io.s_axi.arready(1)) {
    count := count + 1.U
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)) {
    count := count - 1.U
  }

  val full = (count >= 63.U)
  when(full.asBool()){
    offset_arvalid := false.B
  }.otherwise{
    offset_arvalid := true.B
  }

  io.count := count

  //read edge array
  val edge_count = RegInit(0.U(32.W))
  val edge_arvalid =  edge_count > 0.U
    when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && edge_arvalid && xbar.io.s_axi.arready(0)){
      edge_count := edge_count
    }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    edge_count := edge_count + 1.U
  }.elsewhen(edge_arvalid && xbar.io.s_axi.arready(0)) {
    edge_count := edge_count - 1.U
  }

  xbar.io.s_axi.arvalid := Cat(offset_arvalid & !full, edge_arvalid)
  xbar.io.s_axi.arid := Cat(count(AXI_ID_WIDTH-1, 0), edge_count(AXI_ID_WIDTH-1, 0))
  xbar.io.s_axi.araddr := Cat(io.base_addr, (io.base_addr + 0x10000.U))
  xbar.io.s_axi.arburst := Cat(1.U(2.W), 1.U(2.W))
  xbar.io.s_axi.arlen := Cat(0.U(8.W), 7.U(8.W))
  xbar.io.s_axi.arsize := Cat(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W), log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W))
  xbar.io.s_axi.arlock := 0.U(2.W)

  xbar.io.s_axi.rready := Cat(1.U(1.W), 1.U(1.W))

  //write value array
  val wcount = RegInit(0.U(32.W))
  when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && io.data_out.wvalid.asBool() && io.data_out.wready.asBool()){
    wcount := wcount
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    wcount := wcount + 1.U
  }.elsewhen(io.data_out.wvalid.asBool() && io.data_out.wready.asBool()) {
    wcount := wcount - 1.U
  }

  io.data_out.wvalid := wcount > 0.U
  io.data_out.wlast := 1.U
  io.data_out.wdata := 0.U
  io.data_out.wstrb := VecInit(Seq.fill(AXI_DATA_WIDTH)(1.U(1.W))).asUInt()

  val awcount = RegInit(0.U(32.W))
  when(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)
    && io.data_out.awvalid.asBool() && io.data_out.awready.asBool()){
    awcount := awcount
  }.elsewhen(xbar.io.s_axi.rvalid(1) && xbar.io.s_axi.rready(1) && xbar.io.s_axi.rlast(1)){
    awcount := awcount + 1.U
  }.elsewhen(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awcount := awcount - 1.U
  }

  val awaddr = RegInit(0.U(30.W))
  when(io.data_out.awvalid.asBool() && io.data_out.awready.asBool()) {
    awaddr := awaddr + 0x1000.U
  }

  io.data_out.awvalid := awcount > 0.U
  io.data_out.awid := awcount(AXI_ID_WIDTH-1, 0)
  io.data_out.awsize := log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  io.data_out.awlen := 0.U
  io.data_out.awburst := 1.U(2.W)
  io.data_out.awlock := 0.U
  io.data_out.awaddr := io.base_addr + awaddr

  io.data_out.bready := 1.U
}

class axiar(AXI_ADDR_WIDTH : Int = 44, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends Bundle {
  val araddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val arid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val arlen = Input(UInt((NUM * 8).W))
  val arsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val arburst = Input(UInt((NUM * 2).W))
  val arlock = Input(UInt((NUM * 1).W))
  val arvalid = Input(UInt((NUM * 1).W))
  val arready = Output(UInt((NUM * 1).W))
}

class axiaw(AXI_ADDR_WIDTH : Int = 44, AXI_ID_WIDTH: Int = 18, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends Bundle {
  val awaddr = Input(UInt((NUM * AXI_ADDR_WIDTH).W))
  val awid = Input(UInt((NUM * AXI_ID_WIDTH).W))
  val awlen = Input(UInt((NUM * 8).W))
  val awsize = Input(UInt((NUM * AXI_SIZE_WIDTH).W))
  val awburst = Input(UInt((NUM * 2).W))
  val awlock = Input(UInt((NUM * 1).W))
  val awvalid = Input(UInt((NUM * 1).W))
  val awready = Output(UInt((NUM * 1).W))
}

class axiw(AXI_DATA_WIDTH: Int = 16, NUM : Int = 1) extends Bundle {
  val wdata = Input(UInt((NUM*8*AXI_DATA_WIDTH).W))
  val wstrb = Input(UInt((NUM*AXI_DATA_WIDTH).W))
  val wlast = Input(UInt((NUM * 1).W))
  val wvalid = Input(UInt((NUM * 1).W))
  val wready = Output(UInt((NUM * 1).W))
}

class axib(AXI_ID_WIDTH: Int = 18, NUM : Int = 1) extends Bundle {
  val bresp = Output(UInt((NUM * 2).W))
  val bid = Output(UInt((NUM * AXI_ID_WIDTH).W))
  val bvalid = Output(UInt((NUM * 1).W))
  val bready = Input(UInt((NUM * 1).W))
}

class arbitator(AXI_ADDR_WIDTH : Int = 64, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3, NUM : Int = 1) extends  BlackBox {
  val io = IO(new Bundle() {
    val xbar_in = (new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, NUM))
    val ddr_out = Flipped(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
  })
}

class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 4, AXI_SIZE_WIDTH: Int = 3) extends BlackBox{
  val io = IO(new Bundle() {
    val ddr_aw = Flipped(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Flipped(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(new axib(AXI_ID_WIDTH, 1))

    val xbar_in = (new streamdata(8))
  })
}

class selector(AXI_DATA_WIDTH: Int = 64) extends BlackBox{
  val io = IO(new Bundle() {
    val xbar_in = (new streamdata(AXI_DATA_WIDTH))
    val ddr_out = Flipped(new streamdata(4))
  })
}

class embedding_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3, EMBEDDING : Int = 14) extends Module{
  val io = IO(new Bundle() {
    val ddr_out = Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    //val count = Output(UInt(32.W))
    val embedding_base_addr = Input(UInt(64.W))
    val edge_base_addr = Input(UInt(64.W))
    val level_base_addr = Input(UInt(64.W))
    val xbar_in = (new streamdata(64, 4))
    val xbar_out = Flipped(new streamdata(64, 4))
  })
  assert(AXI_DATA_WIDTH >= 8)
  assert(AXI_ID_WIDTH > log2Ceil(32))

  //front end of upward
  val vertex_in_buffer = Module(new fifo(32, 512 + 16))
  vertex_in_buffer.io.dataIn := Cat(io.xbar_in.tdata, io.xbar_in.tkeep)
  vertex_in_buffer.io.writeFlag := io.xbar_in.tvalid

  io.xbar_in.tready := vertex_in_buffer.io.full === false.B

  val Selector = Module(new selector(64))
  Selector.io.xbar_in.tdata := vertex_in_buffer.io.dataOut(512 + 16 - 1, 16)
  Selector.io.xbar_in.tvalid := vertex_in_buffer.io.empty === false.B
  Selector.io.xbar_in.tlast := true.B
  Selector.io.xbar_in.tkeep := vertex_in_buffer.io.dataOut(15, 0)
  vertex_in_buffer.io.readFlag := Selector.io.xbar_in.tready

  //back end of upward
  //read offset and edge embedding array
  val vertex_read_buffer = Module(new fifo(32, 32))
  vertex_read_buffer.io.writeFlag := Selector.io.ddr_out.tvalid
  vertex_read_buffer.io.dataIn := Selector.io.ddr_out.tdata


  //read edge array
  val edge_read_buffer = Module(new fifo(32, 64))
  edge_read_buffer.io.dataIn := io.ddr_out.rdata(8 * 8 - 1, 0)
  edge_read_buffer.io.writeFlag := !io.ddr_out.rid(AXI_ID_WIDTH - 1) & io.ddr_out.rvalid

  val ar_arbitator = Module(new arbitator(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 2))
  io.ddr_out.araddr := ar_arbitator.io.ddr_out.araddr
  io.ddr_out.arid := ar_arbitator.io.ddr_out.arid
  io.ddr_out.arvalid := ar_arbitator.io.ddr_out.arvalid
  io.ddr_out.arsize := ar_arbitator.io.ddr_out.arsize
  io.ddr_out.arlen := ar_arbitator.io.ddr_out.arlen
  io.ddr_out.arlock := ar_arbitator.io.ddr_out.arlock
  io.ddr_out.arburst := ar_arbitator.io.ddr_out.arburst
  ar_arbitator.io.ddr_out.arready := io.ddr_out.arready

  ar_arbitator.io.xbar_in.araddr:= Cat(io.edge_base_addr + edge_read_buffer.io.dataOut(63,32) << 2 ,
    io.embedding_base_addr + vertex_read_buffer.io.dataOut << log2Ceil(4 * EMBEDDING + 8))
  ar_arbitator.io.xbar_in.arvalid := Cat(edge_read_buffer.io.empty === false.B, vertex_read_buffer.io.empty === false.B)
  val remainning_edges = (edge_read_buffer.io.dataOut(31, 0) - EMBEDDING.asUInt())
  val arlen = Mux(((remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < remainning_edges,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() + 1.U,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)))
  ar_arbitator.io.xbar_in.arlen := Cat(arlen.asUInt() - 1.U, ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt())
  ar_arbitator.io.xbar_in.arburst := Cat(1.U(2.W), 1.U(2.W))
  ar_arbitator.io.xbar_in.arlock := 0.U
  ar_arbitator.io.xbar_in.arsize := Cat(log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W),
    log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W))
  ar_arbitator.io.xbar_in.arid := Cat((1.U(AXI_ID_WIDTH) << (AXI_ID_WIDTH - 1)).asUInt() + edge_read_buffer.io.rptr,
    vertex_read_buffer.io.rptr)
  vertex_read_buffer.io.readFlag := ar_arbitator.io.xbar_in.arready(0)
  edge_read_buffer.io.readFlag := ar_arbitator.io.xbar_in.arready(1)

  //front end of down ward
  val num_regfile = Module(new regFile(32, 32))
  num_regfile.io.writeFlag := ar_arbitator.io.xbar_in.arready(1) & ar_arbitator.io.xbar_in.arvalid(1)
  num_regfile.io.wptr := edge_read_buffer.io.rptr
  num_regfile.io.dataIn := remainning_edges
  num_regfile.io.rptr := io.ddr_out.rid(AXI_ID_WIDTH-2, 0)

  //back end of down ward
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x1.U) // i "load"  -> 000_0011
    val remainingBurst   = Value(0x2.U) // i "imm"   -> 001_0011
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && io.ddr_out.rvalid.asBool() && io.ddr_out.rready.asBool()){
    when(io.ddr_out.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num_regfile.io.dataOut
      status := sm.remainingBurst
    }
  }.elsewhen(status === sm.remainingBurst && io.ddr_out.rvalid.asBool() && io.ddr_out.rready.asBool()){
    when(io.ddr_out.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num - 16.U
      status := sm.remainingBurst
    }
  }.otherwise{
    num := num
    status := status
  }
  val keep = Wire(Vec(16, Bool()))
  keep.zipWithIndex.map {
    case(k, i) => k := Mux(io.ddr_out.rid(AXI_ID_WIDTH-1), Mux(status === sm.firstBurst, num_regfile.io.dataOut > i.U, num > i.U),
      Mux(i.U < 2.U, false.B, true.B))
  }

  val vertex_out_fifo = Module(new fifo(2, 512 + 16))
  io.xbar_out.tvalid := vertex_out_fifo.io.empty === false.B
  io.xbar_out.tkeep := vertex_out_fifo.io.dataOut(15, 0)
  io.xbar_out.tdata := vertex_out_fifo.io.dataOut(512 + 16 - 1, 16)
  io.xbar_out.tlast := true.B
  vertex_out_fifo.io.readFlag := io.xbar_out.tready
  vertex_out_fifo.io.writeFlag := io.ddr_out.rvalid
  vertex_out_fifo.io.dataIn := Cat(io.ddr_out.rdata, keep.asUInt())
  io.ddr_out.rready := vertex_out_fifo.io.full === false.B

  //write value array
  val vertex_update_buffer = Module(new fifo(32, 64))
  vertex_update_buffer.io.writeFlag := Selector.io.ddr_out.tvalid
  vertex_update_buffer.io.dataIn := Selector.io.ddr_out.tdata

  Selector.io.ddr_out.tready := vertex_read_buffer.io.full === false.B & vertex_update_buffer.io.full === false.B

  val update_engine = Module(new WB_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_DATA_WIDTH))
  update_engine.io.xbar_in.tdata := vertex_update_buffer.io.dataOut
  update_engine.io.xbar_in.tvalid := vertex_update_buffer.io.empty === false.B
  vertex_update_buffer.io.readFlag := update_engine.io.xbar_in.tready

  io.ddr_out.wvalid := update_engine.io.ddr_w.wvalid
  io.ddr_out.wlast := update_engine.io.ddr_w.wlast
  io.ddr_out.wdata := update_engine.io.ddr_w.wdata
  io.ddr_out.wstrb := update_engine.io.ddr_w.wstrb
  update_engine.io.ddr_w.wready := io.ddr_out.wready

  io.ddr_out.awvalid := update_engine.io.ddr_aw.awvalid
  io.ddr_out.awid := update_engine.io.ddr_aw.awid
  io.ddr_out.awsize := update_engine.io.ddr_aw.awsize
  io.ddr_out.awlen := update_engine.io.ddr_aw.awlen
  io.ddr_out.awburst := update_engine.io.ddr_aw.awburst
  io.ddr_out.awlock := update_engine.io.ddr_aw.awlock
  io.ddr_out.awaddr := update_engine.io.ddr_aw.awaddr
  update_engine.io.ddr_aw.awready := io.ddr_out.awready

  io.ddr_out.bready := update_engine.io.ddr_b.bready
  update_engine.io.ddr_b.bvalid := io.ddr_out.bvalid
  update_engine.io.ddr_b.bresp := io.ddr_out.bresp
  update_engine.io.ddr_b.bid := io.ddr_out.bid
}