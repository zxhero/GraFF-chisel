package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce._
import chisel3.util._
import utils._
import numa.LookupTable

class URAM(size : Int = 1024, width : Int = 32) extends BlackBox{
  val io = IO(new Bundle() {
    val addra = Input(UInt(log2Ceil(size).W))
    val clka = Input(Bool())
    val dina = Input(UInt(width.W))
    val douta = Input(UInt(width.W))
    val ena = Input(Bool())
    val wea = Input(Bool())
    val addrb = Input(UInt(log2Ceil(size).W))
    val clkb = Input(Bool())
    val dinb = Input(UInt(width.W))
    val doutb = Input(UInt(width.W))
    val enb = Input(Bool())
    val web = Input(Bool())
  })
}

class BRAM(size : Int = 1024 * 1024, width : Int = 1) extends BlackBox{
  val io = IO(new Bundle() {
    val addra = Input(UInt(log2Ceil(size).W))
    val clka = Input(Bool())
    val dina = Input(UInt(width.W))
    val douta = Input(UInt(width.W))
    val ena = Input(Bool())
    val wea = Input(Bool())
    val addrb = Input(UInt(log2Ceil(size).W))
    val clkb = Input(Bool())
    val dinb = Input(UInt(width.W))
    val doutb = Input(UInt(width.W))
    val enb = Input(Bool())
    val web = Input(Bool())
  })
}

class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))

    val xbar_in = (new streamdata(4))
    val level_base_addr = Input(UInt(64.W))
    val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val level = Input(UInt(32.W))
  })

  val buffer = Module(new URAM(512 * 1024,32))

  //update buffer
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val read_size  = Value(0x1.U) // i "load"  -> 000_0011
    val write_addr   = Value(0x2.U) // i "imm"   -> 001_0011
    val write_level = Value(0x3.U)
    val write_size = Value(0x4.U)
    val read_addr = Value(0x5.U)

    val read_level1 = Value(0x6.U)
    val wb_level1 = Value(0x7.U)
    val read_level2 = Value(0x8.U)
    val wb_level2 = Value(0x9.U)
  }
  val update_sm = RegInit(sm.read_size)
  val wb_sm = RegInit(sm.idole)
  val vid = RegInit(0.U(32.W))
  val level = RegInit(0.U(32.W))
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    vid := io.xbar_in.tdata
    level := io.level
  }
  io.xbar_in.tready := update_sm === sm.read_size & (wb_sm === sm.idole)
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    update_sm := sm.read_addr
  }.elsewhen(update_sm === sm.read_addr){
    update_sm := sm.write_addr
  }.elsewhen(update_sm === sm.write_addr){
    update_sm := sm.write_level
  }.elsewhen(update_sm === sm.write_level){
    update_sm := sm.write_size
  }.elsewhen(update_sm === sm.write_size){
    update_sm := sm.read_size
  }
  val addr = Wire(UInt(16.W))
  addr := vid(11, 0) << 2.U
  val block_addr = vid(25 - 2, 14 - 2) << 7
  val size_addr = 0.U(7.W)
  val size = RegInit(0.U(32.W))
  val addr_addr = 2.U + size >> 1.U
  val level_addr = (2 + 42).U + size
  when(update_sm === sm.read_size) {
    size := buffer.io.douta
  }
  val old_addr_pair = RegInit(0.U(32.W))
  when(update_sm === sm.read_addr) {
    old_addr_pair := buffer.io.douta
  }
  val new_addr_pair = Mux(size(0), Cat(addr, old_addr_pair(15, 0)), Cat(old_addr_pair(31, 16), addr))
  buffer.io.ena := (io.xbar_in.tvalid & io.xbar_in.tready) | (update_sm === sm.read_addr) | (update_sm === sm.write_addr) |
    (update_sm === sm.write_level) | (update_sm === sm.write_size)
  buffer.io.wea := (update_sm === sm.write_addr) | (update_sm === sm.write_level) | (update_sm === sm.write_size)
  buffer.io.addra := Mux1H(Seq(
    (update_sm === sm.read_size) -> (block_addr.asUInt() + size_addr),
    (update_sm === sm.read_addr) -> (block_addr.asUInt() + addr_addr),
    (update_sm === sm.write_addr) -> (block_addr.asUInt() + addr_addr),
    (update_sm === sm.write_level) -> (block_addr.asUInt() + level_addr),
    (update_sm === sm.write_size) -> (block_addr.asUInt() + size_addr)
  ))
  buffer.io.dina := Mux1H(Seq(
    (update_sm === sm.write_addr) -> (new_addr_pair),
    (update_sm === sm.write_level) -> (level),
    (update_sm === sm.write_size) -> (size + 1.U)
  ))
  buffer.io.clka := clock.asBool()

  //write back buffer
  val count = RegInit(0.U(8.W))
  val aw_buffer = Module(new fifo(32, AXI_ADDR_WIDTH))
  val w_buffer = Module(new fifo(32, 32))
  val wb_block_addr = RegInit(0.U(19.W))
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    wb_block_addr := block_addr
  }
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    wb_sm := sm.read_addr
  }.elsewhen(wb_sm === sm.read_addr) {
    wb_sm := sm.read_level1
  }.elsewhen(wb_sm === sm.read_level1) {
    wb_sm := sm.wb_level1
  }.elsewhen(wb_sm === sm.wb_level1 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    wb_sm := sm.read_level2
  }.elsewhen(wb_sm === sm.read_level2) {
    wb_sm := sm.wb_level2
  }.elsewhen(wb_sm === sm.wb_level2 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    when(count === 83.U){
      wb_sm := sm.write_size
    }.otherwise{
      wb_sm := sm.read_addr
    }
  }.elsewhen(wb_sm === sm.write_size) {
    wb_sm := sm.idole
  }
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    count := 0.U
  }.elsewhen(wb_sm === sm.wb_level1 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B){
    count := count + 1.U
  }.elsewhen(wb_sm === sm.wb_level2 && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    count := count + 1.U
  }

  val wb_addr_pair = RegInit(0.U(32.W))
  val wb_level = RegInit(0.U(32.W))
  buffer.io.enb := (wb_sm === sm.read_addr) | (wb_sm === sm.read_level1) | (wb_sm === sm.read_level2) |
    (wb_sm === sm.write_size)
  buffer.io.web := (wb_sm === sm.write_size)
  buffer.io.addrb := Mux1H(Seq(
    (update_sm === sm.read_addr) -> (wb_block_addr.asUInt() + 2.U + (count >> 1.U)),
    (update_sm === sm.read_level1) -> (wb_block_addr.asUInt() + (2 + 42).U + count),
    (update_sm === sm.read_level2) -> (wb_block_addr.asUInt() + (2 + 42).U + count),
    (update_sm === sm.write_size) -> (wb_block_addr.asUInt() + size_addr)
  ))
  buffer.io.dinb := 0.U
  buffer.io.clkb := clock.asBool()
  when(wb_sm === sm.read_addr){
    wb_addr_pair := buffer.io.doutb
  }
  when(wb_sm === sm.read_level1 || wb_sm === sm.read_level2){
    wb_level := buffer.io.doutb
  }

  aw_buffer.io.writeFlag := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  aw_buffer.io.dataIn := Mux(wb_sm === sm.wb_level1, io.level_base_addr + Cat(wb_block_addr(18, 7), wb_addr_pair(13,0)).asUInt(),
    io.level_base_addr + Cat(wb_block_addr(18, 7), wb_addr_pair(16 + 14 - 1,16)))
  io.ddr_aw.bits.awaddr := aw_buffer.io.dataOut
  io.ddr_aw.bits.awlock := 0.U
  io.ddr_aw.bits.awid := aw_buffer.io.rptr
  io.ddr_aw.bits.awlen := 0.U
  io.ddr_aw.bits.awburst := 1.U(2.W)
  io.ddr_aw.bits.awsize := 2.U
  io.ddr_aw.valid := aw_buffer.io.empty === false.B
  aw_buffer.io.readFlag := io.ddr_aw.ready

  w_buffer.io.writeFlag := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  w_buffer.io.dataIn := wb_level
  io.ddr_w.bits.wdata := w_buffer.io.dataOut
  io.ddr_w.bits.wlast := true.B
  io.ddr_w.valid := w_buffer.io.empty === false.B
  io.ddr_w.bits.wstrb := 0xf.U
  w_buffer.io.readFlag := io.ddr_w.ready

  io.ddr_b.ready := true.B
}

class Apply(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle(){
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))
    val gather_in = (new streamdata(4))

    //control path
    val level = Input(UInt(32.W))
    val level_base_addr = Input(UInt(64.W))
    val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val signal = Input(Bool())    //used in prefetch mode
  })

  //write value array
  val vertex_update_buffer = Module(new fifo(32, 64))
  val FIN = io.gather_in.tdata(31) & io.gather_in.tvalid
  vertex_update_buffer.io.writeFlag := io.gather_in.tvalid & ~FIN
  vertex_update_buffer.io.dataIn := Cat(io.gather_in.tdata, io.level)

  io.gather_in.tready := vertex_update_buffer.io.full === false.B

  val update_engine = Module(new WB_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_DATA_WIDTH))
  update_engine.io.xbar_in.tdata := vertex_update_buffer.io.dataOut
  update_engine.io.xbar_in.tvalid := vertex_update_buffer.io.empty === false.B
  update_engine.io.level_base_addr := io.level_base_addr
  update_engine.io.level_size := io.level_size
  update_engine.io.xbar_in.tlast := true.B
  update_engine.io.xbar_in.tkeep := 0xff.U
  update_engine.io.level := io.level
  vertex_update_buffer.io.readFlag := update_engine.io.xbar_in.tready

  io.ddr_aw <> update_engine.io.ddr_aw
  io.ddr_w <> update_engine.io.ddr_w
  io.ddr_b <> update_engine.io.ddr_b
}

class axis_arbitrator(AXIS_DATA_WIDTH: Int = 4, NUM : Int = 16, ELEMENT_WIDTH: Int = 4) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = (new streamdata(AXIS_DATA_WIDTH * NUM, ELEMENT_WIDTH))
    val ddr_out = Flipped(new streamdata(AXIS_DATA_WIDTH, ELEMENT_WIDTH))
  })

  val data = RegInit(0.U((AXIS_DATA_WIDTH * NUM * 8).W))
  val keep = RegInit(0.U((AXIS_DATA_WIDTH * NUM / ELEMENT_WIDTH).W))

  when(io.xbar_in.tvalid && io.xbar_in.tready){
    data := io.xbar_in.tdata
  }
  io.xbar_in.tready := ~(keep.orR())

  val select = MuxCase(0.U,
    Array.tabulate(NUM)(x => (keep((x+1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH)).orR() -> (1.U(NUM.W) << x).asUInt())))
  io.ddr_out.tvalid := select.orR()
  io.ddr_out.tkeep := Mux1H(
    Seq.tabulate(NUM)(x => (select(x) -> keep((x + 1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH))))
  )
  io.ddr_out.tlast := true.B
  io.ddr_out.tdata :=
    Mux1H(Seq.tabulate(NUM)(x => (select(x) -> data((x + 1) * AXIS_DATA_WIDTH * 8 - 1, x * AXIS_DATA_WIDTH * 8))))

  val next_keep = Wire(Vec(NUM, UInt((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  next_keep.zipWithIndex.map{
    case(k, i) => (k := Mux(select(i), 0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W), ~0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  }
  when(io.xbar_in.tvalid && io.xbar_in.tready){
    keep := io.xbar_in.tkeep
  }.elsewhen(io.ddr_out.tvalid && io.ddr_out.tready) {
    keep := keep & next_keep.asUInt()
  }
}

class Gather(AXI_DATA_WIDTH: Int = 64) extends Module{
  val io = IO(new Bundle() {
    val ddr_in = (new streamdata(AXI_DATA_WIDTH, 4))
    val gather_out = Flipped(new streamdata(4, 4))

    //control path
    val signal = Input(Bool())
  })

  //front end of upward
  val vertex_in_buffer = Module(new fifo(2, 512 + 16))
  vertex_in_buffer.io.dataIn := Cat(io.ddr_in.tdata, io.ddr_in.tkeep)
  vertex_in_buffer.io.writeFlag := io.ddr_in.tvalid
  io.ddr_in.tready := vertex_in_buffer.io.full === false.B

  val Selector = Module(new axis_arbitrator(4, 16, 4))
  Selector.io.xbar_in.tdata := vertex_in_buffer.io.dataOut(512 + 16 - 1, 16)
  Selector.io.xbar_in.tvalid := vertex_in_buffer.io.empty === false.B
  Selector.io.xbar_in.tlast := true.B
  Selector.io.xbar_in.tkeep := vertex_in_buffer.io.dataOut(15, 0)
  vertex_in_buffer.io.readFlag := Selector.io.xbar_in.tready
  io.gather_out <> Selector.io.ddr_out
}

class Broadcast(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3, EMBEDDING : Int = 14) extends Module{
  val io = IO(new Bundle() {
    val ddr_ar = Decoupled(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val ddr_r = Flipped(Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH)))
    val gather_in = (new streamdata(4, 4))
    val xbar_out = Flipped(new streamdata(64, 4))

    //control path
    val embedding_base_addr = Input(UInt(64.W))
    val edge_base_addr = Input(UInt(64.W))
    val signal = Input(Bool())
    val traveled_edges = Output(UInt(64.W))       //every super step
  })
  assert(AXI_DATA_WIDTH >= 8)
  assert(AXI_ID_WIDTH > log2Ceil(32))

  //upward
  object upward_sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val exe  = Value(0x1.U) // i "load"  -> 000_0011
    val fin   = Value(0x2.U) // i "imm"   -> 001_0011
    val output_fin = Value(0x3.U)
  }
  val upward_status = RegInit(upward_sm.idole)
  val inflight_vtxs = RegInit(0.U(64.W))
  val traveled_edges_reg = RegInit(0.U(64.W))
  io.traveled_edges := traveled_edges_reg

  //read offset and edge embedding array
  val vertex_read_buffer = Module(new fifo(32, 32))
  vertex_read_buffer.io.writeFlag := io.gather_in.tvalid
  vertex_read_buffer.io.dataIn := io.gather_in.tdata
  io.gather_in.tready := vertex_read_buffer.io.full === false.B

  //read edge array
  val edge_read_buffer = Module(new fifo(32, 64))
  edge_read_buffer.io.dataIn := io.ddr_r.bits.rdata(8 * 8 - 1, 0)
  edge_read_buffer.io.writeFlag := !io.ddr_r.bits.rid(AXI_ID_WIDTH - 1) & io.ddr_r.valid & (io.ddr_r.bits.rdata(31, 0) > EMBEDDING.asUInt())

  val arbi = Module(new Arbiter(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1), 2))
  io.ddr_ar <> arbi.io.out

  arbi.io.in(0).bits.araddr := io.edge_base_addr + edge_read_buffer.io.dataOut(63,32) << 2
  arbi.io.in(1).bits.araddr :=  io.embedding_base_addr + vertex_read_buffer.io.dataOut << log2Ceil(4 * EMBEDDING + 8)
  arbi.io.in(0).valid := edge_read_buffer.io.empty === false.B
  arbi.io.in(1).valid :=  vertex_read_buffer.io.empty === false.B && vertex_read_buffer.io.dataOut(31) === 0.U
  val remainning_edges = (edge_read_buffer.io.dataOut(31, 0) - EMBEDDING.asUInt())
  //we assume number of edges of a vertex is less than 4096 + 14 for now. TODO
  val arlen = Mux(((remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < remainning_edges,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() + 1.U,
    (remainning_edges >> log2Ceil(AXI_DATA_WIDTH/4)))
  arbi.io.in(0).bits.arlen := arlen.asUInt() - 1.U
  arbi.io.in(1).bits.arlen := ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt()
  arbi.io.in(0).bits.arburst := 1.U(2.W)
  arbi.io.in(1).bits.arburst := 1.U(2.W)
  arbi.io.in(0).bits.arlock := 0.U
  arbi.io.in(1).bits.arlock := 0.U
  arbi.io.in(0).bits.arsize := log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  arbi.io.in(1).bits.arsize :=  log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  arbi.io.in(0).bits.arid := (1.U(AXI_ID_WIDTH) << (AXI_ID_WIDTH - 1)).asUInt() + edge_read_buffer.io.rptr
  arbi.io.in(1).bits.arid :=  vertex_read_buffer.io.rptr
  vertex_read_buffer.io.readFlag := arbi.io.in(1).ready
  edge_read_buffer.io.readFlag := arbi.io.in(0).ready

  //front end of down ward
  val num_regfile = Module(new regFile(32, 32))
  num_regfile.io.writeFlag := arbi.io.in(0).ready & arbi.io.in(0).valid
  num_regfile.io.wptr := edge_read_buffer.io.rptr
  num_regfile.io.dataIn := remainning_edges
  num_regfile.io.rptr := io.ddr_r.bits.rid(AXI_ID_WIDTH-2, 0)

  //back end of down ward
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x1.U) // i "load"  -> 000_0011
    val remainingBurst   = Value(0x2.U) // i "imm"   -> 001_0011
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool()){
    when(io.ddr_r.bits.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num_regfile.io.dataOut
      status := sm.remainingBurst
    }
  }.elsewhen(status === sm.remainingBurst && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool()){
    when(io.ddr_r.bits.rlast.asBool()){
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
    case(k, i) => k := Mux(io.ddr_r.bits.rid(AXI_ID_WIDTH-1), Mux(status === sm.firstBurst, num_regfile.io.dataOut > i.U, num > i.U),
      Mux(i.U < 2.U, false.B, true.B))
  }

  val vertex_out_fifo = Module(new fifo(2, 512 + 16))
  io.xbar_out.tvalid := vertex_out_fifo.io.empty === false.B
  io.xbar_out.tkeep := vertex_out_fifo.io.dataOut(15, 0)
  io.xbar_out.tdata := vertex_out_fifo.io.dataOut(512 + 16 - 1, 16)
  io.xbar_out.tlast := true.B
  vertex_out_fifo.io.readFlag := io.xbar_out.tready
  vertex_out_fifo.io.writeFlag := io.ddr_r.valid | upward_status === upward_sm.output_fin
  vertex_out_fifo.io.dataIn := Mux(upward_status === upward_sm.output_fin, Cat("x80000000".asUInt(512.W), 0x1.U(16.W)), Cat(io.ddr_r.bits.rdata, keep.asUInt()))
  io.ddr_r.ready := vertex_out_fifo.io.full === false.B & edge_read_buffer.io.full === false.B

  //control path
  when(io.signal && (upward_status === upward_sm.idole)){
    upward_status := upward_sm.exe
  }.elsewhen(upward_status === upward_sm.exe && vertex_read_buffer.io.dataOut(31) === 1.U && vertex_read_buffer.io.empty === false.B){
    upward_status := upward_sm.fin
  }.elsewhen(upward_status === upward_sm.fin && inflight_vtxs === 0.U){
    upward_status := upward_sm.output_fin
  }.elsewhen(upward_status === upward_sm.output_fin && vertex_out_fifo.io.full === false.B){
    upward_status := upward_sm.idole
  }

  when(io.signal){
    traveled_edges_reg := 0.U
  }.elsewhen(!io.ddr_r.bits.rid(AXI_ID_WIDTH - 1) & io.ddr_r.valid.asBool() & io.ddr_r.ready.asBool()){
    traveled_edges_reg := traveled_edges_reg + io.ddr_r.bits.rdata(31, 0)
  }

  when(vertex_read_buffer.io.empty === false.B && vertex_read_buffer.io.dataOut(31) === 0.U
    && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool() && !edge_read_buffer.io.writeFlag){
    inflight_vtxs := inflight_vtxs
  }.elsewhen(vertex_read_buffer.io.empty === false.B && vertex_read_buffer.io.dataOut(31) === 0.U){
    inflight_vtxs := inflight_vtxs + 1.U
  }.elsewhen(io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool() && !edge_read_buffer.io.writeFlag){
    inflight_vtxs := inflight_vtxs - 1.U
  }
}

/*
* PE ID is global addressable
* MC ID is local addressable
* */
class broadcast_xbar(AXIS_DATA_WIDTH: Int = 64, SLAVE_NUM: Int, MASTER_NUM: Int) extends Module {
  val io = IO(new Bundle {
    val ddr_in = Vec(MASTER_NUM, new streamdata(AXIS_DATA_WIDTH, 4))
    //val ddr_out = Flipped(Vec(MASTER_NUM, (new streamdata(AXIS_DATA_WIDTH, 4))))
    val pe_out = Flipped(Vec(SLAVE_NUM, (new streamdata(AXIS_DATA_WIDTH, 4))))
    //val pe_in = Vec(SLAVE_NUM, (new streamdata(4, 4)))
  })
  //ddr_in to pe_out direction
  val arbitrator = Module(new axis_arbitrator(AXIS_DATA_WIDTH, MASTER_NUM, 4))
  val ddr_data = Wire(Vec(MASTER_NUM, UInt((8 * AXIS_DATA_WIDTH).W)))
  val ddr_keep = Wire(Vec(MASTER_NUM, UInt((AXIS_DATA_WIDTH / 4).W)))
  val ddr_tvalid = Wire(Vec(MASTER_NUM, Bool()))
  io.ddr_in.zipWithIndex.map{
    case(ddr, i) => {
      ddr_tvalid(i) := ddr.tvalid
      ddr_keep(i) := Mux(ddr.tvalid, ddr.tkeep, 0.U)
      ddr_data(i) := ddr.tdata
      ddr.tready := arbitrator.io.xbar_in.tready
    }
  }
  arbitrator.io.xbar_in.tkeep := ddr_keep.asUInt()
  arbitrator.io.xbar_in.tdata := ddr_data.asUInt()
  arbitrator.io.xbar_in.tlast := true.B
  arbitrator.io.xbar_in.tvalid := ddr_tvalid.asUInt().orR()

  val PES_tready = Wire(Vec(SLAVE_NUM, Bool()))
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      pe.tvalid := arbitrator.io.ddr_out.tvalid
      pe.tkeep := arbitrator.io.ddr_out.tkeep
      pe.tdata := arbitrator.io.ddr_out.tdata
      pe.tlast := arbitrator.io.ddr_out.tlast
      PES_tready(i) := pe.tready
    }
  }
  arbitrator.io.ddr_out.tready := PES_tready.asUInt().andR()
}

/*
* end: there is no more vertexes for the level
* */
class Scatter(AXIS_DATA_WIDTH: Int = 4, SID: Int) extends Module {
  val io = IO(new Bundle() {
    val xbar_in = new streamdata(64, 4)
    val ddr_out = Flipped(new streamdata(AXIS_DATA_WIDTH, 4))

    //control path
    val end = Output(Bool())
  })

  val bitmap = Module(new BRAM(1024 * 1024, 1))
  val arbi = Module(new axis_arbitrator(4, 16, 4))

  def vid_to_sid(vid: UInt, sid: UInt) : Bool = {
    vid(31, 20) === sid
  }

  val filtered_keep = Wire(Vec(16, Bool()))
  filtered_keep.zipWithIndex.map{
    case(k, i) => {
      k := Mux(io.xbar_in.get_ith_data(i)(31), true.B, io.xbar_in.tkeep(i) & vid_to_sid(io.xbar_in.get_ith_data(i), SID.asUInt()))
    }
  }

  arbi.io.xbar_in.tkeep := filtered_keep.asUInt()
  arbi.io.xbar_in.tdata := io.xbar_in.tdata
  arbi.io.xbar_in.tvalid := io.xbar_in.tvalid
  arbi.io.xbar_in.tlast := io.xbar_in.tlast
  io.xbar_in.tready := arbi.io.xbar_in.tready

  val vertex_in_fifo = Module(new fifo(32, 32))
  val vertex_out_fifo = Module(new fifo(32, 32))
  val wait_bitmap_fifo = Module(new fifo(1, 32))
  //read or write bitmap ehnr this is not FIN
  val bitmap_arvalid = vertex_in_fifo.io.empty === false.B & vertex_in_fifo.test_FIN() === false.B
  val bitmap_rready = wait_bitmap_fifo.io.full === false.B
  val bitmap_wvalid = wait_bitmap_fifo.io.empty === false.B & bitmap.io.douta === 0.U & wait_bitmap_fifo.test_FIN() === false.B

  vertex_in_fifo.io.dataIn := arbi.io.ddr_out.tdata
  vertex_in_fifo.io.writeFlag := arbi.io.ddr_out.tvalid
  arbi.io.ddr_out.tready := vertex_in_fifo.io.full === false.B
  bitmap.io.ena :=  bitmap_arvalid
  bitmap.io.addra := vertex_in_fifo.io.dataOut(19, 0)
  bitmap.io.clka := clock.asBool()
  vertex_in_fifo.io.readFlag := bitmap_rready
  wait_bitmap_fifo.io.writeFlag := vertex_in_fifo.io.empty === false.B
  wait_bitmap_fifo.io.dataIn := vertex_in_fifo.io.dataOut
  wait_bitmap_fifo.io.readFlag := vertex_out_fifo.io.full === false.B
  vertex_out_fifo.io.writeFlag := bitmap_wvalid | (wait_bitmap_fifo.test_FIN())
  vertex_out_fifo.io.dataIn := wait_bitmap_fifo.io.dataOut
  bitmap.io.enb := bitmap_wvalid
  bitmap.io.web := bitmap_wvalid
  bitmap.io.clkb := clock.asBool()
  bitmap.io.dinb := 1.U
  bitmap.io.addrb := wait_bitmap_fifo.io.dataOut(19, 0)
  vertex_out_fifo.io.readFlag := io.ddr_out.tready | vertex_out_fifo.test_FIN()
  io.ddr_out.tvalid := vertex_out_fifo.io.empty === false.B & vertex_out_fifo.test_FIN() === false.B
  io.ddr_out.tkeep := true.B
  io.ddr_out.tlast := true.B
  io.ddr_out.tdata := vertex_out_fifo.io.dataOut

  //control path
  io.end := vertex_out_fifo.test_FIN()
}

class Collector(AXI_DATA_WIDTH: Int = 64) extends Module{
  val io = IO(new Bundle() {
    val in = Vec(16, (new streamdata(4, 4)))
    val out = Flipped(new streamdata(AXI_DATA_WIDTH, 4))
    val flush = Input(Bool())
  })

  val collector_fifos = Seq.tabulate(16)(
    i => Module(new BRAM_fifo(4, 32, ("collector_fifo" + i.toString)))
  )
  val counter = RegInit(0.U(log2Ceil(16).W))
  val collector_data = Wire(Vec(16, UInt(32.W)))
  val sorted_data = Wire(Vec(16, UInt(32.W)))
  val sorted_valid = Wire(Vec(16, Bool()))
  val steps = (Seq.tabulate(16)(
    i => {
      Seq.tabulate(i + 1)(x => io.in(x).tvalid.asTypeOf(UInt(4.W))).reduce(_+_)
    }
  ))

  def indexAdd(index : UInt, step : UInt) : UInt = {
    Mux(index + step >= 16.U, index + step - 16.U, index + step)
  }
  def indexSub(index1 : UInt, index2 : UInt) : UInt = {
    Mux(index1 <= index2, index2 - index1, 16.U - index1 + index2)
  }

  io.in.zipWithIndex.map{
    case(in, i) => {
      in.tready := Seq.tabulate(16)(i => collector_fifos(i).io.full === false.B).reduce(_&_)
    }
  }
  sorted_data.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(0.U,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, io.in(x).tdata)
        ))
    }
  }
  sorted_valid.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(false.B,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, io.in(x).tvalid)
        ))
    }
  }

  when(sorted_valid.reduce(_|_)){
    counter := indexAdd(counter, MuxCase(0.U, Seq.tabulate(16)(
      x => (io.in(16 - x - 1).tready && sorted_valid(16 - x - 1), (x + 1).U)
    )))
  }

  collector_fifos.zipWithIndex.map{
    case (f, i) => {
      val fifo_in_data = Mux1H(Seq.tabulate(16)(x => (indexSub(counter, i.U) === x.U, sorted_data(x))))
      val fifo_in_valid = Mux1H(Seq.tabulate(16)(x => (indexSub(counter, i.U) === x.U, sorted_valid(x))))
      f.io.din := fifo_in_data
      f.io.wr_en := fifo_in_valid
      collector_data(i) := f.io.dout
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
      f.io.rd_en := io.out.tvalid & io.out.tready
    }
  }

  io.out.tvalid := Mux(io.flush, Seq.tabulate(16)(i => collector_fifos(i).io.empty === false.B).reduce(_|_),
    Seq.tabulate(16)(i => collector_fifos(i).io.empty === false.B).reduce(_&_))
  io.out.tdata := collector_data.asUInt()
  io.out.tlast := true.B
  io.out.tkeep := (VecInit.tabulate(16)(i => collector_fifos(i).io.empty === false.B)).asUInt()
}

/*
* mc send FIN when no more data in current tier FIFO
* FIN: vid[31] = 1
* */
class multi_port_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module {
  val io = IO(new Bundle() {
    val cacheable_out = Flipped(new streamdata(AXI_DATA_WIDTH, 4))
    val cacheable_in = Vec(16, (new streamdata(4, 4)))
    val non_cacheable_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)
    val ddr_out = Flipped(new axidata_blackbox(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH + 1, AXI_SIZE_WIDTH))

    //control path
    val tiers_base_addr = Vec(2, Input(UInt(AXI_ADDR_WIDTH.W)))
    val unvisited_size = Output(UInt(32.W))
    val start = Input(Bool())
    val root = Input(UInt(32.W))
    val signal = Input(Bool())
    val end = Input(Bool())
  })

  val tier_fifo = Seq.tabulate(2)(i => Module(new BRAM_fifo(16, 512, "tier_fifo")))
  val tier_counter = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))
  val collector = Module(new Collector(AXI_DATA_WIDTH))

  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_tier_is_0  = Value(0x1.U) // i "load"  -> 000_0011
    val next_tier_is_1   = Value(0x2.U) // i "imm"   -> 001_0011
    val flushing_0 = Value(0x3.U)
    val flushing_1 = Value(0x4.U)
    val writeback = Value(0x5.U)
    val readback = Value(0x6.U)
  }
  val status = RegInit(sm.idole)

  when(io.start && status === sm.idole){
    status := sm.next_tier_is_1
  }.elsewhen(io.signal && status === sm.next_tier_is_1){
    status := sm.flushing_1
  }.elsewhen(io.signal && status === sm.next_tier_is_0){
    status := sm.flushing_0
  }.elsewhen(collector.io.out.tvalid === false.B && status === sm.flushing_1){
    status := sm.next_tier_is_0
  }.elsewhen(collector.io.out.tvalid === false.B && status === sm.flushing_0){
    status := sm.next_tier_is_1
  }.elsewhen(io.end){
    status := sm.idole
  }

  val next_tier_mask = Cat(status === sm.next_tier_is_1 | status === sm.flushing_1,
      status === sm.next_tier_is_0 | status === sm.flushing_0)
  val axi = Wire(Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  val tier_base_addr = RegInit(VecInit(Seq.fill(2)(0.U(AXI_ADDR_WIDTH.W))))
  val tier_status = RegInit(VecInit(Seq.fill(2)(sm.idole)))
  tier_base_addr.zipWithIndex.map{
    case(a, i) => {
      when(io.start && status === sm.idole){
        a := io.tiers_base_addr(i)
      }.elsewhen(next_tier_mask(i) && axi.aw.ready.asBool() && axi.aw.valid.asBool()){
        a := a + 1024.U
      }.elsewhen(!next_tier_mask(i) && axi.ar.ready.asBool() && axi.ar.valid.asBool()){
        a := a + 1024.U
      }
    }
  }
  tier_counter.zipWithIndex.map{
    case(c, i) => {
      when(next_tier_mask(i) && collector.io.out.tvalid && collector.io.out.tready){
        c := c + collector.io.out.tkeep.asTypeOf(Vec(16, Bool())).asTypeOf(Vec(16, UInt(4.W))).reduce(_+_)
      }.elsewhen(!next_tier_mask(i) && io.cacheable_out.tready && io.cacheable_out.tvalid){
        c := Mux(c > 16.U, c - 16.U, 0.U)
      }
    }
  }
  tier_fifo.zipWithIndex.map{
    case(f, i) => {
      f.io.rd_en := Mux(next_tier_mask(i), axi.w.ready, io.cacheable_out.tready)
      f.io.wr_en := Mux(next_tier_mask(i), collector.io.out.tvalid, axi.r.valid)
      f.io.din := Mux(next_tier_mask(i), collector.io.out.tdata, axi.r.bits.rdata)
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
    }
  }
  tier_status.zipWithIndex.map{
    case (s, i) => {
      when(next_tier_mask(i) && tier_fifo(i).io.full && s === sm.idole){
        s := sm.writeback
      }.elsewhen(!next_tier_mask(i) && tier_fifo(i).io.empty && s === sm.idole){
        s := sm.readback
      }.elsewhen(s === sm.writeback && axi.w.bits.wlast.asBool()){
        s := sm.idole
      }.elsewhen(s === sm.readback && axi.r.bits.rlast.asBool()){
        s := sm.idole
      }
    }
  }

  collector.io.in <> io.cacheable_in
  collector.io.flush := status === sm.flushing_0 | status === sm.flushing_1
  collector.io.out.tready := Mux(next_tier_mask(0), tier_fifo(0).io.full === false.B, tier_fifo(1).io.full === false.B)
  io.unvisited_size := Mux(next_tier_mask(0), tier_counter(0), tier_counter(1))
  io.cacheable_out.tvalid := Mux(next_tier_mask(0), tier_fifo(1).io.empty === false.B, tier_fifo(0).io.empty === false.B)
  io.cacheable_out.tdata := Mux(next_tier_mask(0), tier_fifo(1).io.dout, tier_fifo(0).io.dout)
  io.cacheable_out.tkeep := VecInit(Seq.fill(16)(true.B)).asUInt()
  io.cacheable_out.tlast := true.B

  val wcount = RegInit(0.U(8.W))
  when(axi.aw.valid.asBool() && axi.aw.ready.asBool()){
    wcount := 16.U
  }.elsewhen(axi.w.valid.asBool() && axi.w.ready.asBool()){
    wcount := wcount - 1.U
  }
  axi.aw.bits.awaddr := Mux(next_tier_mask(0), tier_base_addr(0), tier_base_addr(1))
  axi.aw.bits.awid := 0.U
  axi.aw.bits.awlen := 15.U
  axi.aw.bits.awlock := 0.U
  axi.aw.bits.awburst := 1.U
  axi.aw.bits.awsize := 6.U
  axi.aw.valid := Mux(next_tier_mask(0), tier_fifo(0).io.full, tier_fifo(1).io.full)
  axi.ar.bits.araddr := Mux(next_tier_mask(0), tier_base_addr(1), tier_base_addr(0))
  axi.ar.bits.arid := 0.U
  axi.ar.bits.arlen := 15.U
  axi.ar.bits.arburst := 1.U
  axi.ar.bits.arsize := 6.U
  axi.ar.bits.arlock := 0.U
  axi.ar.bits.arlen := 0.U
  axi.ar.valid := Mux(next_tier_mask(0), tier_fifo(1).io.empty, tier_fifo(0).io.empty)
  axi.w.bits.wdata := Mux(next_tier_mask(0), tier_fifo(0).io.dout, tier_fifo(1).io.dout)
  axi.w.valid := Mux(next_tier_mask(0), tier_status(0) === sm.writeback, tier_status(1) === sm.writeback)
  axi.w.bits.wlast := wcount === 1.U
  axi.w.bits.wstrb := VecInit(Seq.fill(64)(true.B)).asUInt()
  axi.b.ready := true.B
  axi.r.ready := true.B

  val arbitrator = Module(new xbar(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  arbitrator.io.aresetn := !reset.asBool()
  arbitrator.io.aclk := clock.asBool()
  axi.r.valid := arbitrator.io.s_axi.rvalid(1)
  axi.r.bits.rdata := arbitrator.io.s_axi.rdata(2 * AXI_DATA_WIDTH * 8 - 1, AXI_DATA_WIDTH * 8)
  axi.r.bits.rid := arbitrator.io.s_axi.rid(2 * AXI_ID_WIDTH - 1, AXI_ID_WIDTH)
  axi.r.bits.rlast := arbitrator.io.s_axi.rlast(1)
  axi.r.bits.rresp := arbitrator.io.s_axi.rresp(3, 2)
  io.non_cacheable_in.r.valid := arbitrator.io.s_axi.rvalid(0)
  io.non_cacheable_in.r.bits.rdata := arbitrator.io.s_axi.rdata(AXI_DATA_WIDTH * 8 - 1, 0)
  io.non_cacheable_in.r.bits.rid := arbitrator.io.s_axi.rid(AXI_ID_WIDTH - 1, 0)
  io.non_cacheable_in.r.bits.rlast := arbitrator.io.s_axi.rlast(0)
  io.non_cacheable_in.r.bits.rresp := arbitrator.io.s_axi.rresp(1, 0)
  arbitrator.io.s_axi.rready := Cat(axi.r.ready, io.non_cacheable_in.r.ready)

  arbitrator.io.s_axi.araddr := Cat(axi.ar.bits.araddr, io.non_cacheable_in.ar.bits.araddr)
  arbitrator.io.s_axi.arid := Cat(axi.ar.bits.arid, io.non_cacheable_in.ar.bits.arid)
  arbitrator.io.s_axi.arvalid := Cat(axi.ar.valid, io.non_cacheable_in.ar.valid)
  arbitrator.io.s_axi.arsize := Cat(axi.ar.bits.arsize, io.non_cacheable_in.ar.bits.arsize)
  arbitrator.io.s_axi.arlen := Cat(axi.ar.bits.arlen, io.non_cacheable_in.ar.bits.arlen)
  arbitrator.io.s_axi.arlock := Cat(axi.ar.bits.arlock, io.non_cacheable_in.ar.bits.arlock)
  arbitrator.io.s_axi.arburst := Cat(axi.ar.bits.arburst, io.non_cacheable_in.ar.bits.arburst)
  axi.ar.ready := arbitrator.io.s_axi.arready(1)
  io.non_cacheable_in.ar.ready := arbitrator.io.s_axi.arready(0)

  arbitrator.io.s_axi.wvalid := Cat(axi.w.valid, io.non_cacheable_in.w.valid)
  arbitrator.io.s_axi.wlast := Cat(axi.w.bits.wlast, io.non_cacheable_in.w.bits.wlast)
  arbitrator.io.s_axi.wdata := Cat(axi.w.bits.wdata, io.non_cacheable_in.w.bits.wdata)
  arbitrator.io.s_axi.wstrb := Cat(axi.w.bits.wstrb, io.non_cacheable_in.w.bits.wstrb)
  axi.w.ready := arbitrator.io.s_axi.wready(1)
  io.non_cacheable_in.w.ready := arbitrator.io.s_axi.wready(0)

  arbitrator.io.s_axi.awvalid := Cat(axi.aw.valid, io.non_cacheable_in.aw.valid)
  arbitrator.io.s_axi.awid := Cat(axi.aw.bits.awid, io.non_cacheable_in.aw.bits.awid)
  arbitrator.io.s_axi.awsize := Cat(axi.aw.bits.awsize, io.non_cacheable_in.aw.bits.awsize)
  arbitrator.io.s_axi.awlen := Cat(axi.aw.bits.awlen, io.non_cacheable_in.aw.bits.awlen)
  arbitrator.io.s_axi.awburst := Cat(axi.aw.bits.awburst, io.non_cacheable_in.aw.bits.awburst)
  arbitrator.io.s_axi.awlock := Cat(axi.aw.bits.awlock, io.non_cacheable_in.aw.bits.awlock)
  arbitrator.io.s_axi.awaddr := Cat(axi.aw.bits.awaddr, io.non_cacheable_in.aw.bits.awaddr)
  axi.aw.ready := arbitrator.io.s_axi.awready(1)
  io.non_cacheable_in.aw.ready := arbitrator.io.s_axi.awready(0)

  arbitrator.io.s_axi.bready := Cat(axi.b.ready, io.non_cacheable_in.b.ready)
  axi.b.valid := arbitrator.io.s_axi.bvalid(1)
  axi.b.bits.bresp := arbitrator.io.s_axi.bresp(3, 2)
  axi.b.bits.bid := arbitrator.io.s_axi.bid(2 * AXI_ID_WIDTH - 1, AXI_ID_WIDTH)
  io.non_cacheable_in.b.valid := arbitrator.io.s_axi.bvalid(0)
  io.non_cacheable_in.b.bits.bresp := arbitrator.io.s_axi.bresp(1, 0)
  io.non_cacheable_in.b.bits.bid := arbitrator.io.s_axi.bid(AXI_ID_WIDTH - 1, 0)

  io.ddr_out <> arbitrator.io.m_axi
}

/*
* traveled_edges: updated every super step
* unvisited_size: unvisited vertex in tiers
* */
class controller (AXI_ADDR_WIDTH : Int = 64) extends Module{
  val io = IO(new Bundle() {
    val data = Output(Vec(32, UInt(32.W)))
    val fin = Input(Vec(16, (Bool())))
    val signal = Output(Bool())
    val level = Output(UInt(32.W))
    val unvisited_size = Input(UInt(32.W))
    val traveled_edges = Input(UInt(64.W))
    val config = (new axilitedata(AXI_ADDR_WIDTH))
  })

  //0 --- start and end register
  //1 --- embedding_base_addr(31, 0)
  //2 --- embedding_base_addr(63, 32)
  //3 --- edge_base_addr(31, 0)
  //4 --- edge_base_addr(63, 32)
  //5 --- level_base_addr(31, 0)
  //6 --- level_base_addr(63, 32)
  //7 --- root
  //8 --- tier1_base_addr(31, 0)
  //9 --- tier1_base_addr(63, 32)
  //10 --- tier2_base_addr(31, 0)
  //11 --- tier2_base_addr(63, 32)
  //12 --- traveled edges(63, 32)
  //13 --- traveled edges(31, 0)
  val controls = Module(new LookupTable(depth = 32, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  val level = RegInit(0.U(32.W))
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val exe  = Value(0x1.U) // i "load"  -> 000_0011
    val fin   = Value(0x2.U) // i "imm"   -> 001_0011
    val end = Value(0x3.U)
  }
  val status = RegInit(sm.idole)
  val start = controls.io.data(0)(0)
  val FIN = RegInit(VecInit(Seq.fill(16)(false.B)))
  val new_tep = Cat(controls.io.data(12), controls.io.data(13)) + io.traveled_edges

  controls.config <> io.config
  controls.io.writeFlag(0) := status === sm.end | status === sm.fin
  controls.io.wptr(0) := Mux(status === sm.end, 0.U, 12.U)
  controls.io.dataIn(0) := Mux(status === sm.end, 2.U, new_tep(63, 32))
  controls.io.writeFlag(1) := status === sm.fin
  controls.io.wptr(1) := 13.U
  controls.io.dataIn(1) := new_tep(31, 0)

  when(status === sm.idole && start){
    status := sm.exe
  }.elsewhen(status === sm.exe && FIN.reduce(_&_)){
    status := sm.fin
  }.elsewhen(status === sm.fin){
    when(io.unvisited_size === 0.U){
      status := sm.end
    }.otherwise{
      status := sm.exe
    }
  }.elsewhen(status === sm.end){
    status := sm.idole
  }

  FIN.zipWithIndex.map{
    case(f, i) => {
      when(io.fin(i)){
        f := true.B
      }.elsewhen(io.signal){
        f := false.B
      }
    }
  }

  when(status === sm.fin){
    level := level + 1.U
  }.elsewhen(status === sm.idole && start){
    level := 0.U
  }

  io.signal := status === sm.idole && start || status === sm.fin
  io.data := controls.io.data
  io.level := level
}

class BFS(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmemory = Flipped(new axidata_blackbox(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    //val PSmempory = Flipped(Vec(4, new axidata(64, 16, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  })

  val controls = Module(new controller(AXI_ADDR_WIDTH))


  val pl_mc = Module(new multi_port_mc(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
  val Scatters = Seq.tabulate(16)(
    i => Module(new Scatter(4, i))
  )
  val Gathers = Module(new Gather())
  val Applys = Module(new Apply())
  val Broadcasts = Module(new Broadcast())
  val bxbar = Module(new broadcast_xbar(64, 16, 1))

  io.PLmemory <> pl_mc.io.ddr_out
  Applys.io.level_size := 26.U
  Applys.io.gather_in.tdata := Gathers.io.gather_out.tdata
  Applys.io.gather_in.tkeep := Gathers.io.gather_out.tkeep
  Applys.io.gather_in.tvalid := Gathers.io.gather_out.tvalid
  Applys.io.gather_in.tlast := Gathers.io.gather_out.tlast
  Broadcasts.io.gather_in.tdata := Gathers.io.gather_out.tdata
  Broadcasts.io.gather_in.tkeep := Gathers.io.gather_out.tkeep
  Broadcasts.io.gather_in.tvalid := Gathers.io.gather_out.tvalid
  Broadcasts.io.gather_in.tlast := Gathers.io.gather_out.tlast
  Gathers.io.gather_out.tready := Broadcasts.io.gather_in.tready & Applys.io.gather_in.tready
  Gathers.io.ddr_in <> pl_mc.io.cacheable_out
  bxbar.io.ddr_in(0) <> Broadcasts.io.xbar_out

  Broadcasts.io.ddr_r <> pl_mc.io.non_cacheable_in.r
  Broadcasts.io.ddr_ar <> pl_mc.io.non_cacheable_in.ar
  Applys.io.ddr_w <> pl_mc.io.non_cacheable_in.w
  Applys.io.ddr_aw <> pl_mc.io.non_cacheable_in.aw
  Applys.io.ddr_b <> pl_mc.io.non_cacheable_in.b

  Scatters.zipWithIndex.map{
    case (pe, i) => {
      pe.io.xbar_in <> bxbar.io.pe_out(i)
      pe.io.ddr_out <> pl_mc.io.cacheable_in(i)
      controls.io.fin(i) := pe.io.end
    }
  }

  //control path
  controls.io.config <> io.config
  controls.io.traveled_edges := Broadcasts.io.traveled_edges
  controls.io.unvisited_size := pl_mc.io.unvisited_size
  Gathers.io.signal := controls.io.signal
  Applys.io.signal := controls.io.signal
  Broadcasts.io.signal := controls.io.signal
  Broadcasts.io.embedding_base_addr := Cat(controls.io.data(2), controls.io.data(1))
  Broadcasts.io.edge_base_addr := Cat(controls.io.data(4), controls.io.data(3))
  Applys.io.level_base_addr := Cat(controls.io.data(6), controls.io.data(5))
  Applys.io.level := controls.io.level
  pl_mc.io.tiers_base_addr(0) := Cat(controls.io.data(9), controls.io.data(8))
  pl_mc.io.tiers_base_addr(1) := Cat(controls.io.data(11), controls.io.data(10))
  pl_mc.io.root := controls.io.data(7)
  pl_mc.io.signal := controls.io.signal
  pl_mc.io.start := controls.io.data(0)(0)
  pl_mc.io.end := controls.io.data(0)(1)
}