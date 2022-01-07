package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce._
import chisel3.util._
import utils._
import numa.LookupTable

class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))

    val xbar_in = Flipped(Decoupled(new axisdata(4)))
    val level_base_addr = Input(UInt(64.W))
    //val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val level = Input(UInt(32.W))
  })

  val buffer = Module(new URAM(512 * 1024,32))

  def get_addr_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index + 1.U(7.W) + (size >> 1.U).asTypeOf(UInt(7.W)))
  }

  def get_level_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index + (1 + 42 / 2).U(7.W) + size.asTypeOf(UInt(7.W)))
  }

  def get_size_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index + 0.U(7.W))
  }

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
  when(io.xbar_in.valid && io.xbar_in.ready){
    vid := io.xbar_in.bits.tdata
    level := io.level
  }
  io.xbar_in.ready := update_sm === sm.read_size & (wb_sm === sm.idole)
  when(io.xbar_in.valid && io.xbar_in.ready){
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
  val dramaddr = (vid << 2.U).asTypeOf(UInt(AXI_ADDR_WIDTH.W))
  val addr = dramaddr(13, 0).asTypeOf(UInt(16.W))     //page index + page offset
  //addr := vid(11, 0) << 2.U
  val block_index = dramaddr(25, 14)
  val size = RegInit(0.U(32.W))
  when(update_sm === sm.read_addr) {
    size := buffer.io.douta
  }
  val old_addr_pair = buffer.io.douta
  val new_addr_pair = Mux(size(0), Cat(addr, old_addr_pair(15, 0)), Cat(old_addr_pair(31, 16), addr))
  buffer.io.ena := true.B
  buffer.io.wea := (update_sm === sm.write_addr) | (update_sm === sm.write_level) | (update_sm === sm.write_size)
  buffer.io.addra := Mux1H(Seq(
    (update_sm === sm.read_size) -> get_size_addr(block_index, size),
    (update_sm === sm.read_addr) -> get_addr_addr(block_index, size),
    (update_sm === sm.write_addr) -> get_addr_addr(block_index, size),
    (update_sm === sm.write_level) -> get_level_addr(block_index, size),
    (update_sm === sm.write_size) -> get_size_addr(block_index, size)
  ))
  buffer.io.dina := Mux1H(Seq(
    (update_sm === sm.write_addr) -> (new_addr_pair),
    (update_sm === sm.write_level) -> (level),
    (update_sm === sm.write_size) -> (size + 1.U)
  ))
  buffer.io.clka := clock.asBool()

  //write back buffer
  val count = RegInit(0.U(8.W))
  val aw_buffer = Module(new BRAM_fifo(32, AXI_ADDR_WIDTH, "addr_fifo"))
  val w_buffer = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  val wb_block_index = RegInit(0.U(12.W))
  aw_buffer.io.clk := clock.asBool()
  aw_buffer.io.srst := reset.asBool()
  w_buffer.io.clk := clock.asBool()
  w_buffer.io.srst := reset.asBool()
  when(update_sm === sm.write_size && buffer.io.dina === 84.U){
    wb_block_index := block_index
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
  buffer.io.enb := true.B
  buffer.io.web := (wb_sm === sm.write_size)
  buffer.io.addrb := Mux1H(Seq(
    (update_sm === sm.read_addr) -> get_addr_addr(wb_block_index, count),
    (update_sm === sm.read_level1) -> get_level_addr(wb_block_index, count),
    (update_sm === sm.read_level2) -> get_level_addr(wb_block_index, count),
    (update_sm === sm.write_size) -> get_size_addr(wb_block_index, count)
  ))
  buffer.io.dinb := 0.U
  buffer.io.clkb := clock.asBool()
  when(wb_sm === sm.read_addr){
    wb_addr_pair := buffer.io.doutb
  }
  when(wb_sm === sm.read_level1 || wb_sm === sm.read_level2){
    wb_level := buffer.io.doutb
  }

  aw_buffer.io.wr_en := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  aw_buffer.io.din := Mux(wb_sm === sm.wb_level1, io.level_base_addr + Cat(wb_block_index, wb_addr_pair(13,0)).asUInt(),
    io.level_base_addr + Cat(wb_block_index, wb_addr_pair(16 + 14 - 1,16)))
  io.ddr_aw.bits.awaddr := aw_buffer.io.dout
  io.ddr_aw.bits.awlock := 0.U
  io.ddr_aw.bits.awid := aw_buffer.io.data_count
  io.ddr_aw.bits.awlen := 0.U
  io.ddr_aw.bits.awburst := 1.U(2.W)
  io.ddr_aw.bits.awsize := 2.U
  io.ddr_aw.valid := aw_buffer.is_valid()
  aw_buffer.io.rd_en := io.ddr_aw.ready

  w_buffer.io.wr_en := wb_sm === sm.wb_level1 || wb_sm === sm.wb_level2
  w_buffer.io.din := wb_level
  io.ddr_w.bits.wdata := w_buffer.io.dout
  io.ddr_w.bits.wlast := true.B
  io.ddr_w.valid := w_buffer.is_valid()
  io.ddr_w.bits.wstrb := 0xf.U
  w_buffer.io.rd_en := io.ddr_w.ready

  io.ddr_b.ready := true.B
}

class Apply(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle(){
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))
    val gather_in = Flipped(Decoupled(new axisdata(4)))

    //control path
    val level = Input(UInt(32.W))
    val level_base_addr = Input(UInt(64.W))
    //val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val signal = Input(Bool())    //used in prefetch mode
  })

  //write value array
  val vertex_update_buffer = Module(new BRAM_fifo(32, 64, "update_fifo"))
  val FIN = io.gather_in.bits.tdata(31) & io.gather_in.valid
  vertex_update_buffer.io.clk := clock.asBool()
  vertex_update_buffer.io.srst := reset.asBool()
  vertex_update_buffer.io.wr_en := io.gather_in.valid & ~FIN
  vertex_update_buffer.io.din := Cat(io.gather_in.bits.tdata, io.level)

  io.gather_in.ready := vertex_update_buffer.io.full === false.B

  val update_engine = Module(new WB_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_DATA_WIDTH))
  update_engine.io.xbar_in.bits.tdata := vertex_update_buffer.io.dout(63, 32)
  update_engine.io.xbar_in.valid := vertex_update_buffer.is_valid()
  update_engine.io.level_base_addr := io.level_base_addr
  //update_engine.io.level_size := io.level_size
  update_engine.io.xbar_in.bits.tlast := true.B
  update_engine.io.xbar_in.bits.tkeep := 0xff.U
  update_engine.io.level := vertex_update_buffer.io.dout(31, 0)
  vertex_update_buffer.io.rd_en := update_engine.io.xbar_in.ready

  io.ddr_aw <> update_engine.io.ddr_aw
  io.ddr_w <> update_engine.io.ddr_w
  io.ddr_b <> update_engine.io.ddr_b
}

class axis_arbitrator(AXIS_DATA_WIDTH: Int = 4, NUM : Int = 16, ELEMENT_WIDTH: Int = 4) extends Module{
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH * NUM, ELEMENT_WIDTH)))
    val ddr_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, ELEMENT_WIDTH))
  })

  val data = RegInit(0.U((AXIS_DATA_WIDTH * NUM * 8).W))
  val keep = RegInit(0.U((AXIS_DATA_WIDTH * NUM / ELEMENT_WIDTH).W))

  when(io.xbar_in.valid && io.xbar_in.ready){
    data := io.xbar_in.bits.tdata
  }
  io.xbar_in.ready := ~(keep.orR())

  val select = MuxCase(0.U,
    Array.tabulate(NUM)(x => (keep((x+1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH)).orR() -> (1.U(NUM.W) << x).asUInt())))
  io.ddr_out.valid := select.orR()
  io.ddr_out.bits.tkeep := Mux1H(
    Seq.tabulate(NUM)(x => (select(x) -> keep((x + 1) * (AXIS_DATA_WIDTH / ELEMENT_WIDTH) - 1, x * (AXIS_DATA_WIDTH / ELEMENT_WIDTH))))
  )
  io.ddr_out.bits.tlast := true.B
  io.ddr_out.bits.tdata :=
    Mux1H(Seq.tabulate(NUM)(x => (select(x) -> data((x + 1) * AXIS_DATA_WIDTH * 8 - 1, x * AXIS_DATA_WIDTH * 8))))

  val next_keep = Wire(Vec(NUM, UInt((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  next_keep.zipWithIndex.map{
    case(k, i) => (k := Mux(select(i), 0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W), ~0.U((AXIS_DATA_WIDTH / ELEMENT_WIDTH).W)))
  }
  when(io.xbar_in.valid && io.xbar_in.ready){
    keep := io.xbar_in.bits.tkeep
  }.elsewhen(io.ddr_out.valid && io.ddr_out.ready) {
    keep := keep & next_keep.asUInt()
  }
}

class Gather(AXI_DATA_WIDTH: Int = 64) extends Module{
  val io = IO(new Bundle() {
    val ddr_in = Flipped(Decoupled(new axisdata(AXI_DATA_WIDTH, 4)))
    val gather_out = Decoupled(new axisdata(4, 4))

    //control path
    val signal = Input(Bool())
  })
  val Selector = Module(new axis_arbitrator(4, 16, 4))
  Selector.io.xbar_in <> io.ddr_in
  io.gather_out <> Selector.io.ddr_out
}

class Broadcast(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3, EMBEDDING : Int = 14) extends Module{
  val io = IO(new Bundle() {
    val ddr_ar = Decoupled(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH))
    val ddr_r = Flipped(Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH)))
    val gather_in = Flipped(Decoupled(new axisdata(4, 4)))
    val xbar_out = Decoupled(new axisdata(64, 4))

    //control path
    val embedding_base_addr = Input(UInt(64.W))
    val edge_base_addr = Input(UInt(64.W))
    val signal = Input(Bool())
    val traveled_edges = Output(UInt(64.W))       //every super step
  })
  assert(AXI_DATA_WIDTH >= 8)
  assert(AXI_ID_WIDTH > log2Ceil(32))

  def get_edge_array_index(mdata : UInt) : UInt = {
    mdata(31, 0)
  }

  def get_edge_count(mdata : UInt) : UInt = {
    mdata(63, 32)
  }

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
  val vertex_read_buffer = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  vertex_read_buffer.io.clk := clock.asBool()
  vertex_read_buffer.io.srst := reset.asBool()
  vertex_read_buffer.io.wr_en := io.gather_in.valid
  vertex_read_buffer.io.din := io.gather_in.bits.tdata
  io.gather_in.ready := vertex_read_buffer.io.full === false.B

  //read edge array
  val edge_read_buffer = Module(new BRAM_fifo(32, 64, "meta_fifo"))

  edge_read_buffer.io.clk := clock.asBool()
  edge_read_buffer.io.srst := reset.asBool()
  edge_read_buffer.io.din := io.ddr_r.bits.rdata(8 * 8 - 1, 0)
  edge_read_buffer.io.wr_en := !io.ddr_r.bits.rid(AXI_ID_WIDTH - 1) & io.ddr_r.valid & (get_edge_count(io.ddr_r.bits.rdata) > EMBEDDING.asUInt())

  val arbi = Module(new Arbiter(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1), 2))
  //TODO we should ensure regfile is big enough in case the data return slowly
  val num_regfile = Module(new regFile(32, 32))

  io.ddr_ar <> arbi.io.out
  arbi.io.in(0).bits.araddr := io.edge_base_addr + get_edge_array_index(edge_read_buffer.io.dout) << 2
  arbi.io.in(1).bits.araddr :=  io.embedding_base_addr + vertex_read_buffer.io.dout << log2Ceil(4 * EMBEDDING + 8)
  arbi.io.in(0).valid := edge_read_buffer.is_valid()
  arbi.io.in(1).valid :=  vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U
  val remainning_edges = (get_edge_count(edge_read_buffer.io.dout) - EMBEDDING.asUInt())
  // TODO we assume number of edges of a vertex is less than 4096 + 14 for now.
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
  arbi.io.in(0).bits.arid := (1.U(AXI_ID_WIDTH) << (AXI_ID_WIDTH - 1)).asUInt() + num_regfile.io.wcount
  arbi.io.in(1).bits.arid :=  vertex_read_buffer.io.data_count.asTypeOf(UInt(AXI_ID_WIDTH.W))
  vertex_read_buffer.io.rd_en := arbi.io.in(1).ready | upward_status === upward_sm.output_fin
  edge_read_buffer.io.rd_en := arbi.io.in(0).ready

  //front end of down ward
  num_regfile.io.writeFlag := arbi.io.in(0).ready & arbi.io.in(0).valid
  num_regfile.io.wptr := num_regfile.io.wcount
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
      Mux(i.U < 2.U, false.B, (get_edge_count(io.ddr_r.bits.rdata) + 2.U) > i.U))
  }

  val vertex_out_fifo = Module(new BRAM_fifo(32, 512 + 16, "edge_fifo"))
  vertex_out_fifo.io.srst := reset.asBool()
  vertex_out_fifo.io.clk := clock.asBool()
  io.xbar_out.valid := vertex_out_fifo.is_valid()
  io.xbar_out.bits.tkeep := vertex_out_fifo.io.dout(15, 0)
  io.xbar_out.bits.tdata := vertex_out_fifo.io.dout(512 + 16 - 1, 16)
  io.xbar_out.bits.tlast := true.B
  vertex_out_fifo.io.rd_en := io.xbar_out.ready
  vertex_out_fifo.io.wr_en := io.ddr_r.valid | upward_status === upward_sm.output_fin
  vertex_out_fifo.io.din := Mux(upward_status === upward_sm.output_fin, Cat(vertex_read_buffer.io.dout, 0x1.U(16.W)), Cat(io.ddr_r.bits.rdata, keep.asUInt()))
  io.ddr_r.ready := vertex_out_fifo.io.full === false.B & edge_read_buffer.io.full === false.B

  //control path
  when(io.signal && (upward_status === upward_sm.idole)){
    upward_status := upward_sm.exe
  }.elsewhen(upward_status === upward_sm.exe && vertex_read_buffer.test_FIN()){
    upward_status := upward_sm.fin
  }.elsewhen(upward_status === upward_sm.fin && inflight_vtxs === 0.U){
    upward_status := upward_sm.output_fin
  }.elsewhen(upward_status === upward_sm.output_fin && vertex_out_fifo.io.full === false.B){
    upward_status := upward_sm.idole
  }

  when(io.signal){
    traveled_edges_reg := 0.U
  }.elsewhen(!io.ddr_r.bits.rid(AXI_ID_WIDTH - 1) & io.ddr_r.valid.asBool() & io.ddr_r.ready.asBool()){
    traveled_edges_reg := traveled_edges_reg + get_edge_count(io.ddr_r.bits.rdata)
  }

  when(vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U && vertex_read_buffer.io.rd_en
    && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool() && !edge_read_buffer.io.wr_en){
    inflight_vtxs := inflight_vtxs
  }.elsewhen(vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U && vertex_read_buffer.io.rd_en){
    inflight_vtxs := inflight_vtxs + 1.U
  }.elsewhen(io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool() && !edge_read_buffer.io.wr_en){
    inflight_vtxs := inflight_vtxs - 1.U
  }
}

/*
* PE ID is global addressable
* MC ID is local addressable
* */
class broadcast_xbar(AXIS_DATA_WIDTH: Int = 64, SLAVE_NUM: Int, MASTER_NUM: Int) extends Module {
  val io = IO(new Bundle {
    val ddr_in = Vec(MASTER_NUM, Flipped(Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))))
    //val ddr_out = Flipped(Vec(MASTER_NUM, (new streamdata(AXIS_DATA_WIDTH, 4))))
    val pe_out = (Vec(SLAVE_NUM, Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))))
    //val pe_in = Vec(SLAVE_NUM, (new streamdata(4, 4)))
  })
  //ddr_in to pe_out direction
  val arbitrator = Module(new Arbiter(new axisdata(AXIS_DATA_WIDTH, 4), MASTER_NUM))
  arbitrator.io.in <> io.ddr_in

  val PES_tready = Wire(Vec(SLAVE_NUM, Bool()))
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      pe.valid := arbitrator.io.out.valid
      pe.bits.tkeep := arbitrator.io.out.bits.tkeep
      pe.bits.tdata := arbitrator.io.out.bits.tdata
      pe.bits.tlast := arbitrator.io.out.bits.tlast
      PES_tready(i) := pe.ready
    }
  }
  arbitrator.io.out.ready := PES_tready.asUInt().andR()
}

/*
* end: there is no more vertexes for the level
* */
class Scatter(AXIS_DATA_WIDTH: Int = 4, SID: Int) extends Module {
  val io = IO(new Bundle() {
    val xbar_in = Flipped(Decoupled(new axisdata(64, 4)))
    val ddr_out = Decoupled(new axisdata(AXIS_DATA_WIDTH, 4))

    //control path
    val end = Output(Bool())
    val start = Input(Bool())
    val root = Input(UInt(32.W))
  })

  val bitmap = Module(new BRAM(1024 * 1024, 1, "bitmap_0"))
  val arbi = Module(new axis_arbitrator(4, 16, 4))

  def vid_to_sid(vid: UInt, sid: UInt) : Bool = {
    vid(31, 20) === sid
  }

  val filtered_keep = Wire(Vec(16, Bool()))
  filtered_keep.zipWithIndex.map{
    case(k, i) => {
      k := Mux(io.xbar_in.bits.get_ith_data(i)(31), true.B, io.xbar_in.bits.tkeep(i) & vid_to_sid(io.xbar_in.bits.get_ith_data(i), SID.asUInt()))
    }
  }

  arbi.io.xbar_in.bits.tkeep := filtered_keep.asUInt()
  arbi.io.xbar_in.bits.tdata := io.xbar_in.bits.tdata
  arbi.io.xbar_in.valid := io.xbar_in.valid
  arbi.io.xbar_in.bits.tlast := io.xbar_in.bits.tlast
  io.xbar_in.ready := arbi.io.xbar_in.ready

  val vertex_in_fifo = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  val vertex_out_fifo = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  vertex_in_fifo.io.clk := clock.asBool()
  vertex_in_fifo.io.srst := reset.asBool()
  vertex_out_fifo.io.clk := clock.asBool()
  vertex_out_fifo.io.srst := reset.asBool()

  //read or write bitmap ehnr this is not FIN
  val write_root = io.start && vid_to_sid(io.root, SID.asUInt())
  val bitmap_arvalid = vertex_in_fifo.is_valid()
  val halt = vertex_out_fifo.io.full === true.B
  val bitmap_write = Module(new pipeline(32))
  bitmap_write.io.din.valid := bitmap_arvalid
  bitmap_write.io.din.bits := vertex_in_fifo.io.dout
  bitmap_write.io.dout.ready := !halt

  vertex_in_fifo.io.din := arbi.io.ddr_out.bits.tdata
  vertex_in_fifo.io.wr_en := arbi.io.ddr_out.valid
  arbi.io.ddr_out.ready := vertex_in_fifo.io.full === false.B
  bitmap.io.enb :=  true.B//bitmap_arvalid
  bitmap.io.addrb := vertex_in_fifo.io.dout(19, 0)
  bitmap.io.clkb := clock.asBool()
  vertex_in_fifo.io.rd_en := !halt
  vertex_out_fifo.io.wr_en := bitmap_write.io.dout.valid && bitmap.io.doutb =/= 1.U(1.W)
  vertex_out_fifo.io.din := bitmap_write.io.dout.bits
  bitmap.io.ena := true.B
  bitmap.io.wea := bitmap_write.io.dout.valid & bitmap_write.io.dout.bits(31) === 0.U(1.W) | write_root
  bitmap.io.clka := clock.asBool()
  bitmap.io.dina := 1.U
  bitmap.io.addra := Mux(write_root, io.root(19, 0), bitmap_write.io.dout.bits(19, 0))
  vertex_out_fifo.io.rd_en := io.ddr_out.ready //| vertex_out_fifo.test_FIN()
  io.ddr_out.valid := vertex_out_fifo.is_valid() & vertex_out_fifo.io.dout(31) === 0.U
  io.ddr_out.bits.tkeep := true.B
  io.ddr_out.bits.tlast := true.B
  io.ddr_out.bits.tdata := vertex_out_fifo.io.dout

  //control path
  io.end := vertex_out_fifo.test_FIN()
}

class Collector(AXI_DATA_WIDTH: Int = 64) extends Module{
  val io = IO(new Bundle() {
    val in = Vec(16, Flipped(Decoupled(new axisdata(4, 4))))
    val out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
    val flush = Input(Bool())
  })

  val collector_fifos = Seq.tabulate(16)(
    i => Module(new BRAM_fifo(16, 32, ("collector_fifo_0")))
  )
  val counter = RegInit(0.U(log2Ceil(16).W))
  val collector_data = Wire(Vec(16, UInt(32.W)))
  val sorted_data = Wire(Vec(16, UInt(32.W)))
  val sorted_valid = Wire(Vec(16, Bool()))
  val steps = (Seq.tabulate(16)(
    i => {
      Seq.tabulate(i + 1)(x => io.in(x).valid.asTypeOf(UInt(4.W))).reduce(_+_)
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
      in.ready := Seq.tabulate(16)(i => collector_fifos(i).io.full === false.B).reduce(_&_)
    }
  }
  sorted_data.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(0.U,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, io.in(x).bits.tdata)
        ))
    }
  }
  sorted_valid.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(false.B,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, io.in(x).valid)
        ))
    }
  }

  when(sorted_valid.reduce(_|_)){
    counter := indexAdd(counter, MuxCase(0.U, Seq.tabulate(16)(
      x => (io.in(16 - x - 1).ready && sorted_valid(16 - x - 1), (16 - x).U)
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
      f.io.rd_en := io.out.valid & io.out.ready
    }
  }

  io.out.valid := Mux(io.flush, Seq.tabulate(16)(i => collector_fifos(i).is_valid()).reduce(_|_),
    Seq.tabulate(16)(i => collector_fifos(i).is_valid()).reduce(_&_))
  io.out.bits.tdata := collector_data.asUInt()
  io.out.bits.tlast := true.B
  io.out.bits.tkeep := (VecInit.tabulate(16)(i => collector_fifos(i).is_valid())).asUInt()
}

/*
* mc send FIN when no more data in current tier FIFO
* FIN: vid[31] = 1
* */
class multi_port_mc(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module {
  val io = IO(new Bundle() {
    val cacheable_out = Decoupled(new axisdata(AXI_DATA_WIDTH, 4))
    val cacheable_in = Vec(16, Flipped(Decoupled(new axisdata(4, 4))))
    val non_cacheable_in = new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)
    val ddr_out = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))

    //control path
    val tiers_base_addr = Vec(2, Input(UInt(AXI_ADDR_WIDTH.W)))
    val unvisited_size = Output(UInt(32.W))
    val start = Input(Bool())
    val root = Input(UInt(32.W))
    val signal = Input(Bool())
    val end = Input(Bool())
  })

  val tier_fifo = Seq.tabulate(2)(i => Module(new BRAM_fifo(16, 512, "tier_fifo_0")))
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
    val start = Value(0x7.U)
    val sync_send_0 = Value(0x8.U)
    val sync_send_1 = Value(0x9.U)
    val signal_wait_0 = Value(0x10.U)
    val signal_wait_1 = Value(0x11.U)
    val writebackdata = Value(0x12.U)
  }
  val status = RegInit(sm.idole)

  when(io.start && status === sm.idole){
    status := sm.start
  }.elsewhen(status === sm.start && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.next_tier_is_1
  }.elsewhen(status === sm.next_tier_is_1 && tier_counter(0) === 0.U){
    status := sm.sync_send_0
  }.elsewhen(status === sm.sync_send_0 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_0
  }.elsewhen(io.signal && status === sm.signal_wait_0){
    status := sm.flushing_1
  }.elsewhen(status === sm.next_tier_is_0 && tier_counter(1) === 0.U){
    status := sm.sync_send_1
  }.elsewhen(status === sm.sync_send_1 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_1
  }.elsewhen(io.signal && status === sm.signal_wait_1){
    status := sm.flushing_0
  }.elsewhen(collector.io.out.valid === false.B && status === sm.flushing_1){
    status := sm.next_tier_is_0
  }.elsewhen(collector.io.out.valid === false.B && status === sm.flushing_0){
    status := sm.next_tier_is_1
  }.elsewhen(io.end){
    status := sm.idole
  }

  val next_tier_mask = Cat(status === sm.next_tier_is_1 | status === sm.flushing_1 | status === sm.sync_send_0 | status === sm.signal_wait_0,
      status === sm.next_tier_is_0 | status === sm.flushing_0 | status === sm.sync_send_1 | status === sm.signal_wait_1)
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
      when(next_tier_mask(i) && io.cacheable_in.map{ case i => i.ready && i.valid }.reduce(_|_)){
        c := c + io.cacheable_in.map{ case i => (i.ready && i.valid).asTypeOf(UInt(6.W)) }.reduce(_+_)
      }.elsewhen(!next_tier_mask(i) && io.cacheable_out.ready && io.cacheable_out.valid){
        c := Mux(c > 16.U, c - 16.U, 0.U)
      }
    }
  }
  tier_fifo.zipWithIndex.map{
    case(f, i) => {
      f.io.rd_en := Mux(next_tier_mask(i), axi.w.ready, io.cacheable_out.ready)
      f.io.wr_en := Mux(next_tier_mask(i), collector.io.out.valid, axi.r.valid)
      f.io.din := Mux(next_tier_mask(i), collector.io.out.bits.tdata, axi.r.bits.rdata)
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
    }
  }
  tier_status.zipWithIndex.map{
    case (s, i) => {
      when(next_tier_mask(i) && tier_fifo(i).io.full && s === sm.idole){
        s := sm.writeback
      }.elsewhen(!next_tier_mask(i) && tier_fifo(i).is_empty() && tier_counter(i) =/= 0.U && s === sm.idole){
        s := sm.readback
      }.elsewhen(s === sm.writeback && axi.aw.ready){
        s := sm.writebackdata
      }.elsewhen(s === sm.readback && axi.ar.ready){
        s := sm.idole
      }.elsewhen(s === sm.writebackdata && axi.w.bits.wlast.asBool()){
        s := sm.idole
      }
    }
  }

  collector.io.in <> io.cacheable_in
  collector.io.flush := status === sm.flushing_0 | status === sm.flushing_1
  collector.io.out.ready := Mux(next_tier_mask(0), tier_fifo(0).io.full === false.B, tier_fifo(1).io.full === false.B)
  io.unvisited_size := Mux(next_tier_mask(0), tier_counter(0), tier_counter(1))
  io.cacheable_out.valid := MuxCase(status === sm.start,
    Array(next_tier_mask(0) -> (tier_fifo(1).is_valid() | status === sm.sync_send_1),
          next_tier_mask(1) -> (tier_fifo(0).is_valid() | status === sm.sync_send_0)))
  io.cacheable_out.bits.tdata := MuxCase(io.root.asTypeOf(UInt(512.W)),
    Array((status === sm.sync_send_0) -> "x80000000".asUInt(512.W),
          (status === sm.sync_send_1) -> "x80000000".asUInt(512.W),
          next_tier_mask(0) -> tier_fifo(1).io.dout,
          next_tier_mask(1) -> tier_fifo(0).io.dout))
  io.cacheable_out.bits.tkeep := Mux(status === sm.start | status === sm.sync_send_0 | status === sm.sync_send_1, 1.U(16.W),
    VecInit(Seq.tabulate(16)(
      i => Mux(next_tier_mask(0), tier_counter(1) > i.U, tier_counter(0) > i.U)
    )).asUInt())
  io.cacheable_out.bits.tlast := true.B

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
  axi.aw.valid := Mux(next_tier_mask(0), tier_status(0) === sm.writeback, tier_status(1) === sm.writeback)
  axi.ar.bits.araddr := Mux(next_tier_mask(0), tier_base_addr(1), tier_base_addr(0))
  axi.ar.bits.arid := 0.U
  axi.ar.bits.arlen := 15.U
  axi.ar.bits.arburst := 1.U
  axi.ar.bits.arsize := 6.U
  axi.ar.bits.arlock := 0.U
  axi.ar.bits.arlen := 0.U
  axi.ar.valid := Mux(next_tier_mask(0), tier_status(1) === sm.readback, tier_status(0) === sm.readback)
  axi.w.bits.wdata := Mux(next_tier_mask(0), tier_fifo(0).io.dout, tier_fifo(1).io.dout)
  axi.w.valid := Mux(next_tier_mask(0), tier_status(0) === sm.writebackdata, tier_status(1) === sm.writebackdata)
  axi.w.bits.wlast := wcount === 1.U
  axi.w.bits.wstrb := VecInit(Seq.fill(64)(true.B)).asUInt()
  axi.b.ready := true.B
  axi.r.ready := true.B

  io.ddr_out(0) <> axi
  io.ddr_out(1) <> io.non_cacheable_in
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
    val start = Output(Bool())
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
    val start = Value(0x4.U)
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
    status := sm.start
  }.elsewhen(status === sm.start){
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

  io.signal := status === sm.start || status === sm.fin
  io.data := controls.io.data
  io.level := level
  io.start := status === sm.start
}

class BFS(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val config = new axilitedata(AXI_ADDR_WIDTH)
    val PLmemory = Vec(2, Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
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
  //Applys.io.level_size := 26.U
  Applys.io.gather_in.bits.tdata := Gathers.io.gather_out.bits.tdata
  Applys.io.gather_in.bits.tkeep := Gathers.io.gather_out.bits.tkeep
  Applys.io.gather_in.valid := Gathers.io.gather_out.valid
  Applys.io.gather_in.bits.tlast := Gathers.io.gather_out.bits.tlast
  Broadcasts.io.gather_in.bits.tdata := Gathers.io.gather_out.bits.tdata
  Broadcasts.io.gather_in.bits.tkeep := Gathers.io.gather_out.bits.tkeep
  Broadcasts.io.gather_in.valid := Gathers.io.gather_out.valid
  Broadcasts.io.gather_in.bits.tlast := Gathers.io.gather_out.bits.tlast
  Gathers.io.gather_out.ready := Broadcasts.io.gather_in.ready & Applys.io.gather_in.ready
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
      pe.io.root := controls.io.data(7)
      pe.io.start := controls.io.start
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
  pl_mc.io.start := controls.io.start
  pl_mc.io.end := controls.io.data(0)(1)
}