package bfs

import chisel3._
import chisel3.experimental.ChiselEnum
import nf_arm_doce._
import chisel3.util._
import utils._
import numa.LookupTable

//TODO: rename to cache
class WB_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3) extends Module{
  val io = IO(new Bundle() {
    val ddr_aw = Decoupled(new axiaw(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    val ddr_w = Decoupled(new axiw(AXI_DATA_WIDTH, 1))
    val ddr_b = Flipped(Decoupled(new axib(AXI_ID_WIDTH, 1)))

    val xbar_in = Flipped(Decoupled(new axisdata(4)))
    val level_base_addr = Input(UInt(64.W))
    //val level_size = Input(UInt(64.W))     //2^level_size in bytes
    val level = Input(UInt(32.W))
    val end = Output(Bool())
    val flush = Input(Bool())
  })
  val buffer = Module(new URAM(256 * 1024,48))
  val region_counter = RegInit(VecInit(Seq.fill(4096)(0.U(7.W))))

  def get_level_addr(block_index: UInt, size : UInt) : UInt = {
    Cat(block_index, size.asTypeOf(UInt(6.W)))
  }

  //update buffer
  val vid = io.xbar_in.bits.tdata
  val level = io.level
  val dramaddr = (vid << 2.U).asTypeOf(UInt(AXI_ADDR_WIDTH.W))
  val addr = dramaddr(13, 0).asTypeOf(UInt(16.W))     //page index + page offset
  //addr := vid(11, 0) << 2.U
  val block_index = dramaddr(25, 14)

  io.xbar_in.ready := region_counter(block_index) =/= 64.U
  buffer.io.ena := true.B
  buffer.io.wea := io.xbar_in.valid && io.xbar_in.ready
  buffer.io.addra := get_level_addr(block_index, region_counter(block_index))
  buffer.io.dina := Cat(addr, level)
  buffer.io.clka := clock.asBool()

  //write back buffer
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val wb_level = Value(0x1.U)
    val wb_1block = Value(0x2.U)
  }
  val wb_sm = RegInit(sm.idole)
  val count = RegInit(0.U(8.W))
  val aw_buffer = Module(new BRAM_fifo(32, AXI_ADDR_WIDTH, "addr_fifo"))
  val w_buffer = Module(new BRAM_fifo(32, 32, "vid_fifo"))
  val wb_block_index = RegInit(0.U(12.W))
  val flush_start = (wb_sm === sm.idole) && io.flush
  val size_b = region_counter(wb_block_index) - 1.U
  when(io.xbar_in.valid && io.xbar_in.ready) {
    region_counter(block_index) := region_counter(block_index) + 1.U
  }.elsewhen(wb_sm === sm.wb_level && aw_buffer.io.full === false.B && w_buffer.io.full === false.B
            && count === size_b){
    region_counter(block_index) := 0.U
  }
  aw_buffer.io.clk := clock.asBool()
  aw_buffer.io.srst := reset.asBool()
  w_buffer.io.clk := clock.asBool()
  w_buffer.io.srst := reset.asBool()
  when(io.xbar_in.valid && io.xbar_in.ready && region_counter(block_index) === 63.U){
    wb_block_index := block_index
  }.elsewhen(flush_start){
    wb_block_index := 0.U
  }.elsewhen(wb_sm === sm.wb_1block && wb_block_index =/= (4 * 1024 - 1).U){
    wb_block_index := wb_block_index + 1.U
  }
  //TODO: speculatively check the dirty of a block
  when(flush_start && region_counter(0.U) =/= 0.U){
    wb_sm := sm.wb_level
  }.elsewhen(flush_start && region_counter(0.U) === 0.U){
    wb_sm := sm.wb_1block
  }.elsewhen(io.xbar_in.valid && io.xbar_in.ready && region_counter(block_index) === 63.U){
    wb_sm := sm.wb_level
  }.elsewhen(wb_sm === sm.wb_level && aw_buffer.io.full === false.B && w_buffer.io.full === false.B) {
    when(count === size_b){
      when(io.flush){
        wb_sm := sm.wb_1block
      }.otherwise{
        wb_sm := sm.idole
      }
    }.otherwise{
      wb_sm := sm.wb_level
    }
  }.elsewhen(wb_sm === sm.wb_1block) {
    when(wb_block_index =/= (4 * 1024 - 1).U){
      when(region_counter(wb_block_index + 1.U) =/= 0.U){
        wb_sm := sm.wb_level
      }.otherwise{
        wb_sm := sm.wb_1block
      }
    }.otherwise{
      wb_sm := sm.idole
    }
  }
  when(wb_sm === sm.idole || wb_sm === sm.wb_1block){
    count := 0.U
  }.elsewhen(wb_sm === sm.wb_level && aw_buffer.io.full === false.B && w_buffer.io.full === false.B){
    count := count + 1.U
  }

  buffer.io.enb := true.B
  buffer.io.web := false.B
  buffer.io.addrb := Mux1H(Seq(
    flush_start -> get_level_addr(0.U, count),
    (io.xbar_in.valid && io.xbar_in.ready && region_counter(block_index) === 63.U) -> get_level_addr(block_index, count),
    (wb_sm === sm.wb_level) -> get_level_addr(wb_block_index, count)
  ))
  buffer.io.dinb := 0.U
  buffer.io.clkb := clock.asBool()

  aw_buffer.io.wr_en := wb_sm === sm.wb_level && w_buffer.io.full === false.B
  aw_buffer.io.din := io.level_base_addr + Cat(wb_block_index, buffer.io.doutb(13 + 32,32)).asUInt()
  io.ddr_aw.bits.awaddr := aw_buffer.io.dout
  io.ddr_aw.bits.awlock := 0.U
  io.ddr_aw.bits.awid := aw_buffer.io.data_count
  io.ddr_aw.bits.awlen := 0.U
  io.ddr_aw.bits.awburst := 1.U(2.W)
  io.ddr_aw.bits.awsize := 2.U
  io.ddr_aw.valid := aw_buffer.is_valid()
  aw_buffer.io.rd_en := io.ddr_aw.ready

  w_buffer.io.wr_en := wb_sm === sm.wb_level && aw_buffer.io.full === false.B
  w_buffer.io.din := buffer.io.doutb(31, 0)
  io.ddr_w.bits.wdata := w_buffer.io.dout
  io.ddr_w.bits.wlast := true.B
  io.ddr_w.valid := w_buffer.is_valid()
  io.ddr_w.bits.wstrb := 0xf.U
  w_buffer.io.rd_en := io.ddr_w.ready

  io.ddr_b.ready := true.B

  io.end := wb_sm === sm.wb_1block && wb_block_index === (4 * 1024 - 1).U
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
    val end = Output(Bool())
    val flush = Input(Bool())
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
  io.end := update_engine.io.end
  update_engine.io.flush := io.flush & vertex_update_buffer.is_empty()
}

//TODO: use axis_width_converter IP
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
    val gather_out = Vec(2, Decoupled(new axisdata(4, 4)))

    //control path
    val signal = Input(Bool())
  })
  val Selector = Module(new axis_arbitrator(4, 16, 4))
  Selector.io.xbar_in <> io.ddr_in

  val broadcaster = Module(new axis_broadcaster(4, 2, "gather_broadcaster"))
  broadcaster.io.s_axis.connectfrom(Selector.io.ddr_out.bits)
  broadcaster.io.s_axis.tvalid := Selector.io.ddr_out.valid
  Selector.io.ddr_out.ready := broadcaster.io.s_axis.tready
  broadcaster.io.aresetn := ~reset.asBool()
  broadcaster.io.aclk := clock.asBool()
  io.gather_out.zipWithIndex.map{
    case (gather, i) => {
      broadcaster.io.m_axis.connectto(gather.bits, i)
      gather.valid := broadcaster.io.m_axis.tvalid(i)
    }
  }
  broadcaster.io.m_axis.tready := VecInit.tabulate(2)(i => io.gather_out(i).ready).asUInt()
}

//TODO: add read cache
class readEdge_engine(AXI_ADDR_WIDTH : Int = 64, AXI_DATA_WIDTH: Int = 64, AXI_ID_WIDTH: Int = 6, AXI_SIZE_WIDTH: Int = 3, EMBEDDING : Int = 14) extends Module{
  val io = IO(new Bundle() {
    val in = Flipped(Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH)))
    val out = Decoupled(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1))
    //control signal
    val edge_base_addr = Input(UInt(64.W))
    val free_ptr = Input(UInt((AXI_ID_WIDTH-1).W))
    val read_edge_num = Output(UInt(32.W))
  })

  def get_edge_array_index(mdata : UInt) : UInt = {
    mdata(31, 0)
  }

  def get_edge_count(mdata : UInt) : UInt = {
    mdata(63, 32)
  }

  //read edge array
  val edge_read_buffer = Module(new BRAM_fifo(32, 64, "meta_fifo"))
  edge_read_buffer.io.clk := clock.asBool()
  edge_read_buffer.io.srst := reset.asBool()
  edge_read_buffer.io.din := io.in.bits.rdata(8 * 8 - 1, 0)
  edge_read_buffer.io.wr_en := !io.in.bits.rid(AXI_ID_WIDTH - 1) & io.in.valid & (get_edge_count(io.in.bits.rdata) > EMBEDDING.asUInt())
  io.in.ready := edge_read_buffer.io.full === false.B

  val remainning_edges = (get_edge_count(edge_read_buffer.io.dout) - EMBEDDING.asUInt())
  // TODO we assume number of edges of a vertex is less than 4096 + 14 for now.
  object cache_sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_page  = Value(0x1.U) // i "load"  -> 000_0011
  }
  val cache_status = RegInit(cache_sm.idole)
  val counter = RegInit(0.U(32.W))
  val next_counter = Wire(UInt(32.W))
  val araddr = RegInit(0.U(AXI_ADDR_WIDTH.W))
  next_counter := counter - 1024.U
  when(remainning_edges.asUInt() > 1024.U && io.out.ready && io.out.valid && cache_status === cache_sm.idole){
    cache_status := cache_sm.next_page
    counter := remainning_edges - 1024.U
    araddr := io.edge_base_addr + (get_edge_array_index(edge_read_buffer.io.dout) << 2).asUInt() + 4096.U
  }.elsewhen(io.out.ready && io.out.valid && cache_status === cache_sm.next_page){
    when(counter <= 1024.U){
      cache_status := cache_sm.idole
      counter := 0.U
      araddr := 0.U
    }.otherwise{
      counter := next_counter
      araddr := araddr + 4096.U
    }
  }

  val num_vertex = MuxCase(1024.U(32.W), Array(
    (cache_status === cache_sm.idole && remainning_edges <= 1024.U) -> (remainning_edges),
    (cache_status === cache_sm.next_page && counter <= 1024.U) -> counter
  ))
  var arlen = Mux(((num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)) << log2Ceil(AXI_DATA_WIDTH/4)).asUInt() < num_vertex,
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)),
    (num_vertex >> log2Ceil(AXI_DATA_WIDTH/4)).asUInt() - 1.U)
  io.out.bits.araddr := Mux(cache_status === cache_sm.idole,
    io.edge_base_addr + (get_edge_array_index(edge_read_buffer.io.dout) << 2),
    araddr)
  io.out.valid := Mux(cache_status === cache_sm.idole, edge_read_buffer.is_valid(), true.B)
  io.out.bits.arlen := arlen.asUInt()
  io.out.bits.arburst := 1.U(2.W)
  io.out.bits.arlock := 0.U
  io.out.bits.arsize := log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  io.out.bits.arid := Cat(1.U(1.W), io.free_ptr)
  edge_read_buffer.io.rd_en := io.out.ready & cache_status === cache_sm.idole
  io.read_edge_num := num_vertex
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

  val edge_cache = Module(new readEdge_engine(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, EMBEDDING))
  edge_cache.io.edge_base_addr := io.edge_base_addr
  val r_demux = Module(new axis_broadcaster(AXI_DATA_WIDTH, 2, "r_demux", AXI_ID_WIDTH))
  val r_demux_out = Wire(Vec(2, Decoupled(new axir(AXI_DATA_WIDTH, AXI_ID_WIDTH))))
  r_demux.io.s_axis.tvalid := io.ddr_r.valid
  r_demux.io.s_axis.tdata := io.ddr_r.bits.rdata
  r_demux.io.s_axis.tlast := io.ddr_r.bits.rlast
  r_demux.io.s_axis.disable_tkeep()
  r_demux.io.s_axis.tid := io.ddr_r.bits.rid
  r_demux.io.aclk := clock.asBool()
  r_demux.io.aresetn := ~reset.asBool()
  io.ddr_r.ready := r_demux.io.s_axis.tready
  r_demux_out.zipWithIndex.map{
    case (r, i) => {
      r.valid := r_demux.io.m_axis.tvalid(i)
      r.bits.rid := r_demux.io.m_axis.tid(AXI_ID_WIDTH * (i + 1) - 1, AXI_ID_WIDTH * i)
      r.bits.rlast := r_demux.io.m_axis.tlast(i)
      r.bits.rdata := r_demux.io.m_axis.tdata(8 * AXI_DATA_WIDTH * (i + 1) - 1, 8 * AXI_DATA_WIDTH * i)
    }
  }
  r_demux.io.m_axis.tready := VecInit.tabulate(2)(i => r_demux_out(i).ready).asUInt()
  edge_cache.io.in <> r_demux_out(1)

  val arbi = Module(new AMBA_Arbiter(new axiar(AXI_ADDR_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH, 1), 2))
  io.ddr_ar <> arbi.io.out
  arbi.io.in(1).bits.araddr :=  io.embedding_base_addr + (vertex_read_buffer.io.dout << log2Ceil(4 * EMBEDDING + 8))
  arbi.io.in(1).valid :=  vertex_read_buffer.is_valid() && vertex_read_buffer.io.dout(31) === 0.U
  arbi.io.in(1).bits.arlen := ((4 * EMBEDDING + 8) / AXI_DATA_WIDTH - 1).asUInt()
  arbi.io.in(1).bits.arburst := 1.U(2.W)
  arbi.io.in(1).bits.arlock := 0.U
  arbi.io.in(1).bits.arsize :=  log2Ceil(AXI_DATA_WIDTH).U(AXI_SIZE_WIDTH.W)
  arbi.io.in(1).bits.arid :=  Cat(0.U(1.W), vertex_read_buffer.io.data_count.asTypeOf(UInt((AXI_ID_WIDTH-1).W)))
  vertex_read_buffer.io.rd_en := arbi.io.in(1).ready | upward_status === upward_sm.output_fin
  arbi.io.in(0) <> edge_cache.io.out

  //front end of down ward
  //TODO we should ensure regfile is big enough in case the data return slowly
  val num_regfile = Module(new regFile(32, 32))
  num_regfile.io.writeFlag := arbi.io.in(0).ready & arbi.io.in(0).valid
  num_regfile.io.wptr := num_regfile.io.wcount
  num_regfile.io.dataIn := edge_cache.io.read_edge_num
  num_regfile.io.rptr := io.ddr_r.bits.rid(AXI_ID_WIDTH-2, 0)
  edge_cache.io.free_ptr := num_regfile.io.wcount

  //back end of down ward
  object sm extends ChiselEnum {
    //val idole = Value(0x0.U)
    val firstBurst  = Value(0x1.U) // i "load"  -> 000_0011
    val remainingBurst   = Value(0x2.U) // i "imm"   -> 001_0011
  }
  val status = RegInit(sm.firstBurst)
  val num = RegInit(0.U(32.W))
  when(status === sm.firstBurst && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool()){
    when(r_demux_out(0).bits.rlast.asBool()){
      status := sm.firstBurst
      num := 0.U
    }.otherwise{
      num := num_regfile.io.dataOut
      status := sm.remainingBurst
    }
  }.elsewhen(status === sm.remainingBurst && r_demux_out(0).valid.asBool() && r_demux_out(0).ready.asBool()){
    when(r_demux_out(0).bits.rlast.asBool()){
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
    case(k, i) => k := Mux(r_demux_out(0).bits.rid(AXI_ID_WIDTH-1), Mux(status === sm.firstBurst, num_regfile.io.dataOut > i.U, num > i.U),
      Mux(i.U < 2.U, false.B, (edge_cache.get_edge_count(r_demux_out(0).bits.rdata) + 2.U) > i.U))
  }

  val vertex_out_fifo = Module(new BRAM_fifo(32, 512 + 16, "edge_fifo"))
  vertex_out_fifo.io.srst := reset.asBool()
  vertex_out_fifo.io.clk := clock.asBool()
  io.xbar_out.valid := vertex_out_fifo.is_valid()
  io.xbar_out.bits.tkeep := vertex_out_fifo.io.dout(15, 0)
  io.xbar_out.bits.tdata := vertex_out_fifo.io.dout(512 + 16 - 1, 16)
  io.xbar_out.bits.tlast := true.B
  vertex_out_fifo.io.rd_en := io.xbar_out.ready
  vertex_out_fifo.io.wr_en := r_demux_out(0).valid | upward_status === upward_sm.output_fin
  vertex_out_fifo.io.din := Mux(upward_status === upward_sm.output_fin, Cat(vertex_read_buffer.io.dout, 0x1.U(16.W)), Cat(r_demux_out(0).bits.rdata, keep.asUInt()))
  r_demux_out(0).ready := vertex_out_fifo.io.full === false.B

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
    traveled_edges_reg := traveled_edges_reg + edge_cache.get_edge_count(io.ddr_r.bits.rdata)
  }

  when(io.ddr_ar.valid && io.ddr_ar.ready
    && io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool()){
    inflight_vtxs := inflight_vtxs
  }.elsewhen(io.ddr_ar.valid && io.ddr_ar.ready){
    inflight_vtxs := inflight_vtxs + 1.U
  }.elsewhen(io.ddr_r.valid.asBool() && io.ddr_r.ready.asBool() && io.ddr_r.bits.rlast.asBool()){
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

  val xbar = Module(new axis_broadcaster(AXIS_DATA_WIDTH, SLAVE_NUM, "axis_broadcaster"))
  xbar.io.aclk := clock.asBool()
  xbar.io.aresetn := ~reset.asBool()
  xbar.io.s_axis.connectfrom(arbitrator.io.out.bits)
  xbar.io.s_axis.tvalid := arbitrator.io.out.valid
  arbitrator.io.out.ready := xbar.io.s_axis.tready
  io.pe_out.zipWithIndex.map{
    case(pe, i) => {
      xbar.io.m_axis.connectto(pe.bits, i)
      pe.valid := xbar.io.m_axis.tvalid(i)
    }
  }
  xbar.io.m_axis.tready := (VecInit.tabulate(SLAVE_NUM)(
    i => io.pe_out(i).ready
  )).asUInt()
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
    vid(3, 0) === sid
  }

  def vid2bitmap_addr(vid: UInt) : UInt = {
    vid(31, 4)
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
  val bitmap_wait = Module(new pipeline(UInt(32.W)))
  bitmap_wait.io.din.valid := bitmap_arvalid
  bitmap_wait.io.din.bits := vertex_in_fifo.io.dout
  bitmap_wait.io.dout.ready := !halt
  val bitmap_write = Module(new pipeline(UInt(32.W)))
  bitmap_write.io.din.valid := bitmap_wait.io.dout.valid && (bitmap.io.doutb =/= 1.U(1.W) | bitmap_wait.io.dout.bits(31) === 1.U(1.W))
  bitmap_write.io.din.bits := bitmap_wait.io.dout.bits
  bitmap_write.io.dout.ready := !halt

  vertex_in_fifo.io.din := arbi.io.ddr_out.bits.tdata
  vertex_in_fifo.io.wr_en := arbi.io.ddr_out.valid
  arbi.io.ddr_out.ready := vertex_in_fifo.io.full === false.B
  bitmap.io.enb :=  true.B//bitmap_arvalid
  bitmap.io.addrb := vid2bitmap_addr(vertex_in_fifo.io.dout)
  bitmap.io.clkb := clock.asBool()
  vertex_in_fifo.io.rd_en := !halt
  vertex_out_fifo.io.wr_en := bitmap_write.io.dout.valid
  vertex_out_fifo.io.din := bitmap_write.io.dout.bits
  bitmap.io.ena := true.B
  bitmap.io.wea := bitmap_write.io.dout.valid & bitmap_write.io.dout.bits(31) === 0.U(1.W) | write_root
  bitmap.io.clka := clock.asBool()
  bitmap.io.dina := 1.U
  bitmap.io.addra := Mux(write_root, vid2bitmap_addr(io.root), vid2bitmap_addr(bitmap_write.io.dout.bits))
  vertex_out_fifo.io.rd_en := io.ddr_out.ready //| vertex_out_fifo.test_FIN()
  io.ddr_out.valid := vertex_out_fifo.is_valid() & vertex_out_fifo.io.dout(31) === 0.U
  io.ddr_out.bits.tkeep := true.B
  io.ddr_out.bits.tlast := true.B
  io.ddr_out.bits.tdata := vertex_out_fifo.io.dout

  //control path
  io.end := vertex_out_fifo.test_FIN()
}

//TODO: Try to use fifoIO
class multi_channel_fifo(AXI_DATA_WIDTH: Int = 64, size : Int = 16) extends Module{
  val io = IO(new Bundle() {
    val in = Vec(16, Flipped(Decoupled(new axisdata(4, 4))))
    val out = new Bundle() {
      val full = Output(Bool())
      val din = Input(UInt((8 * AXI_DATA_WIDTH).W))
      val wr_en = Input(Bool())
      val empty = Output(Bool())
      val dout = Output(UInt((8 * AXI_DATA_WIDTH).W))
      val rd_en = Input(Bool())
      val data_count = Output(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))
      val valid = Output(Bool())
    }
    val flush = Input(Bool())
  })

  def is_empty() : Bool = {
    io.out.data_count === 0.U
  }

  def is_valid() : Bool = {
    io.out.valid
  }

  val collector_fifos = Seq.tabulate(16)(
    i => Module(new BRAM_fifo(16, 32, ("collector_fifo_0")))
  )
  val fifos_ready = Seq.tabulate(16)(i => collector_fifos(i).io.full === false.B).reduce(_&_)
  val counter = RegInit(0.U(log2Ceil(16).W))
  val collector_data = Wire(Vec(16, UInt(32.W)))
  val sorted_data = Wire(Vec(16, UInt(32.W)))
  val sorted_valid = Wire(Vec(16, Bool()))
  val in_pipeline = Seq.fill(16)(
    Module(new pipeline((new axisdata(4, 4))))
  )
  in_pipeline.zipWithIndex.map{
    case(p, i) => {
      p.io.din <> io.in(i)
      p.io.dout.ready := Mux(io.out.wr_en, false.B, fifos_ready)
    }
  }
  val steps = (Seq.tabulate(16)(
    i => {
      Seq.tabulate(i + 1)(x => in_pipeline(x).io.dout.valid.asTypeOf(UInt(4.W))).reduce(_+_)
    }
  ))

  def indexAdd(index : UInt, step : UInt) : UInt = {
    Mux(index + step >= 16.U, index + step - 16.U, index + step)
  }
  def indexSub(index1 : UInt, index2 : UInt) : UInt = {
    Mux(index1 <= index2, index2 - index1, 16.U - index1 + index2)
  }

  sorted_data.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(0.U,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, in_pipeline(x).io.dout.bits.tdata)
        ))
    }
  }
  sorted_valid.zipWithIndex.map{
    case (s, i) => {
      s := MuxCase(false.B,
        Array.tabulate(16)(
          x => (steps(x) === (i + 1).U, in_pipeline(x).io.dout.valid & fifos_ready)
        ))
    }
  }

  when(sorted_valid.reduce(_|_)){
    counter := indexAdd(counter, MuxCase(0.U, Seq.tabulate(16)(
      x => (sorted_valid(16 - x - 1), (16 - x).U)
    )))
  }.elsewhen(io.flush){
    counter := 0.U
  }

  collector_fifos.zipWithIndex.map{
    case (f, i) => {
      val fifo_in_data = Mux1H(Seq.tabulate(16)(x => (indexSub(counter, i.U) === x.U, sorted_data(x))))
      val fifo_in_valid = Mux1H(Seq.tabulate(16)(x => (indexSub(counter, i.U) === x.U, sorted_valid(x))))
      f.io.din := Mux(io.out.wr_en, io.out.din(32 * (i + 1) - 1, 32 * i), fifo_in_data)
      f.io.wr_en := Mux(io.out.wr_en, io.out.wr_en ,fifo_in_valid)
      collector_data(i) := f.io.dout
      f.io.clk := clock.asBool()
      f.io.srst := reset.asBool()
      f.io.rd_en := io.out.rd_en
    }
  }

  io.out.valid := Seq.tabulate(16)(i => collector_fifos(i).is_valid()).reduce(_|_)
  io.out.dout := collector_data.asUInt()
  io.out.data_count := collector_fifos.map{i => i.io.data_count.asTypeOf(UInt((log2Ceil(size * AXI_DATA_WIDTH / 4) + 1).W))}.reduce(_+_)
  io.out.full := ~fifos_ready
  io.out.empty := collector_fifos.map{i => i.io.empty}.reduce(_&_)
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

  val tier_fifo = Seq.tabulate(2)(i => Module(new multi_channel_fifo(AXI_DATA_WIDTH, 16)))
  val tier_counter = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))

  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val next_tier_is_0  = Value(0x1.U) // i "load"  -> 000_0011
    val next_tier_is_1   = Value(0x2.U) // i "imm"   -> 001_0011
    val writeback = Value(0x5.U)
    val readback = Value(0x6.U)
    val start = Value(0x7.U)
    val sync_send_0 = Value(0x8.U)
    val sync_send_1 = Value(0x9.U)
    val signal_wait_0 = Value(0x10.U)
    val signal_wait_1 = Value(0x11.U)
    val writebackdata = Value(0x12.U)
    val readbackdata = Value(0x13.U)
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
    status := sm.next_tier_is_0
  }.elsewhen(status === sm.next_tier_is_0 && tier_counter(1) === 0.U){
    status := sm.sync_send_1
  }.elsewhen(status === sm.sync_send_1 && io.cacheable_out.valid && io.cacheable_out.ready){
    status := sm.signal_wait_1
  }.elsewhen(io.signal && status === sm.signal_wait_1){
    status := sm.next_tier_is_1
  }.elsewhen(io.end){
    status := sm.idole
  }

  val next_tier_mask = Cat(status === sm.next_tier_is_1 | status === sm.sync_send_0 | status === sm.signal_wait_0,
      status === sm.next_tier_is_0 | status === sm.sync_send_1 | status === sm.signal_wait_1)
  val axi = Wire(Flipped(new axidata(AXI_ADDR_WIDTH, AXI_DATA_WIDTH, AXI_ID_WIDTH, AXI_SIZE_WIDTH)))
  val tier_base_addr = RegInit(VecInit(Seq.fill(2)(0.U(AXI_ADDR_WIDTH.W))))
  val tier_status = RegInit(VecInit(Seq.fill(2)(sm.idole)))
  val step_fin = io.signal && (status === sm.signal_wait_0 | status === sm.signal_wait_1)
  tier_base_addr.zipWithIndex.map{
    case(a, i) => {
      when((io.start && status === sm.idole) | step_fin){
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
        c := Mux(tier_fifo(i).io.out.data_count > 16.U, c - 16.U, c - tier_fifo(i).io.out.data_count)
      }
    }
  }
  tier_fifo.zipWithIndex.map{
    case(f, i) => {
      f.io.out.rd_en := Mux(next_tier_mask(i), axi.w.ready, io.cacheable_out.ready)
      f.io.out.wr_en := Mux(next_tier_mask(i), 0.U, axi.r.valid)
      f.io.out.din := Mux(next_tier_mask(i), 0.U, axi.r.bits.rdata)
      f.io.flush := io.signal && (status === sm.signal_wait_0 || status === sm.signal_wait_1)
      f.io.in.zipWithIndex.map{
        case(in, x) => {
          in.valid := Mux(next_tier_mask(i), io.cacheable_in(x).valid, false.B)
          in.bits := Mux(next_tier_mask(i), io.cacheable_in(x).bits, 0.U.asTypeOf(in.bits.cloneType))
        }
      }
    }
  }
  tier_status.zipWithIndex.map{
    case (s, i) => {
      when(next_tier_mask(i) && tier_fifo(i).io.out.full && s === sm.idole){
        s := sm.writeback
      }.elsewhen(!next_tier_mask(i) && tier_fifo(i).is_empty() && tier_counter(i) =/= 0.U && s === sm.idole){
        s := sm.readback
      }.elsewhen(s === sm.writeback && axi.aw.ready){
        s := sm.writebackdata
      }.elsewhen(s === sm.readback && axi.ar.ready){
        s := sm.readbackdata
      }.elsewhen(s === sm.readbackdata && axi.r.bits.rlast.asBool() && axi.r.ready && axi.r.valid){
        s := sm.idole
      }.elsewhen(s === sm.writebackdata && axi.w.bits.wlast.asBool()){
        s := sm.idole
      }
    }
  }

  io.cacheable_in.zipWithIndex.map{
    case (in, i) => {
      in.ready := Mux(next_tier_mask(0), tier_fifo(0).io.in(i).ready, tier_fifo(1).io.in(i).ready)
    }
  }
  io.unvisited_size := Mux(next_tier_mask(0), tier_counter(0), tier_counter(1))
  io.cacheable_out.valid := MuxCase(status === sm.start,
    Array(next_tier_mask(0) -> (tier_fifo(1).is_valid() | status === sm.sync_send_1),
          next_tier_mask(1) -> (tier_fifo(0).is_valid() | status === sm.sync_send_0)))
  io.cacheable_out.bits.tdata := MuxCase(io.root.asTypeOf(UInt(512.W)),
    Array((status === sm.sync_send_0) -> "x80000000".asUInt(512.W),
          (status === sm.sync_send_1) -> "x80000000".asUInt(512.W),
          next_tier_mask(0) -> tier_fifo(1).io.out.dout,
          next_tier_mask(1) -> tier_fifo(0).io.out.dout))
  io.cacheable_out.bits.tkeep := Mux(status === sm.start | status === sm.sync_send_0 | status === sm.sync_send_1, 1.U(16.W),
    VecInit(Seq.tabulate(16)(
      i => Mux(next_tier_mask(0), tier_fifo(1).io.out.data_count > i.U, tier_fifo(0).io.out.data_count > i.U)
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
  axi.ar.valid := Mux(next_tier_mask(0), tier_status(1) === sm.readback, tier_status(0) === sm.readback)
  axi.w.bits.wdata := Mux(next_tier_mask(0), tier_fifo(0).io.out.dout, tier_fifo(1).io.out.dout)
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
    val flush_cache = Output(Bool())
    val flush_cache_end = Input(Bool())
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
  //14 --- clock(31, 0)
  //15 --- clock(63, 21)
  val controls = Module(new LookupTable(depth = 32, AXI_ADDR_WIDTH = AXI_ADDR_WIDTH))
  val level = RegInit(0.U(32.W))
  object sm extends ChiselEnum {
    val idole = Value(0x0.U)
    val exe  = Value(0x1.U) // i "load"  -> 000_0011
    val fin   = Value(0x2.U) // i "imm"   -> 001_0011
    val end = Value(0x3.U)
    val start = Value(0x4.U)
    val flush_cache = Value(0x5.U)
    val write_clock = Value(0x6.U)
  }
  val status = RegInit(sm.idole)
  val start = controls.io.data(0)(0)
  val FIN = RegInit(VecInit(Seq.fill(16)(false.B)))
  val new_tep = Cat(controls.io.data(12), controls.io.data(13)) + io.traveled_edges
  val counterValue = RegInit(0.U(64.W))

  controls.config <> io.config
  controls.io.writeFlag(0) := status === sm.end | status === sm.fin | status === sm.write_clock
  controls.io.wptr(0) := Mux1H(Seq(
    (status === sm.fin) -> 12.U,
    (status === sm.write_clock) -> 14.U,
    (status === sm.end) -> 0.U
  ))
  controls.io.dataIn(0) := Mux1H(Seq(
    (status === sm.fin) -> new_tep(63, 32),
    (status === sm.write_clock) -> counterValue(31, 0),
    (status === sm.end) -> 2.U
  ))
  controls.io.writeFlag(1) := status === sm.fin | status === sm.write_clock
  controls.io.wptr(1) := Mux(status === sm.fin, 13.U, 15.U)
  controls.io.dataIn(1) := Mux1H(Seq(
    (status === sm.fin) -> new_tep(31, 0),
    (status === sm.write_clock) -> counterValue(63, 32)
  ))

  when(status === sm.idole && start){
    status := sm.start
  }.elsewhen(status === sm.start){
    status := sm.exe
  }.elsewhen(status === sm.exe && FIN.reduce(_&_)){
    status := sm.fin
  }.elsewhen(status === sm.fin){
    when(io.unvisited_size === 0.U){
      status := sm.write_clock
    }.otherwise{
      status := sm.exe
    }
  }.elsewhen(status === sm.flush_cache && io.flush_cache_end){
    status := sm.end
  }.elsewhen(status === sm.write_clock){
    status := sm.flush_cache
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

  when(status === sm.start) {
    counterValue := 0.U
  }.otherwise{
    counterValue := counterValue + 1.U
  }

  io.signal := status === sm.start || status === sm.fin
  io.data := controls.io.data
  io.level := level
  io.start := status === sm.start
  io.flush_cache := status === sm.flush_cache
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
  Applys.io.gather_in <> Gathers.io.gather_out(0)
  Broadcasts.io.gather_in <> Gathers.io.gather_out(1)
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
  controls.io.flush_cache_end := Applys.io.end
  Applys.io.flush := controls.io.flush_cache
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